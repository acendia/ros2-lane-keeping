"""Lane perception from the forward camera."""

from dataclasses import dataclass
import math
from typing import Optional

from auto_drive_sim_interfaces.msg import LaneEstimate
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


@dataclass(frozen=True)
class CameraModel:
    """Fixed forward camera used by the first-release car."""

    width: int = 640
    height: int = 360
    horizontal_fov_rad: float = 1.5708
    height_m: float = 1.3
    pitch_rad: float = 0.05
    x_offset_m: float = 1.4

    @property
    def fx(self) -> float:
        """Horizontal focal length in pixels."""
        return (self.width / 2.0) / math.tan(self.horizontal_fov_rad / 2.0)

    @property
    def fy(self) -> float:
        """Vertical focal length in pixels."""
        return self.fx

    @property
    def cx(self) -> float:
        """Principal point column."""
        return self.width / 2.0

    @property
    def cy(self) -> float:
        """Principal point row."""
        return self.height / 2.0


@dataclass(frozen=True)
class PerceptionConfig:
    """Thresholds and validation limits for lane detection."""

    target_distance_m: float = 4.0
    roi_min_forward_m: float = 3.5
    roi_max_forward_m: float = 12.0
    roi_half_width_m: float = 5.0
    min_component_area_px: int = 100
    min_boundary_points: int = 50
    min_boundary_span_m: float = 2.5
    min_separation_m: float = 2.8
    max_separation_m: float = 4.5
    max_fit_residual_m: float = 0.25
    max_fit_rms_m: float = 0.15
    max_center_offset_m: float = 2.5
    white_value_min: int = 180
    white_saturation_max: int = 60
    max_image_age_s: float = 0.3


@dataclass(frozen=True)
class BoundaryFit:
    """Quadratic fit y(x) for one lane boundary."""

    coefficients: np.ndarray
    rms_m: float
    point_count: int
    x_min_m: float
    x_max_m: float

    def evaluate(self, x_m: float) -> float:
        """Evaluate the boundary lateral position at a forward distance."""
        return float(np.polyval(self.coefficients, x_m))


@dataclass(frozen=True)
class PerceptionResult:
    """One lane estimate and the validation outcome."""

    valid: bool
    confidence: float
    lateral_error_m: float
    target_x_m: float
    target_y_m: float
    lane_width_m: float
    reason: str
    left_fit: Optional[BoundaryFit] = None
    right_fit: Optional[BoundaryFit] = None


def _invalid(reason: str) -> PerceptionResult:
    return PerceptionResult(
        valid=False,
        confidence=0.0,
        lateral_error_m=0.0,
        target_x_m=0.0,
        target_y_m=0.0,
        lane_width_m=0.0,
        reason=reason,
    )


class GroundPlane:
    """Pixel and ground-plane mapping for a fixed camera."""

    def __init__(self, camera: CameraModel) -> None:
        """Precompute the per-pixel ground projection."""
        self.camera = camera
        u, v = np.meshgrid(
            np.arange(camera.width, dtype=np.float64),
            np.arange(camera.height, dtype=np.float64))
        right = (u - camera.cx) / camera.fx
        down = (v - camera.cy) / camera.fy
        forward = np.ones_like(right)
        left = -right
        up = -down

        cos_pitch = math.cos(camera.pitch_rad)
        sin_pitch = math.sin(camera.pitch_rad)
        ray_x = forward * cos_pitch + up * sin_pitch
        ray_y = left
        ray_z = -forward * sin_pitch + up * cos_pitch
        scale = -camera.height_m / ray_z
        self._x = scale * ray_x
        self._y = scale * ray_y
        self._valid = (ray_z < -1e-6) & (self._x > 0.0)

    def points_for_mask(self, mask: np.ndarray) -> tuple:
        """Project white pixels to ground coordinates in the camera frame."""
        selected = (mask > 0) & self._valid
        return self._x[selected], self._y[selected]

    def project(self, x_m: float, y_m: float) -> tuple:
        """Project a camera-frame ground point back to a pixel."""
        cos_pitch = math.cos(self.camera.pitch_rad)
        sin_pitch = math.sin(self.camera.pitch_rad)
        forward = x_m * cos_pitch - self.camera.height_m * sin_pitch
        left = y_m
        up = x_m * sin_pitch - self.camera.height_m * cos_pitch
        if forward <= 1e-6:
            return None
        u = self.camera.cx - (left / forward) * self.camera.fx
        v = self.camera.cy - (up / forward) * self.camera.fy
        return int(round(u)), int(round(v))


def threshold_markings(image: np.ndarray,
                       config: PerceptionConfig) -> np.ndarray:
    """Return a binary mask of bright, low-saturation markings."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    return cv2.inRange(
        hsv,
        (0, 0, config.white_value_min),
        (180, config.white_saturation_max, 255))


def extract_boundary_points(mask: np.ndarray, ground: GroundPlane,
                            config: PerceptionConfig) -> tuple:
    """Find the two largest marking components and split them by side."""
    count, labels, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    components = []
    for label in range(1, count):
        if stats[label, cv2.CC_STAT_AREA] < config.min_component_area_px:
            continue
        component_mask = (labels == label).astype(np.uint8)
        x_values, y_values = ground.points_for_mask(component_mask)
        keep = (
            (x_values >= config.roi_min_forward_m) &
            (x_values <= config.roi_max_forward_m) &
            (np.abs(y_values) <= config.roi_half_width_m))
        x_values = x_values[keep]
        y_values = y_values[keep]
        if x_values.size >= config.min_boundary_points:
            components.append((x_values, y_values))

    components.sort(key=lambda component: component[0].size, reverse=True)
    if len(components) < 2:
        empty = (np.array([]), np.array([]))
        return empty, empty

    first = components[0]
    second = components[1]
    if first[1].mean() > second[1].mean():
        return (first[0], first[1]), (second[0], second[1])
    return (second[0], second[1]), (first[0], first[1])


def fit_boundary(x_values: np.ndarray, y_values: np.ndarray,
                 config: PerceptionConfig) -> Optional[BoundaryFit]:
    """Robustly fit a quadratic boundary and reject weak fits."""
    if x_values.size < config.min_boundary_points:
        return None
    coefficients = np.polyfit(x_values, y_values, 2)
    inliers = np.ones(x_values.size, dtype=bool)
    for _ in range(3):
        residuals = y_values - np.polyval(coefficients, x_values)
        inliers = np.abs(residuals) <= config.max_fit_residual_m
        if inliers.sum() < config.min_boundary_points:
            return None
        coefficients = np.polyfit(
            x_values[inliers], y_values[inliers], 2)
    residuals = y_values[inliers] - np.polyval(
        coefficients, x_values[inliers])
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    span = float(x_values[inliers].max() - x_values[inliers].min())
    if rms > config.max_fit_rms_m or span < config.min_boundary_span_m:
        return None
    return BoundaryFit(
        coefficients=coefficients,
        rms_m=rms,
        point_count=int(inliers.sum()),
        x_min_m=float(x_values[inliers].min()),
        x_max_m=float(x_values[inliers].max()),
    )


def validate_boundaries(left: BoundaryFit, right: BoundaryFit,
                        config: PerceptionConfig) -> str:
    """Return an empty string or the reason the pair is implausible."""
    width = left.evaluate(config.target_distance_m) - \
        right.evaluate(config.target_distance_m)
    if width <= 0.0:
        return 'swapped_boundaries'
    if width < config.min_separation_m:
        return 'lane_too_narrow'
    if width > config.max_separation_m:
        return 'lane_too_wide'
    center = 0.5 * (
        left.evaluate(config.target_distance_m) +
        right.evaluate(config.target_distance_m))
    if abs(center) > config.max_center_offset_m:
        return 'lane_offset_too_large'
    return ''


def is_stale(image_stamp_s: float, now_s: float, max_age_s: float) -> bool:
    """Return true when an image timestamp is too old or too far ahead."""
    age = now_s - image_stamp_s
    return age > max_age_s or age < -max_age_s


def analyze_frame(image: np.ndarray, ground: GroundPlane,
                  config: PerceptionConfig) -> PerceptionResult:
    """Run the pure lane pipeline on one BGR frame."""
    mask = threshold_markings(image, config)
    (left_x, left_y), (right_x, right_y) = extract_boundary_points(
        mask, ground, config)
    if left_x.size == 0 and right_x.size == 0:
        return _invalid('no_lane_markings')
    if left_x.size == 0 or right_x.size == 0:
        return _invalid('one_boundary')

    left = fit_boundary(left_x, left_y, config)
    right = fit_boundary(right_x, right_y, config)
    if left is None or right is None:
        return _invalid('sparse_boundaries')

    reason = validate_boundaries(left, right, config)
    if reason:
        return _invalid(reason)

    target_x = config.target_distance_m
    left_y_at_target = left.evaluate(target_x)
    right_y_at_target = right.evaluate(target_x)
    lane_width = left_y_at_target - right_y_at_target
    center = 0.5 * (left_y_at_target + right_y_at_target)
    point_score = min(1.0, min(left.point_count, right.point_count) / 200.0)
    residual_score = max(
        0.0, 1.0 - max(left.rms_m, right.rms_m) / config.max_fit_rms_m)
    width_score = max(0.0, 1.0 - abs(lane_width - 3.62) / 0.7)
    confidence = 0.35 * point_score + 0.35 * residual_score + \
        0.30 * width_score

    return PerceptionResult(
        valid=True,
        confidence=round(float(confidence), 3),
        lateral_error_m=round(float(center), 4),
        target_x_m=round(
            float(ground.camera.x_offset_m + target_x), 4),
        target_y_m=round(float(center), 4),
        lane_width_m=round(float(lane_width), 4),
        reason='',
        left_fit=left,
        right_fit=right,
    )


def make_debug_image(image: np.ndarray, result: PerceptionResult,
                     ground: GroundPlane,
                     config: PerceptionConfig) -> np.ndarray:
    """Draw the threshold, boundary fits, and lookahead target."""
    debug = image.copy()
    mask = threshold_markings(image, config)
    mask[:int(image.shape[0] * 0.5), :] = 0
    tint = np.zeros_like(image)
    tint[mask > 0] = (0, 200, 0)
    debug = cv2.addWeighted(debug, 1.0, tint, 0.35, 0.0)

    if result.left_fit is not None and result.right_fit is not None:
        distances = np.linspace(
            config.roi_min_forward_m, config.roi_max_forward_m, 24)
        for fit, color in ((result.left_fit, (0, 255, 0)),
                           (result.right_fit, (0, 0, 255))):
            points = []
            for x_m in distances:
                pixel = ground.project(x_m, fit.evaluate(x_m))
                if pixel is not None:
                    points.append(pixel)
            if len(points) > 1:
                cv2.polylines(
                    debug, [np.array(points, dtype=np.int32)],
                    False, color, 2, cv2.LINE_AA)
        target = ground.project(
            config.target_distance_m, result.target_y_m)
        if target is not None:
            cv2.circle(debug, target, 6, (0, 255, 255), -1, cv2.LINE_AA)

    label = (
        f'valid={result.valid} conf={result.confidence:.2f} '
        f'err={result.lateral_error_m:.2f}')
    if result.reason:
        label += f' {result.reason}'
    cv2.putText(debug, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(debug, label, (12, 28), cv2.FONT_HERSHEY_SIMPLEX,
                0.6, (255, 255, 255), 1, cv2.LINE_AA)
    return debug


class LanePerception(Node):
    """ROS node that publishes LaneEstimate and a debug overlay."""

    def __init__(self) -> None:
        """Create the camera subscription and publishers."""
        super().__init__('lane_perception')
        self.camera = CameraModel()
        self.config = PerceptionConfig()
        self.ground = GroundPlane(self.camera)
        self.bridge = CvBridge()
        self.estimate_publisher = self.create_publisher(
            LaneEstimate, '/car/lane_estimate', 10)
        self.debug_publisher = self.create_publisher(
            Image, '/car/debug_image', 10)
        self.create_subscription(
            Image, '/car/camera/image_raw', self._on_image,
            qos_profile_sensor_data)

    def _on_image(self, message: Image) -> None:
        stamp_s = (
            message.header.stamp.sec +
            message.header.stamp.nanosec * 1e-9)
        now_s = self.get_clock().now().nanoseconds * 1e-9
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        if is_stale(stamp_s, now_s, self.config.max_image_age_s):
            result = _invalid('stale_image')
        else:
            result = analyze_frame(image, self.ground, self.config)

        estimate = LaneEstimate()
        estimate.header = message.header
        estimate.valid = result.valid
        estimate.confidence = result.confidence
        estimate.reason = result.reason
        estimate.lateral_error = result.lateral_error_m
        estimate.lookahead_target.x = result.target_x_m
        estimate.lookahead_target.y = result.target_y_m
        estimate.lookahead_target.z = 0.0
        self.estimate_publisher.publish(estimate)

        debug = make_debug_image(image, result, self.ground, self.config)
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)


def main() -> None:
    """Run the perception node."""
    rclpy.init()
    node = LanePerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
