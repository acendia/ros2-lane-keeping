"""Lane perception from the forward camera."""

from copy import deepcopy
from dataclasses import dataclass
import math
from typing import Optional

from auto_drive_sim_interfaces.msg import LaneEstimate
from std_msgs.msg import String
import json
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy._rclpy_pybind11 import RCLError
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
    roi_max_forward_m: float = 15.0
    roi_half_width_m: float = 8.0
    min_component_area_px: int = 8
    min_component_points: int = 5
    min_boundary_points: int = 50
    min_boundary_span_m: float = 2.5
    min_linear_span_m: float = 1.0
    max_target_gap_m: float = 1.6
    min_separation_m: float = 2.8
    max_separation_m: float = 4.5
    nominal_lane_width_m: float = 3.62
    pair_x_tolerance_m: float = 1.6
    pair_min_separation_m: float = 2.0
    pair_max_separation_m: float = 6.0
    max_assignment_distance_m: float = 0.8
    max_fit_residual_m: float = 0.25
    max_fit_rms_m: float = 0.15
    max_center_offset_m: float = 2.5
    white_value_min: int = 180
    white_saturation_max: int = 60
    local_contrast_window_px: int = 51
    local_contrast_min: int = 35
    local_value_min: int = 60
    marking_open_px: int = 3
    horizontal_marking_window_px: int = 25
    horizontal_marking_contrast_min: int = 20
    hold_last_valid_s: float = 0.25
    lane_strategy: str = 'single'
    max_image_age_s: float = 0.3


@dataclass(frozen=True)
class BoundaryFit:
    """Polynomial fit y(x) for one lane boundary."""

    coefficients: np.ndarray
    rms_m: float
    point_count: int
    x_min_m: float
    x_max_m: float

    def evaluate(self, x_m: float) -> float:
        """Evaluate the boundary lateral position at a forward distance."""
        return float(np.polyval(self.coefficients, x_m))


@dataclass(frozen=True)
class MarkingComponent:
    """One connected marking piece with its ground points."""

    x_values: np.ndarray
    y_values: np.ndarray
    x_center_m: float
    y_center_m: float


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


def threshold_markings(image: np.ndarray, config: PerceptionConfig,
                       use_local: bool = True) -> np.ndarray:
    """Return a mask of bright markings, with optional shadow handling.

    The global threshold handles lit paint. The local-contrast pass compares
    each pixel with a blurred neighbourhood, so paint in shadow still stands
    out from the road. A small opening removes isolated shadow-edge noise.
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    bright = cv2.inRange(
        hsv,
        (0, 0, config.white_value_min),
        (180, config.white_saturation_max, 255))
    if not use_local:
        return bright

    value = hsv[:, :, 2]
    window = config.local_contrast_window_px
    background = cv2.GaussianBlur(value, (window, window), 0)
    local_contrast = cv2.subtract(value, background)
    local = ((local_contrast >= config.local_contrast_min) &
             (hsv[:, :, 1] <= config.white_saturation_max) &
             (value >= config.local_value_min)).astype(np.uint8) * 255
    mask = cv2.bitwise_or(bright, local)

    open_size = config.marking_open_px
    if open_size >= 2:
        kernel = np.ones((open_size, open_size), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return mask


def _marking_components(mask: np.ndarray, ground: GroundPlane,
                        config: PerceptionConfig) -> list:
    """Collect marking pieces that reach the road ROI."""
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
        if x_values.size < config.min_component_points:
            continue
        components.append(MarkingComponent(
            x_values=x_values,
            y_values=y_values,
            x_center_m=float(np.median(x_values)),
            y_center_m=float(np.median(y_values)),
        ))
    return components


def _component_pairs(components: list, config: PerceptionConfig) -> list:
    """Pair dashes that sit at the same forward distance on opposite sides."""
    pairs = []
    used = set()
    for first_index, first in enumerate(components):
        if first_index in used:
            continue
        best_index = None
        best_score = None
        for second_index, second in enumerate(components):
            if second_index == first_index or second_index in used:
                continue
            dx = abs(first.x_center_m - second.x_center_m)
            if dx > config.pair_x_tolerance_m:
                continue
            dy = abs(first.y_center_m - second.y_center_m)
            if not config.pair_min_separation_m <= dy <= \
                    config.pair_max_separation_m:
                continue
            score = dx + abs(dy - config.nominal_lane_width_m)
            if best_score is None or score < best_score:
                best_index = second_index
                best_score = score
        if best_index is not None:
            used.add(first_index)
            used.add(best_index)
            pairs.append((first_index, best_index))
    return pairs


def _join_points(components: list, config: PerceptionConfig) -> tuple:
    """Aggregate paired pieces and attach any remaining pieces to one side."""
    left_x = []
    left_y = []
    right_x = []
    right_y = []
    paired = set()
    for first_index, second_index in _component_pairs(components, config):
        first = components[first_index]
        second = components[second_index]
        paired.add(first_index)
        paired.add(second_index)
        if first.y_center_m >= second.y_center_m:
            left, right = first, second
        else:
            left, right = second, first
        left_x.append(left.x_values)
        left_y.append(left.y_values)
        right_x.append(right.x_values)
        right_y.append(right.y_values)

    if not left_x:
        empty = (np.array([]), np.array([]))
        return empty, empty

    left_x_array = np.concatenate(left_x)
    left_y_array = np.concatenate(left_y)
    right_x_array = np.concatenate(right_x)
    right_y_array = np.concatenate(right_y)
    if left_x_array.size < 3 or right_x_array.size < 3:
        return (left_x_array, left_y_array), (right_x_array, right_y_array)

    # Paired fragments can project to only one or two distinct ground depths.
    # A quadratic is then underdetermined and cannot guide assignment safely.
    if np.unique(left_x_array).size < 3 or np.unique(right_x_array).size < 3:
        return (left_x_array, left_y_array), (right_x_array, right_y_array)
    left_coefficients = np.polyfit(left_x_array, left_y_array, 2)
    right_coefficients = np.polyfit(right_x_array, right_y_array, 2)

    for index, component in enumerate(components):
        if index in paired:
            continue
        x_m = component.x_center_m
        left_y_at_x = float(np.polyval(left_coefficients, x_m))
        right_y_at_x = float(np.polyval(right_coefficients, x_m))
        separation = left_y_at_x - right_y_at_x
        if not config.pair_min_separation_m <= separation <= \
                config.pair_max_separation_m:
            continue
        left_distance = abs(component.y_center_m - left_y_at_x)
        right_distance = abs(component.y_center_m - right_y_at_x)
        if min(left_distance, right_distance) > \
                config.max_assignment_distance_m:
            continue
        if left_distance <= right_distance:
            left_x_array = np.concatenate(
                [left_x_array, component.x_values])
            left_y_array = np.concatenate(
                [left_y_array, component.y_values])
        else:
            right_x_array = np.concatenate(
                [right_x_array, component.x_values])
            right_y_array = np.concatenate(
                [right_y_array, component.y_values])
    return (left_x_array, left_y_array), (right_x_array, right_y_array)


def extract_boundary_points(mask: np.ndarray, ground: GroundPlane,
                            config: PerceptionConfig) -> tuple:
    """Group marking pieces into left and right boundary point sets."""
    components = _marking_components(mask, ground, config)
    if config.lane_strategy == 'rightmost' and len(components) > 2:
        components = sorted(
            components, key=lambda component: component.y_center_m)[:2]
    return _join_points(components, config)


def fit_boundary(x_values: np.ndarray, y_values: np.ndarray,
                 config: PerceptionConfig) -> Optional[BoundaryFit]:
    """Fit a boundary and reject short or weak point sets.

    Long spans use a quadratic. A single dash near the lookahead target uses
    a line, because a quadratic is not constrained over a 1.5 m dash.
    """
    if x_values.size < config.min_boundary_points:
        return None
    span = float(x_values.max() - x_values.min())
    if span >= config.min_boundary_span_m:
        degree = 2
    elif span >= config.min_linear_span_m:
        degree = 1
    else:
        return None

    coefficients = np.polyfit(x_values, y_values, degree)
    inliers = np.ones(x_values.size, dtype=bool)
    for _ in range(3):
        residuals = y_values - np.polyval(coefficients, x_values)
        inliers = np.abs(residuals) <= config.max_fit_residual_m
        if inliers.sum() < config.min_boundary_points:
            return None
        coefficients = np.polyfit(
            x_values[inliers], y_values[inliers], degree)
    residuals = y_values[inliers] - np.polyval(
        coefficients, x_values[inliers])
    rms = float(np.sqrt(np.mean(residuals ** 2)))
    span = float(x_values[inliers].max() - x_values[inliers].min())
    if degree == 1:
        if span < config.min_linear_span_m:
            return None
        if x_values[inliers].min() > \
                config.target_distance_m + config.max_target_gap_m:
            return None
    elif span < config.min_boundary_span_m:
        return None
    if rms > config.max_fit_rms_m:
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


def within_hold(last_stamp_s: Optional[float], image_stamp_s: float,
                max_hold_s: float) -> bool:
    """Return true when the last valid estimate is recent enough to hold."""
    if last_stamp_s is None:
        return False
    age = image_stamp_s - last_stamp_s
    return 0.0 <= age <= max_hold_s


def _analyze_mask(mask: np.ndarray, ground: GroundPlane,
                  config: PerceptionConfig) -> PerceptionResult:
    """Run the lane pipeline on a binary marking mask."""
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
    width_score = max(
        0.0, 1.0 - abs(lane_width - config.nominal_lane_width_m) / 0.7)
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


def analyze_frame(image: np.ndarray, ground: GroundPlane,
                  config: PerceptionConfig) -> PerceptionResult:
    """Try bright paint, local contrast, then horizontal marking contrast."""
    bright_mask = threshold_markings(image, config, use_local=False)
    result = _analyze_mask(bright_mask, ground, config)
    if result.valid:
        return result

    local_mask = threshold_markings(image, config, use_local=True)
    if not np.array_equal(local_mask, bright_mask):
        result = _analyze_mask(local_mask, ground, config)
        if result.valid:
            return result

    # Compare across each row so a horizontal shadow edge does not dominate
    # the contrast of narrow paint stripes crossing it.
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    value = hsv[:, :, 2]
    kernel = np.ones((1, config.horizontal_marking_window_px), dtype=np.uint8)
    contrast = cv2.morphologyEx(value, cv2.MORPH_TOPHAT, kernel)
    mask = ((contrast >= config.horizontal_marking_contrast_min) &
            (hsv[:, :, 1] <= config.white_saturation_max) &
            (value >= config.local_value_min)).astype(np.uint8) * 255
    recovered = _analyze_mask(mask, ground, config)
    return recovered if recovered.valid else result


def make_debug_image(image: np.ndarray, result: PerceptionResult,
                     ground: GroundPlane, config: PerceptionConfig,
                     label_suffix: str = '') -> np.ndarray:
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
    label += label_suffix
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
        self.declare_parameter('lane_strategy', 'single')
        self.config = PerceptionConfig(
            lane_strategy=self.get_parameter(
                'lane_strategy').value)
        self.ground = GroundPlane(self.camera)
        self.bridge = CvBridge()
        self.estimate_publisher = self.create_publisher(
            LaneEstimate, '/car/lane_estimate', 10)
        self.debug_publisher = self.create_publisher(
            Image, '/car/debug_image', 10)
        self.diagnostic_publisher = self.create_publisher(String, '/car/lane_diagnostic', 10)
        self.last_valid_estimate = None
        self.last_valid_result = None
        self.last_valid_stamp_s = None
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

        diagnostic = String()
        diagnostic.data = json.dumps({'image_stamp_s': stamp_s, 'valid': result.valid,
                                      'reason': result.reason, 'confidence': result.confidence})
        self.diagnostic_publisher.publish(diagnostic)

        estimate = LaneEstimate()
        estimate.header = message.header
        estimate.valid = result.valid
        estimate.confidence = result.confidence
        estimate.reason = result.reason
        estimate.lateral_error = result.lateral_error_m
        estimate.lookahead_target.x = result.target_x_m
        estimate.lookahead_target.y = result.target_y_m
        estimate.lookahead_target.z = 0.0
        if result.valid:
            self.last_valid_estimate = estimate
            self.last_valid_result = result
            self.last_valid_stamp_s = stamp_s
            self.estimate_publisher.publish(estimate)
            debug_result = result
            suffix = ''
        elif within_hold(
                self.last_valid_stamp_s, stamp_s,
                self.config.hold_last_valid_s):
            held = deepcopy(self.last_valid_estimate)
            held.reason = 'held_last_valid'
            self.estimate_publisher.publish(held)
            debug_result = self.last_valid_result
            suffix = ' held'
        else:
            self.estimate_publisher.publish(estimate)
            debug_result = result
            suffix = ''

        debug = make_debug_image(
            image, debug_result, self.ground, self.config, suffix)
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding='bgr8')
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)


def main() -> None:
    """Run the perception node."""
    rclpy.init()
    node = LanePerception()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except RCLError:
        if rclpy.ok():
            raise
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
