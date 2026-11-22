"""Detect the lead vehicle in the forward camera."""

from dataclasses import dataclass
from typing import Optional

from auto_drive_sim.lane_perception import CameraModel, GroundPlane
import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from auto_drive_sim_interfaces.msg import LeadEstimate


@dataclass(frozen=True)
class LeadDetectorConfig:
    """Colour and range limits for the lead vehicle."""

    min_red_pixels: int = 10
    min_saturation: int = 120
    min_value: int = 80
    min_distance_m: float = 1.0
    max_distance_m: float = 40.0
    roi_half_width_m: float = 6.0


def estimate_lead_distance(image: np.ndarray, ground: GroundPlane,
                           config: LeadDetectorConfig) -> Optional[float]:
    """Return ground-plane distance to the nearest red lead pixel."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(
        hsv,
        np.array([0, config.min_saturation, config.min_value]),
        np.array([25, 255, 255]))
    red_wrap = cv2.inRange(
        hsv,
        np.array([160, config.min_saturation, config.min_value]),
        np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask, red_wrap)
    mask = cv2.morphologyEx(
        mask, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))

    x_values, y_values = ground.points_for_mask(mask)
    keep = (
        (x_values >= config.min_distance_m) &
        (x_values <= config.max_distance_m) &
        (np.abs(y_values) <= config.roi_half_width_m))
    x_values = x_values[keep]
    if x_values.size < config.min_red_pixels:
        return None
    return float(np.percentile(x_values, 5))


class LeadDetector(Node):
    """Publish the camera distance to a red lead vehicle."""

    def __init__(self) -> None:
        """Create the camera subscription and distance publisher."""
        super().__init__('lead_detector')
        self.config = LeadDetectorConfig()
        self.ground = GroundPlane(CameraModel())
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            LeadEstimate, '/car/lead_estimate', 10)
        self.create_subscription(
            Image, '/car/camera/image_raw', self._on_image,
            qos_profile_sensor_data)

    def _on_image(self, message: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        distance = estimate_lead_distance(image, self.ground, self.config)
        stamp_s = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp_s
        estimate = LeadEstimate()
        estimate.header = message.header
        estimate.valid = 0.0 <= age <= 0.3
        estimate.detected = distance is not None
        estimate.distance = distance if distance is not None else 0.0
        self.publisher.publish(estimate)


def main() -> None:
    """Run the lead detector node."""
    rclpy.init()
    node = LeadDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
