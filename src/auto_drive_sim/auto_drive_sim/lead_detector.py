"""Detect the lead vehicle in the forward camera."""

from dataclasses import dataclass
from typing import Optional

import cv2
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


@dataclass(frozen=True)
class LeadDetectorConfig:
    """Colour and size limits for the lead vehicle."""

    lead_width_m: float = 1.6
    min_area_px: int = 40
    min_saturation: int = 120
    min_value: int = 80
    min_distance_m: float = 1.0
    max_distance_m: float = 40.0


def estimate_lead_distance(image: np.ndarray, fx: float,
                           config: LeadDetectorConfig) -> Optional[float]:
    """Return distance to the largest red lead-vehicle blob."""
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower = np.array([0, config.min_saturation, config.min_value])
    upper = np.array([25, 255, 255])
    mask = cv2.inRange(hsv, lower, upper)
    red_wrap = cv2.inRange(
        hsv,
        np.array([160, config.min_saturation, config.min_value]),
        np.array([180, 255, 255]))
    mask = cv2.bitwise_or(mask, red_wrap)
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    count, _, stats, _ = cv2.connectedComponentsWithStats(mask, 8)
    best_width = 0
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        width = int(stats[label, cv2.CC_STAT_WIDTH])
        if area >= config.min_area_px and width > best_width:
            best_width = width
    if best_width <= 0:
        return None

    distance = fx * config.lead_width_m / best_width
    if not config.min_distance_m <= distance <= config.max_distance_m:
        return None
    return float(distance)


class LeadDetector(Node):
    """Publish the camera distance to a red lead vehicle."""

    def __init__(self) -> None:
        """Create the camera subscription and distance publisher."""
        super().__init__('lead_detector')
        self.config = LeadDetectorConfig()
        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Float32, '/car/lead_distance', 10)
        self.create_subscription(
            Image, '/car/camera/image_raw', self._on_image,
            qos_profile_sensor_data)

    def _on_image(self, message: Image) -> None:
        image = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        distance = estimate_lead_distance(image, 320.0, self.config)
        if distance is not None:
            self.publisher.publish(Float32(data=distance))


def main() -> None:
    """Run the lead detector node."""
    rclpy.init()
    node = LeadDetector()
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
