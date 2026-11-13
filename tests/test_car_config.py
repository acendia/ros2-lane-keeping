"""Static checks for the car model and the bridge configuration."""

from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

REPO_ROOT = Path(__file__).resolve().parents[1]
CAR_SDF = REPO_ROOT / 'src/auto_drive_sim/models/car/model.sdf'
BRIDGE_CONFIG = REPO_ROOT / 'src/auto_drive_sim/config/bridge.yaml'


def test_ackermann_parameters():
    """The car must keep the documented Ackermann limits."""
    root = ET.parse(CAR_SDF).getroot()
    plugin = root.find(
        ".//plugin[@name='gz::sim::systems::AckermannSteering']")
    assert plugin is not None

    expected = {
        'wheel_base': '2.4',
        'wheel_separation': '1.4',
        'wheel_radius': '0.32',
        'steering_limit': '0.55',
        'max_velocity': '3.0',
        'topic': '/car/cmd_vel',
        'odom_topic': '/car/odometry',
    }
    for name, value in expected.items():
        assert plugin.findtext(name) == value


def test_camera_sensor():
    """The camera must keep the documented topic, size, and rate."""
    root = ET.parse(CAR_SDF).getroot()
    sensor = root.find(".//sensor[@type='camera']")
    assert sensor is not None
    assert sensor.findtext('topic') == '/car/camera/image_raw'
    assert sensor.findtext('update_rate') == '15'
    assert sensor.findtext('camera/horizontal_fov') == '1.5708'
    assert sensor.findtext('camera/image/width') == '640'
    assert sensor.findtext('camera/image/height') == '360'


def test_bridge_mappings():
    """Every required topic must have one explicit bridge direction."""
    bridges = yaml.safe_load(BRIDGE_CONFIG.read_text(encoding='utf-8'))
    by_topic = {bridge['ros_topic_name']: bridge for bridge in bridges}

    expected = {
        '/clock': ('GZ_TO_ROS', 'rosgraph_msgs/msg/Clock', 'gz.msgs.Clock'),
        '/car/camera/image_raw': (
            'GZ_TO_ROS', 'sensor_msgs/msg/Image', 'gz.msgs.Image'),
        '/car/camera/camera_info': (
            'GZ_TO_ROS', 'sensor_msgs/msg/CameraInfo', 'gz.msgs.CameraInfo'),
        '/car/odometry': (
            'GZ_TO_ROS', 'nav_msgs/msg/Odometry', 'gz.msgs.Odometry'),
        '/car/cmd_vel': (
            'ROS_TO_GZ', 'geometry_msgs/msg/Twist', 'gz.msgs.Twist'),
    }
    assert set(by_topic) == set(expected)

    for topic, (direction, ros_type, gz_type) in expected.items():
        assert by_topic[topic]['direction'] == direction
        assert by_topic[topic]['ros_type_name'] == ros_type
        assert by_topic[topic]['gz_type_name'] == gz_type
