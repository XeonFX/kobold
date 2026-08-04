"""Camera only -- useful for calibration, framing and debugging without the NPU."""

from os.path import join

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    params = join(get_package_share_directory("kobold_perception"), "config", "perception.yaml")
    return LaunchDescription(
        [
            Node(
                package="kobold_perception",
                executable="camera_node",
                name="camera_node",
                parameters=[params],
                output="screen",
            )
        ]
    )
