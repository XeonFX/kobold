"""Camera + NPU detector.

Note these run in SEPARATE containers in compose.yaml (camera on the A55
cluster, detector pinned to A76 core 4), so this launch file is mainly for
running the pair by hand on the robot or on a bench.
"""

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
            ),
            Node(
                package="kobold_perception",
                executable="detector_node",
                name="detector_node",
                parameters=[params],
                output="screen",
            ),
        ]
    )
