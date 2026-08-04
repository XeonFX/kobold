"""Launch the base state publishers without owning the serial hardware.

This is the container-oriented half of ``bringup.launch.py``. Compose runs the
serial bridge in its own service so it can be restarted or version-gated
independently; this launch file owns only the URDF/TF tree and the odometry EKF.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = get_package_share_directory("kobold_bringup")
    urdf = os.path.join(pkg, "urdf", "kobold.urdf.xacro")
    ekf_config = os.path.join(pkg, "config", "ekf.yaml")

    use_ekf = DeclareLaunchArgument(
        "use_ekf",
        default_value="true",
        description="Fuse wheel odometry with gyro yaw.",
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {
                "robot_description": ParameterValue(
                    Command(["xacro ", urdf]), value_type=str
                )
            }
        ],
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
        condition=IfCondition(LaunchConfiguration("use_ekf")),
    )

    return LaunchDescription([use_ekf, robot_state_publisher, ekf])
