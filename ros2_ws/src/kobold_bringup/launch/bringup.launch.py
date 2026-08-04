"""Bring up the robot base: bridge, EKF, and the URDF/TF tree.

    ros2 launch kobold_bringup bringup.launch.py

Against the simulator (no hardware needed — see kobold_bridge/sim.py):

    ros2 run kobold_bridge sim
    ros2 launch kobold_bringup bringup.launch.py \\
        drive_port:=/dev/pts/3 sense_port:=/dev/pts/4

Table mode caps speed hard and is what you want before putting the robot on any
surface it can fall off:

    ros2 launch kobold_bringup bringup.launch.py table_mode:=true
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
    bridge_config = os.path.join(pkg, "config", "bridge.yaml")

    args = [
        DeclareLaunchArgument("drive_port", default_value="/dev/robot-drive"),
        DeclareLaunchArgument("sense_port", default_value="/dev/robot-sense"),
        DeclareLaunchArgument(
            "table_mode",
            default_value="false",
            description="Cap speed for driving on tables. Cliff sensors are the "
            "only thing preventing a fall — test the reflex on the floor first.",
        ),
        DeclareLaunchArgument(
            "use_ekf",
            default_value="true",
            description="Fuse wheel odometry with gyro yaw. Strongly recommended: "
            "skid-steer wheel heading alone is unreliable by construction.",
        ),
    ]

    bridge = Node(
        package="kobold_bridge",
        executable="bridge",
        name="kobold_bridge",
        output="screen",
        parameters=[
            bridge_config,
            {
                "drive_port": LaunchConfiguration("drive_port"),
                "sense_port": LaunchConfiguration("sense_port"),
                "table_mode": LaunchConfiguration("table_mode"),
            },
        ],
        # If the bridge dies, the firmware watchdog has already stopped the
        # motors. Restarting is safe and beats a robot that needs a human to
        # notice a crashed node.
        respawn=True,
        respawn_delay=2.0,
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

    # Owns the odom -> base_link transform. The bridge deliberately does not
    # publish it; two writers on one TF edge is a debugging nightmare.
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[ekf_config],
        condition=IfCondition(LaunchConfiguration("use_ekf")),
    )

    return LaunchDescription(args + [bridge, robot_state_publisher, ekf])
