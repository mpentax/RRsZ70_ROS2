"""Model only: URDF + joint sliders + RViz2. No robot contact."""
from launch import LaunchDescription
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc = FindPackageShare("z70_description")
    urdf = ParameterValue(Command(["xacro ", PathJoinSubstitution([desc, "urdf", "z70.urdf.xacro"])]), value_type=str)
    return LaunchDescription([
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": urdf}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui"),
        Node(package="rviz2", executable="rviz2", arguments=["-d", PathJoinSubstitution([desc, "rviz", "z70.rviz"])]),
    ])
