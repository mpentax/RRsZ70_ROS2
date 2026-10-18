"""Live observation/mapping; optional idle pick server commands only on a goal."""
import os
import yaml
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc = FindPackageShare("z70_description")
    urdf = ParameterValue(Command(["xacro ", PathJoinSubstitution([desc, "urdf", "z70.urdf.xacro"])]), value_type=str)
    repo = Path(os.environ.get("Z70_REPO", Path.cwd()))
    from ament_index_python.packages import get_package_share_directory
    calibration = Path(get_package_share_directory('z70_description'))/'config'
    mount = yaml.safe_load((Path(get_package_share_directory('z70_description'))/'config/mount_calibration.yaml').read_text())
    camera_x = mount['front_camera_x_m']
    return LaunchDescription([
        DeclareLaunchArgument("origin", default_value="http://127.0.0.1:57010"),
        DeclareLaunchArgument("token_file", default_value=str(repo / ".local/service-token")),
        DeclareLaunchArgument("camera_info", default_value=str(calibration / "z70_front_camera_info.yaml")),
        DeclareLaunchArgument("calibration", default_value=str(calibration / "front_camera.json")),
        DeclareLaunchArgument("manipulation", default_value="false"),
        DeclareLaunchArgument("web_gateway", default_value="false"),
        DeclareLaunchArgument("web_gateway_port", default_value="57020"),
        Node(package='z70_web_gateway', executable='gateway', condition=IfCondition(LaunchConfiguration('web_gateway')),
             parameters=[{'token_file': LaunchConfiguration('token_file'),
                          'port': ParameterValue(LaunchConfiguration('web_gateway_port'), value_type=int)}]),
        Node(package='z70_manipulation',executable='perception'),
        Node(package='z70_manipulation',executable='pick_server',condition=IfCondition(LaunchConfiguration('manipulation')),
             parameters=[{'repo':str(repo)}]),
        DeclareLaunchArgument("mapping", default_value="true"),
        DeclareLaunchArgument("depth", default_value="true"),
        DeclareLaunchArgument("depth_python", default_value=str(Path.home() / ".venvs/roboclaw-depth/bin/python")),
        DeclareLaunchArgument("camera_hz", default_value="3.0"),
        Node(package="z70_mapping", executable="vendor_map", condition=IfCondition(LaunchConfiguration("mapping")),
             parameters=[]),
        Node(package="z70_mapping", executable="depth_cloud", condition=IfCondition(LaunchConfiguration("depth")),
             parameters=[{"calibration": LaunchConfiguration("calibration"), "depth_python": LaunchConfiguration("depth_python")}]),
        DeclareLaunchArgument("foxglove_port", default_value="8765"),
        Node(package="robot_state_publisher", executable="robot_state_publisher", parameters=[{"robot_description": urdf}]),
        Node(package="z70_driver", executable="driver", parameters=[{
            "origin": LaunchConfiguration("origin"),
            "token_file": LaunchConfiguration("token_file"),
            "camera_info": LaunchConfiguration("camera_info"),
            "camera_hz": ParameterValue(LaunchConfiguration("camera_hz"), value_type=float),
        }]),
        Node(package="z70_odometry", executable="floor_odometry", parameters=[{
            "calibration": LaunchConfiguration("calibration"), "camera_x_m": camera_x,
        }]),
        Node(package='z70_odometry',executable='chassis_calibrator'),
        Node(package="z70_odometry", executable="floor_mosaic", parameters=[{
            "calibration": LaunchConfiguration("calibration"), "camera_x_m": camera_x,
            "resolution": 0.005, "size_m": 4.0, "max_range": 1.2,
        }]),
        Node(package="foxglove_bridge", executable="foxglove_bridge", parameters=[{
            "address": "127.0.0.1", "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
        }]),
    ])
