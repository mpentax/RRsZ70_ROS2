"""Unified ROS 2 and native sidecar bringup launch description."""
import os
import shutil
import yaml
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    desc = FindPackageShare("z70_description")
    urdf = ParameterValue(Command(["xacro ", PathJoinSubstitution([desc, "urdf", "z70.urdf.xacro"])]), value_type=str)
    repo = Path(os.environ.get("Z70_REPO", Path.cwd()))
    calibration = Path(get_package_share_directory('z70_description')) / 'config'
    mount = yaml.safe_load((calibration / 'mount_calibration.yaml').read_text())
    camera_x = mount['front_camera_x_m']
    node_bin = shutil.which("node") or "node"
    hardware_sidecars = IfCondition(PythonExpression(["'", LaunchConfiguration("sidecars"), "' == 'true' and '", LaunchConfiguration("sim"), "' != 'true'"]))
    return LaunchDescription([
        DeclareLaunchArgument("origin", default_value="http://127.0.0.1:57010"),
        DeclareLaunchArgument("token_file", default_value=str(repo / ".local/service-token")),
        DeclareLaunchArgument("camera_info", default_value=str(calibration / "z70_front_camera_info.yaml")),
        DeclareLaunchArgument("calibration", default_value=str(calibration / "front_camera.json")),
        DeclareLaunchArgument("sidecars", default_value="false"),
        DeclareLaunchArgument("sim", default_value="false"),
        DeclareLaunchArgument("manipulation", default_value="false"),
        DeclareLaunchArgument("garment_perception", default_value="false"),
        DeclareLaunchArgument("garment_backend", default_value="grounding_dino_clipseg"),
        DeclareLaunchArgument("garment_python", default_value=str(Path.home() / ".venvs/roboclaw-depth/bin/python")),
        DeclareLaunchArgument("qualified_small_lift", default_value="true"),
        DeclareLaunchArgument("allow_tow", default_value="true"),
        DeclareLaunchArgument("towing_qualified", default_value="false"),
        DeclareLaunchArgument("towing_artifact_uri", default_value="qualified"),
        ExecuteProcess(
            cmd=["ros2", "run", "z70_sim", "sim_sidecar", "--token-file", LaunchConfiguration("token_file")],
            cwd=str(repo),
            condition=IfCondition(LaunchConfiguration("sim")),
            output="screen",
        ),
        ExecuteProcess(
            cmd=[str(repo / ".local/bin/go2rtc"), "-config", str(repo / "deploy/go2rtc.yaml")],
            additional_env={"ROBOCLAW_WEBRTC_HOST": os.environ.get("ROBOCLAW_WEBRTC_HOST", "127.0.0.1")},
            condition=hardware_sidecars,
            output="log",
        ),
        ExecuteProcess(
            cmd=[node_bin, str(repo / "adapters/vendor/src/service-main.ts")],
            cwd=str(repo),
            additional_env={
                "NODE_ENV": "production",
                "ROBOCLAW_HOST": os.environ.get("ROBOCLAW_HOST", "127.0.0.1"),
                "ROBOCLAW_PORT": os.environ.get("ROBOCLAW_PORT", "57010"),
                "ROBOCLAW_PUBLIC_ORIGIN": os.environ.get("ROBOCLAW_PUBLIC_ORIGIN", "http://127.0.0.1:57010"),
                "ROBOCLAW_STATE_DIR": str(repo / ".local"),
                "GO2RTC_ORIGIN": os.environ.get("GO2RTC_ORIGIN", "http://127.0.0.1:57001"),
                "Z70_ROS_GATEWAY_ORIGIN": os.environ.get("Z70_ROS_GATEWAY_ORIGIN", "http://127.0.0.1:57020"),
            },
            condition=hardware_sidecars,
            output="screen",
        ),
        Node(package='z70_manipulation', executable='garment_perception',
             condition=IfCondition(LaunchConfiguration('garment_perception')),
             parameters=[{'backend': LaunchConfiguration('garment_backend'),
                          'garment_python': LaunchConfiguration('garment_python'),
                          'qualified_small_lift': ParameterValue(LaunchConfiguration('qualified_small_lift'), value_type=bool),
                          'allow_tow': ParameterValue(LaunchConfiguration('allow_tow'), value_type=bool)}]),
        DeclareLaunchArgument("web_gateway", default_value="false"),
        DeclareLaunchArgument("web_gateway_port", default_value="57020"),
        Node(package='z70_web_gateway', executable='gateway', condition=IfCondition(LaunchConfiguration('web_gateway')),
             parameters=[{'token_file': LaunchConfiguration('token_file'),
                          'port': ParameterValue(LaunchConfiguration('web_gateway_port'), value_type=int)}]),
        Node(package='z70_manipulation', executable='perception'),
        Node(package='z70_manipulation', executable='pick_server', condition=IfCondition(LaunchConfiguration('manipulation')),
             parameters=[{'repo': str(repo),
                          'towing_qualified': ParameterValue(LaunchConfiguration('towing_qualified'), value_type=bool),
                          'towing_artifact_uri': LaunchConfiguration('towing_artifact_uri')}]),
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
        Node(package='z70_odometry', executable='chassis_calibrator'),
        Node(package="z70_odometry", executable="floor_mosaic", parameters=[{
            "calibration": LaunchConfiguration("calibration"), "camera_x_m": camera_x,
            "resolution": 0.005, "size_m": 4.0, "max_range": 1.2,
        }]),
        Node(package="foxglove_bridge", executable="foxglove_bridge", parameters=[{
            "address": "127.0.0.1", "port": ParameterValue(LaunchConfiguration("foxglove_port"), value_type=int),
        }]),
    ])
