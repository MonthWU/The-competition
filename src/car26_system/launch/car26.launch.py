"""Optional ROS launch wrapper; the application does not create ROS nodes."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import ExecutableInPackage


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("config"), DeclareLaunchArgument("mode", default_value="work"),
        ExecuteProcess(cmd=[ExecutableInPackage(package="car26_system", executable="car26"),
                            "--config", LaunchConfiguration("config"),
                            "--mode", LaunchConfiguration("mode")], output="screen"),
    ])
