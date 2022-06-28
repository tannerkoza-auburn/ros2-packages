import os

from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_share_file(package_name, file_name):
    return os.path.join(get_package_share_directory(package_name), file_name)


def generate_launch_description():

    razor_imu_param_file = get_share_file(
        package_name="razor_imu", file_name="config/razor_imu.yaml"
    )

    razor_imu_config = DeclareLaunchArgument(
        "razor_imu_param_file",
        default_value=razor_imu_param_file,
        description="Path to config file for razor_imu",
    )

    razor_imu_node = Node(
        package="razor_imu",
        executable="razor_imu_node",
        namespace="razor_imu",
        name="ola_imu",
        parameters=[LaunchConfiguration("razor_imu_param_file")],
        emulate_tty=True,
    )

    return LaunchDescription([razor_imu_config, razor_imu_node])
