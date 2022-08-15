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

    # Define config file location
    i2cpwm_ros2_config_file = get_share_file(
        package_name='i2cpwm_ros2', file_name='config/i2cpwm_ros2_config.yaml'
    )

    # tell ros we are using a config file
    i2cpwm_ros2_config = DeclareLaunchArgument(
        'i2cpwm_ros2_config_file',
        default_value=i2cpwm_ros2_config_file,
        description='Path to config file for i2cpwm parameters'
    )


    
    # define node to launch and parameters to use
    i2cpwm_ros2_node = Node(
        package='i2cpwm_ros2',
        executable='i2cpwm_ros2',
        output='screen',
        # arguments=['--ros-args', '--log-level', 'DEBUG'],
        parameters=[LaunchConfiguration('i2cpwm_ros2_config_file')],
    )

    return LaunchDescription([
        i2cpwm_ros2_config,
        i2cpwm_ros2_node
    ])


