import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Camera model (force value)
    sensor_model = 'leapmotion'

    # ZED Wrapper node
    leapmotion_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('leapmotion_wrapper'),
            '/launch/include/leapmotion_sensor.launch.py'
        ]),
        launch_arguments={
            'sensor_model': sensor_model
        }.items()
    )

    # Define LaunchDescription variable
    ld = LaunchDescription()

    # Add nodes to LaunchDescription
    ld.add_action(leapmotion_wrapper_launch)

    return ld
