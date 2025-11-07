import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription,ExecuteProcess
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import TimerAction

def generate_launch_description():

    command_launch  = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path('pronto_tuning'), 'launch', 'command_tuning.launch.py')
                )
            )
    record_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path('pronto_tuning'), 'launch', 'filter_tuning.launch.py')
                )
        )
    command_time = TimerAction(
        period=0.5,
        actions=[command_launch]
    )

    return LaunchDescription([
        command_time,
        record_launch
    ])
