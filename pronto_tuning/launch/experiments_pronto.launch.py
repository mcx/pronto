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
        period=3.0,
        actions=[command_launch]
    )
    command_and_record = TimerAction(
        period=3.0,
        actions=[
            record_launch,command_time
        ]
    )

    simulator_solo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_path('robot_gazebo'), 'launch', 'anymal_c.launch.py')
                ),
                launch_arguments={
                    'gait': "walking_trot",
                    'use_rviz': "False"
                }.items()
        )

    load_estimator = ExecuteProcess(
        cmd=[["ros2 control load_controller pronto_estimator --set-state active"]],
        shell=True,
        output = "screen"
    )

    return LaunchDescription([
        simulator_solo,

        TimerAction(period= 1.0,
            actions=[load_estimator]
        ),
    ])

