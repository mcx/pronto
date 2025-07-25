import os
from launch.substitutions import LaunchConfiguration,PathJoinSubstitution,FindExecutable
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path
from launch.actions import DeclareLaunchArgument,EmitEvent, RegisterEventHandler,ExecuteProcess
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)



def generate_launch_description():

    global config_file_path
    exp_name_value = LaunchConfiguration("exp_name")
    package_path = FindPackageShare("pronto_tuning")
    exp_name_arg = DeclareLaunchArgument("exp_name",default_value="Exp_1_cmd.yaml")

    config_path = PathJoinSubstitution([
        package_path,
        "config",exp_name_value
    ])

    return LaunchDescription([
        exp_name_arg,
        # Launch the Pronto Solo node with parameters set in "state_estimator.yaml"

        Node(
            package='pronto_tuning',
            executable='command_node',
            parameters=[config_path]
        )
    ])
