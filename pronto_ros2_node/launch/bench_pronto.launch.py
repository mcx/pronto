import os
from datetime import datetime
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution,TextSubstitution,FindExecutable
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,ExecuteProcess,TimerAction,EmitEvent
from launch.events import Shutdown
def generate_launch_description():


    pronto_instance = ['2']

    bag_name = "Exp_2023_11_22_01_19_54"
    exp_name = os.path.join("bags",bag_name,bag_name+"_0.mcap")
    # xacro_pkg_arg = DeclareLaunchArgument("xacro_pkg",default_value="pronto_ros2_node")
    # xacro_name_arg = DeclareLaunchArgument("xacro_name",default_value="mulinex")
    # config_name_arg = DeclareLaunchArgument("config_name",default_value="pronto_mulinex.yaml")
    # xacro_file_path = PathJoinSubstitution([
    #     FindPackageShare(xacro_pkg_value),
    #    'urdf', xacro_name_value
    # ])
    # config_file_path = PathJoinSubstitution([
    #     package_share_path,
    #     'config',config_name_value
    # ])
    # use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    ld = LaunchDescription()

    pronto_conf =[]
    for i in range(len(pronto_instance)):
        param_file = "omnicar_tune_" + pronto_instance[i]
        param_file_ext = param_file + ".yaml"
        print(param_file_ext)
        pronto_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare('pronto_ros2_node'),
                    'launch',
                    'pronto_node.launch.py'
                ])
            ]),

            launch_arguments={
                'xacro_pkg': 'mulinex_description',
                'xacro_name': 'omnicar.xacro',
                'config_name': TextSubstitution(text=param_file_ext),
                'node_name' : "Exp_" + pronto_instance[i],
                'topic_name' : "pose_est_" + pronto_instance[i],
                'v_topic_name' : 'twist_est_' + pronto_instance[i]
            }.items()
        )
        ld.add_action(pronto_launch)
    start_bag = ExecuteProcess(
        cmd=['ros2','bag','play', exp_name]
    )
    bag_delayed = TimerAction(
        actions=[start_bag], period=1.0

    )

    ld.add_action(bag_delayed)
    # ======================================================================== #

    return ld
