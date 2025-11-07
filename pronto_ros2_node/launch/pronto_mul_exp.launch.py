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


    # xacro_pkg_value = LaunchConfiguration("xacro_pkg")
    # xacro_name_value = LaunchConfiguration("xacro_name")
    # config_name_value = LaunchConfiguration("config_name")

    # package_share_path = FindPackageShare('pronto_ros2_node')
    bag_base_path = '/home/punk-opc/Documents/Davides_Simulation/control_quadrupeds_soft_contacts/src/Pronto_Estimator/pronto_ros2_node/Experiments/'
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
    param_file = "pronto_mulinex"
    param_file_ext = param_file + ".yaml"
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
            'xacro_name': 'mulinex.xacro',
            'config_name': TextSubstitution(text=param_file_ext)
        }.items()
    )

    # start the bag dalayed
    time = datetime.now()

    time_str = time.strftime("%Y_%m_%d_%H_%M_%S")

    bag_path = bag_base_path + "Exp_"+param_file+"_"+time_str

    start_bag = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '/rigid_bodies',
             '/pose_est','/twist_est','/rh_veldebug','/rh_grf' ,
             '/rf_veldebug','/rf_grf' ,
             '/lh_veldebug','/lh_grf' ,
             '/lf_veldebug','/lf_grf' ,
             '/state_broadcaster/joints_state',
             '/state_broadcaster/performance_indexes',
             '/IMU_Broadcaster/imu','-o', bag_path ],
     output='screen'
    )
    # start the motion delayed ++

    start_motion = ExecuteProcess(
        cmd=[[
            FindExecutable(name='ros2'),
            " service call ",
            "base_motion_node/start_motion_service ",
            "std_srvs/srv/SetBool ",
            '"{data: True}"',
        ]],
        shell=True, output='screen'
    )

    # close everithing deleyed +++++++++++++++
    end_exp = EmitEvent(event=Shutdown(reason="End Benchmarking Experiments"))


    end_delayed = TimerAction(
        period=20.0,
        actions=[end_exp]
    )
    motion_delayed = TimerAction(
        period=5.0,
        actions=[start_motion,end_delayed]
    )
    bag_delayed = TimerAction(
        period=1.0,
        actions=[start_bag,motion_delayed]
    )
    # ======================================================================== #

    return LaunchDescription([
        pronto_launch,
        bag_delayed

    ])
