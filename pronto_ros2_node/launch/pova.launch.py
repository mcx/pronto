import os

from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():


    xacro_pkg_value = LaunchConfiguration("xacro_pkg")
    xacro_name_value = LaunchConfiguration("xacro_name")
    config_name_value = LaunchConfiguration("config_name")
    node_name_value = LaunchConfiguration("node_name")
    package_share_path = FindPackageShare('pronto_ros2_node')
    pose_topic_name = LaunchConfiguration('e_pose_top')

    xacro_pkg_arg = DeclareLaunchArgument("xacro_pkg",default_value="pronto_ros2_node")
    xacro_name_arg = DeclareLaunchArgument("xacro_name",default_value="mulinex")
    config_name_arg = DeclareLaunchArgument("config_name",default_value="pronto_mulinex.yaml")
    node_name_arg = DeclareLaunchArgument("node_name",default_value="pronto_node")
    pose_topic_name_arg = DeclareLaunchArgument('e_pose_top',default_value="prova")

    xacro_file_path = PathJoinSubstitution([
        FindPackageShare(xacro_pkg_value),
       'urdf', xacro_name_value
    ])
    config_file_path = PathJoinSubstitution([
        package_share_path,
        'config',config_name_value
    ])
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # ======================================================================== #

    return LaunchDescription([

        xacro_name_arg,
        xacro_pkg_arg,
        config_name_arg,
        pose_topic_name_arg,
        node_name_arg,

        # Subscribe to the joint states of the robot, and publish the 3D pose of each link.
        Node(
            package='pronto_ros2_node',
            executable= "pronto_node",
            name= node_name_value,
            parameters=[
                {'urdf_file': ParameterValue(Command(['xacro', ' ' ,xacro_file_path]), value_type=str)},
                config_file_path
            ],
        #      remappings=[
        #     ('/pose_est', pose_topic_name),
        #     # ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
        # ]
        )

        # # Launch RViz
        # Node(
        #     package='rviz2',
        #     namespace='',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', config_file_path],
        # )
    ])
