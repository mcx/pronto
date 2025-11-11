from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pronto_ros',
            namespace='pronto_ros',
            executable='pronto_ros_node',
            name='pronto_ros_node'
        ),
        Node(
            package='pronto_ros',
            namespace='pronto_ros',
            executable='joint_state_with_acceleration_republisher',
            name='joint_states_republisher'
        ),

    ])