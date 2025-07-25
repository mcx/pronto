import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path


def generate_launch_description():
    package_path = get_package_share_path("pronto_tuning")
    config_path = os.path.join(package_path, 'config/tuning_node_param.yaml')
    print("Launch bag recorder")
    return LaunchDescription([

        # Launch the Pronto Solo node with parameters set in "state_estimator.yaml"
        Node(
            package='pronto_tuning',
            executable='tuning_node',
            parameters=[config_path]
        )
    ])
