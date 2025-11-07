import os

from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_path

def generate_launch_description():

    package_path = get_package_share_path("pronto_solo12")
    config_path = os.path.join(package_path, '/config/state_estimator.yaml')

    return LaunchDescription([

        # Launch the Pronto Solo node with parameters set in "state_estimator.yaml"
        Node(
            package='pronto_solo12',
            namespace='pronto_solo12',
            executable='pronto_estimator_node',
            name='pronto_estimator',
            parameters=[config_path]
        ),

        # # Converts TSIF Pose messages into TF
        # Node(
        #     package='pronto_anymal_b',
        #     executable='pose_to_tf.py',
        #     name='tsif_tf_publisher',
        #     output='log',
        #     arguments=['--config_file', FindPackageShare('pronto_anymal_b') + '/config/pose_to_tf.yaml'],
        # ),
    ])
