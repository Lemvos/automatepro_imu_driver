from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('automatepro_imu_driver'), 
        'config',
        'imu_params.yaml'
    )

    return LaunchDescription([
        Node(
            package='automatepro_imu_driver',  
            executable='imu_driver',  
            name='imu_driver',
            output='screen',
            parameters=[config]
        ),
    ])
