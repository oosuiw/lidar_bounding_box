from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('lidar_bounding_box'),
        'config',
        'params.yaml'
    )

    return LaunchDescription([
        Node(
            package='lidar_bounding_box',
            executable='lidar_bounding_box_node',
            name='lidar_bounding_box_node',
            output='screen',
            parameters=[config],
            remappings=[
                ('/input_yolo_topic', '/yolo_result'),
                ('/input_camera_info_topic', '/camera_info'),
                ('/input_point_cloud_topic', '/point_cloud'),
                ('/output_bounding_box_topic', '/lidar_bounding_box')
            ]
        )
    ])