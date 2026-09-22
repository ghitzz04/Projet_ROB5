import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('mon_deuxieme_package')
    slam_params_file = os.path.join(pkg_dir, 'config', 'slam_params.yaml')

    projection_node = Node(
        package='mon_deuxieme_package',
        executable='test_node',
        name='pointcloud_to_scan_node',
        output='screen'
    )

    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params_file]
    )

    return LaunchDescription([
        projection_node,
        slam_toolbox_node
    ])
