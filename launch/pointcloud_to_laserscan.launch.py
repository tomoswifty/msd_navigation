from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/livox/lidar'),
                        ('scan', '/scan')],
            parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.01,
                'min_height': -0.250,
                'max_height': 0.100,
                'angle_min': -3.1415,  # -M_PI/2
                'angle_max': 3.1415,  # M_PI/2
                # 'angle_min': -1.5708,  # -M_PI/2
                # 'angle_max': 1.5708, 
                'angle_increment': 0.0087,  # M_PI/360.0
                'scan_time': 0.3333,
                'range_min': 0.05,
                'range_max': 30.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan'
        ),
        Node(
            package='pointcloud_to_laserscan', executable='pointcloud_to_laserscan_node',
            remappings=[('cloud_in', '/camera/camera/depth/color/points'),
                        ('scan', '/scan_realsense')],
            parameters=[{
                'target_frame': '',
                'transform_tolerance': 0.01,
                'min_height': -0.100,
                'max_height': 0.500,
                # 'angle_min': -3.1415,  # -M_PI/2
                # 'angle_max': 3.1415,  # M_PI/2
                'angle_min': -1.5708,
                'angle_max': 1.5708,
                'angle_increment': 0.0087,
                'scan_time': 0.3333,
                'range_min': 0.1,
                'range_max': 4.0,
                'use_inf': True,
                'inf_epsilon': 1.0
            }],
            name='pointcloud_to_laserscan_realsense'
        ),
        # lidar_filtersのノード起動設定
        Node(
            package   = 'laser_filters',
            executable= 'scan_to_scan_filter_chain',
            parameters=['/home/minashigo/minashigo_ws/src/msd_navigation/config/laser_filter.yaml'],
        )
    ])
