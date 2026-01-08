from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # パッケージのパスを取得
    livox_pkg = FindPackageShare('livox_ros_driver2').find('livox_ros_driver2')
    msd_navigation_pkg = FindPackageShare('msd_navigation').find('msd_navigation')
    msd_mk4_description_pkg = FindPackageShare('msd_mk4_description').find('msd_mk4_description')
    
    # 設定ファイルのパス
    slam_toolbox_param_file = os.path.expanduser('~/ros2_ws/src/msd_mk4_description/config/slam_toolbox.yaml')
    
    # Livoxドライバーの起動
    livox_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'rviz_MID360_launch.py'],
        output='screen'
    )
    
    # GLIM ROSノードの起動
    glim_node = Node(
        package='glim_ros',
        executable='glim_rosnode',
        name='glim_rosnode',
        parameters=[{
            'config_path': os.path.expanduser('~/config')
        }],
        output='screen'
    )
    
    # PointCloud to LaserScan変換の起動
    pointcloud_to_laserscan_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'msd_navigation', 'pointcloud_to_laserscan.launch.py'],
        output='screen'
    )
    
    # SLAMの起動
    slam_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'msd_mk4_description', 'slam.launch.py'],
        output='screen'
    )
    
    # SLAM Toolboxの起動
    slam_toolbox_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'slam_toolbox', 'online_async_launch.py', f'param_file:={slam_toolbox_param_file}'],
        output='screen'
    )
    
    return LaunchDescription([
        livox_launch,
        glim_node,
        pointcloud_to_laserscan_launch,
        slam_launch,
        slam_toolbox_launch
    ])
