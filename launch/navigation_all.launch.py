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
    lidar_localization_pkg = FindPackageShare('lidar_localization_ros2').find('lidar_localization_ros2')
    
    # 設定ファイルのパス
    nav2_params_file = os.path.expanduser('~/ros2_ws/src/msd_navigation/config/nav2_params.yaml')
    localization_rviz_config = os.path.expanduser('~/ros2_ws/src/lidar_localization_ros2/rviz/localization.rviz')
    nav2_rviz_config = os.path.expanduser('~/ros2_ws/src/msd_navigation/rviz/nav2_viewer.rviz')
    
    # Livoxドライバーの起動
    livox_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'livox_ros_driver2', 'rviz_MID360_launch.py'],
        output='screen'
    )
    
    # PointCloud to LaserScan変換の起動
    pointcloud_to_laserscan_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'msd_navigation', 'pointcloud_to_laserscan.launch.py'],
        output='screen'
    )
    
    # ロボットの表示用launch
    display_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'msd_mk4_description', 'display.launch.py'],
        output='screen'
    )
    
    # LiDARローカライゼーションの起動
    localization_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'lidar_localization_ros2', 'lidar_localization.launch.py'],
        output='screen'
    )
    
    # ローカライゼーション用RVizの起動
    localization_rviz = ExecuteProcess(
        cmd=['rviz2', '-d', localization_rviz_config],
        output='screen'
    )
    
    # Navigationの起動
    navigation_launch = ExecuteProcess(
        cmd=['ros2', 'launch', 'msd_navigation', 'navigation.launch.py', f'params_file:={nav2_params_file}'],
        output='screen'
    )
    
    # Navigation用RVizの起動
    nav2_rviz = ExecuteProcess(
        cmd=['rviz2', '-d', nav2_rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        livox_launch,
        pointcloud_to_laserscan_launch,
        display_launch,
        localization_launch,
        localization_rviz,
        navigation_launch,
        nav2_rviz
    ]) 