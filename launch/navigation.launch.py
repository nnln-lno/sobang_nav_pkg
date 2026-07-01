import os
import launch

from launch import LaunchDescription
from launch_ros.actions import Node 
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    nav_pkg_dir = get_package_share_directory('sobang_navigation')
    nav_param_file = os.path.join(nav_pkg_dir, 'param', 'nav_params.yaml')
    
    rviz_dir = os.path.join(nav_pkg_dir, 'rviz', 'navigation_visualizer.rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'nav_param_file',
            default_value=nav_param_file,
            description='Path to the navigation parameter file'
        ),
        Node(
            package='sobang_navigation',
            executable='sobang_navigation_node',
            output='screen',            
            parameters=[LaunchConfiguration('nav_param_file')]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_dir]
        )
    ])