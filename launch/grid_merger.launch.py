from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    param = os.path.join(get_package_share_directory("grid_merger"), "config", "warthog_grid_merger.yaml")

    return LaunchDescription([
        Node(
            package='grid_merger',
            executable='grid_merger',
            name='grid_merger_node',
            output='screen',
            parameters=[ param],
        )
    ])
