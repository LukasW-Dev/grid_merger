from launch import LaunchDescription
from launch_ros.actions import Node
import os

def generate_launch_description():
    
    filter_path = os.path.expanduser('~/ros2_ws/src/grid_merger/config/warthog_grid_merger.yaml')

    return LaunchDescription([
        Node(
            package='grid_merger',
            executable='grid_merger',
            name='grid_merger_node',
            output='screen',
            parameters=[ filter_path],
        )
    ])
