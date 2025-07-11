from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='b3rb_ros_aim_india',
            executable='explore',  # Use entry point from setup.py
            name='warehouse_node',
            output='screen',
            parameters=[
                {'shelf_count': 5},          # Optional: adjust as needed
                {'initial_angle': 0.0}       # Optional
            ]
        )
    ])
