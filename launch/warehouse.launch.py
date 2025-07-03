from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='b3rb_ros_aim_india',
            executable='b3rb_ros_warehouse.py',
            name='warehouse_node',
            output='screen'
        )
    ])

