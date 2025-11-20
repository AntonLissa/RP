from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='arm_sim',
            executable='arm_node',
            name='arm_node',
            parameters=[{
                'dh_file': '/home/tony/progetti_ros/RP/ws/arm_sim/config/2R_planar.json',
                'target_x': 0.7,
                'target_y': 0.7,
                'target_z': 0.0
            }]
        )
    ])
