import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory('arm_sim')

    urdf_path = os.path.join(pkg_share, 'urdf', '3R.urdf')
    rviz_config_file = os.path.join(pkg_share, 'config', 'visualizer.rviz')
    dh_file = os.path.join(pkg_share, 'config', '3R.json')

    # Carica URDF
    with open(urdf_path, 'r') as infp:
        robot_desc = infp.read()

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_desc}]
        ),

        Node(
            package='arm_sim',
            executable='arm_node',
            name='arm_node',
            output='screen',
            parameters=[{
                'dh_file': dh_file,
                'target_x': -0.9,
                'target_y': -0.5,
                'target_z': 0.0
            }]
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file]
        )
    ])
