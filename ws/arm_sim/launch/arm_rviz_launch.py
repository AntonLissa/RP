from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command, PathJoinSubstitution

def generate_launch_description():

    # Percorso all'URDF
    urdf_file = PathJoinSubstitution([
        FindPackageShare('arm_sim'),
        'urdf',
        '2R_planar.urdf'
    ])

    # Nodo robot_state_publisher per pubblicare il robot_description
    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': Command(['cat ', urdf_file])}]
    )

    # Nodo del braccio
    arm_node = Node(
        package='arm_sim',
        executable='arm_node',
        name='arm_node',
        output='screen',
        parameters=[{
            'dh_file': PathJoinSubstitution([FindPackageShare('arm_sim'), 'config', '2R_planar.json']),
            'target_x': -0.3,
            'target_y': -0.2,
            'target_z': 0.0
        }]
    )

    # Nodo RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([FindPackageShare('arm_sim'), 'config', '2R_planar.rviz'])]
    )

    return LaunchDescription([
        robot_state_pub,
        arm_node,
        rviz_node
    ])
