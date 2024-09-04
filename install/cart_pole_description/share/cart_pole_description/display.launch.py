from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='urdf/cart_pole.urdf',
                              description='Path to the URDF file'),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=[
                '-file', LaunchConfiguration('model'),
                '-entity', 'cart_pole'
            ],
            output='screen'
        )
    ])
