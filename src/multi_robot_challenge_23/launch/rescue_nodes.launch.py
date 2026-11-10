from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Which wall each robot follows. In most worlds tb3_0 starts above (bigger y)
    # tb3_1, so tb3_0 follows the right wall (goes up) and tb3_1 the left wall (goes down).
    side_0 = LaunchConfiguration('tb3_0_side')
    side_1 = LaunchConfiguration('tb3_1_side')

    return LaunchDescription([
        DeclareLaunchArgument('tb3_0_side', default_value='right'),
        DeclareLaunchArgument('tb3_1_side', default_value='left'),

        Node(package='multi_robot_challenge_23', executable='leader',
             name='leader', output='screen'),

        Node(package='multi_robot_challenge_23', executable='robot_handler',
             name='robot_handler_0', output='screen',
             parameters=[{'namespace': 'tb3_0', 'side': side_0}]),

        Node(package='multi_robot_challenge_23', executable='robot_handler',
             name='robot_handler_1', output='screen',
             parameters=[{'namespace': 'tb3_1', 'side': side_1}]),
    ])
