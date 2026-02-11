from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    message_prefix = LaunchConfiguration('message_prefix')
    publish_period_sec = LaunchConfiguration('publish_period_sec')

    return LaunchDescription([
        DeclareLaunchArgument(
            'message_prefix',
            default_value='Hello ROS2',
            description='Prefix prepended to each published message.',
        ),
        DeclareLaunchArgument(
            'publish_period_sec',
            default_value='1.0',
            description='Seconds between talker messages.',
        ),
        Node(
            package='talker_pkg',
            executable='talker_node',
            name='talker_node',
            parameters=[{
                'message_prefix': message_prefix,
                'publish_period_sec': publish_period_sec,
            }],
            output='screen',
        ),
        Node(
            package='listener_pkg',
            executable='listener_node',
            name='listener_node',
            output='screen',
        ),
        Node(
            package='service_pkg',
            executable='service_server',
            name='service_server',
            output='screen',
        ),
    ])
