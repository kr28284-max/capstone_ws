from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package='vision_pkg',
                executable='vision_master_node3',
                name='vision_node3',
                output='screen',
            ),
            ExecuteProcess(
                cmd=[
                    'gnome-terminal',
                    '--',
                    'bash',
                    '-lc',
                    'ros2 run control_pkg test_node5; exec bash',
                ],
                output='screen',
            ),
        ]
    )
