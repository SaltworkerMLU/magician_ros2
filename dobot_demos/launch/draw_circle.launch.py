import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config = os.path.join(
        get_package_share_directory('dobot_drawing'),
        'config',
        'draw_circle.yaml'
        )

    return LaunchDescription([
        Node(
            package='dobot_drawing',
            executable='draw_circle',
            output='screen',
        ),
    ExecuteProcess(
        cmd=[[
            'sleep 5;', 'ros2 param load /dobot_draw_circle ', config
        ]],
        shell=True
    )
    ])