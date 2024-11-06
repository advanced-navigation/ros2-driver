import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    config = os.path.join(
        get_package_share_directory('adnav_launch'),
        'config',
        'adnav_tcp_server.yaml'
        )
        
    node=Node(
        name = 'adnav_node',
        package = 'adnav_driver',
        executable = 'adnav_driver',
        emulate_tty = True,
        output = 'screen',
        # arguments=['--ros-args', '--log-level', 'debug'],
        parameters = [config]
    )

    ld.add_action

    ld.add_action(node)
    return ld