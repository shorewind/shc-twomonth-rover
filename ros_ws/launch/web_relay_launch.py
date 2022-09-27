from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="pico_relay",
            executable="pico_relay",
            name="pico_relay"
        ),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket"
        )
    ])