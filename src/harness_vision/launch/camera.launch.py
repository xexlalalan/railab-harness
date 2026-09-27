import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cfg = os.path.join(get_package_share_directory("harness_vision"), "config", "camera.yaml")
    return LaunchDescription([Node(package="harness_vision", executable="tag_node", name="tag_node",
                                   parameters=[cfg], output="screen", emulate_tty=True)])
