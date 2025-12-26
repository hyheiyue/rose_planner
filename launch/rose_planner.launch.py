from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rose_planner_node = Node(
        package="rose_planner",
        executable="rose_planner_node",
        name="rose_planner_node",
        output="screen",
        parameters=[
            PathJoinSubstitution(
                [
                    FindPackageShare("rose_planner"),
                    "config",
                    "rose_planner.yaml",
                ]
            )
        ],
    )

    return LaunchDescription([rose_planner_node])
