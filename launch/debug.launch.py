from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    param_file = PathJoinSubstitution(
        [FindPackageShare("rose_planner"), "config", "rose_planner.yaml"]
    )

    rose_planner_node = Node(
        package="rose_planner",
        executable="rose_planner_node",
        name="rose_planner",
        output="screen",
        arguments=["--ros-args", "--params-file", param_file],
        # 直接用 prefix 里执行 ros2 pkg prefix 并传入 gdb
        prefix='gnome-terminal -- bash -c "gdb -ex run --args $(ros2 pkg prefix rose_planner)/lib/rose_planner/rose_planner_node --ros-args --params-file $(ros2 pkg prefix rose_planner)/share/rose_planner/config/rose_planner.yaml; exec bash"',
    )

    return LaunchDescription([rose_planner_node])
