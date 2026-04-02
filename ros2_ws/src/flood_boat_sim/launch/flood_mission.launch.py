from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description() -> LaunchDescription:
    params_file = get_package_share_directory('flood_boat_sim') + '/params/mission.yaml'

    mission_node = Node(
        package='flood_boat_sim',
        executable='mission_node',
        name='mission_node',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([mission_node])
