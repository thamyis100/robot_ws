from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('mecanum_bot'),
                'launch',
                'sim.launch.py',
            )
        )
    )

    twist_mux = Node(
        package='robot',
        executable='twist_mux',
        name='twist_mux',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    controller_node = Node(
        package='robot',
        executable='controller_node',
        name='controller_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'allow_no_device': True,
        }],
    )

    return LaunchDescription([sim_launch, twist_mux, controller_node])
