import os
from ament_index_python.packages import get_package_share_directory

import launch
import launch.actions
import launch.substitutions
import launch_ros.actions


def generate_launch_description():
    ld = launch.LaunchDescription()

    config = os.path.join(
        get_package_share_directory('limovelo'),
        'config',
        'limovelo_config.yaml'
    )
    
    rviz_config_file = os.path.join(
        get_package_share_directory('limovelo'),
        'rviz',
        'LIMObueno.rviz'
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file]
    )

    slam_node = launch_ros.actions.Node(
        package='limovelo',
        executable='limovelo',
        output='screen',
        name='limovelo',
        parameters=[config],
    )

    
    ld.add_action(rviz_node)
    ld.add_action(slam_node)
    return ld
