import os

import ament_index_python.packages
import launch
import launch_ros.actions


def generate_launch_description():

    config_dir = os.path.join(
        ament_index_python.packages.get_package_share_directory('pointcloud_filter'),'config')

    params = os.path.join(config_dir, 'PC_Filter_params.yaml')
    ros_node = launch_ros.actions.Node(package='pointcloud_filter',
                                       executable='pcfilter',
                                       output='both',
                                       parameters=[params])

    return launch.LaunchDescription([ros_node,
                                     launch.actions.RegisterEventHandler(
                                        event_handler=launch.event_handlers.OnProcessExit(
                                            target_action=ros_node,
                                            on_exit=[
                                                launch.actions.EmitEvent(event=launch.events.Shutdown())
                                            ],
                                        )
                                     ),
    ])
