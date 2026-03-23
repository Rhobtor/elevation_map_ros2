import os

import launch
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('elevation_mapping')
    config_dir = os.path.join(share_dir, 'config')
    list_params = []

    for filee in [
        'robots/ground_truth_demo.yaml',
        'elevation_maps/long_range.yaml',
        'postprocessing/postprocessor_pipeline.yaml',
    ]:
        list_params.append(os.path.join(config_dir, filee))

    return launch.LaunchDescription(
        [
            launch_ros.actions.Node(
                package='elevation_mapping',
                executable='elevation_mapping_node',
                name='elevation_mapping',
                output='screen',
                parameters=list_params,
            ),
        ]
    )