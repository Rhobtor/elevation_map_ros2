import os
import launch
import launch.actions
import launch.launch_description_sources
import launch.substitutions
import launch_ros.actions
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    share_dir = get_package_share_directory('elevation_mapping')
    config_dir = os.path.join(share_dir, 'config')
    list_params = []
    for filee in ["robots/velodyne_static.yaml", "elevation_maps/long_range.yaml", "sensor_processors/RSLidar-32.yaml", "postprocessing/postprocessor_pipeline.yaml"]:
        list_params.append(os.path.join(config_dir, filee))

    return launch.LaunchDescription(
        [
            launch.actions.DeclareLaunchArgument(
                'publish_sensor_observations', default_value='false',
                description='Export fresh Velodyne evidence; does not start the input filter'),
            launch_ros.actions.Node(
                package='elevation_mapping',
                executable='elevation_mapping_node',
                name='elevation_mapping',
                output='screen',
                parameters=list_params + [{
                    'sensor_observations.enabled': ParameterValue(
                        launch.substitutions.LaunchConfiguration('publish_sensor_observations'),
                        value_type=bool),
                }],
            ),
            #launch_ros.actions.Node(
            #package='robot_state_publisher',
            #executable='robot_state_publisher',
            #name='robot_state_publisher',
            #output='screen',
            #parameters=[{
            #    'robot_description': launch.substitutions.Command(['xacro', ' ', xacro_path])
            #}]
        #)
        ]
    )
