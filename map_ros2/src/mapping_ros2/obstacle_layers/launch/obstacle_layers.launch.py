from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="obstacle_layers",
                executable="obstacle_layers_node",
                name="obstacle_layers",
                output="screen",
                parameters=[
                    {
                        # Topics
                        "input_map_topic": "/elevation_mapping/elevation_map_raw",
                        "output_map_topic": "/elevation_mapping/elevation_map_obstacles",
                        "input_pointcloud_topic": "",  # set to e.g. "/points_fused" if available

                        # Input layer names
                        "elevation_layer": "elevation",
                        "variance_layer": "variance",
                        "slope_layer": "slope",
                        "step_layer": "step",
                        "rough_layer": "rough",
                        "clearance_layer": "clearance",
                        "negatives_layer": "negatives",

                        # Support surface
                        "support_window_m": 0.45,
                        "support_percentile": 0.20,
                        "variance_min": 0.01,
                        "variance_max": 0.05,

                        # Robot / evidence params
                        "step_hard": 0.25,
                        "rough_hard": 0.10,
                        "slope_hard": 0.52,
                        "neg_hard": 0.25,
                        "h_soft": 0.10,
                        "h_hard": 0.25,
                        "clear_min": 0.40,
                        "h_climb": 0.10,
                        "h_body": 0.45,

                        # Temporal
                        "alpha": 0.85,

                        # Probability & cost
                        "kh": 8.0,
                        "ks": 6.0,
                        "Th": 0.55,
                        "Ts": 0.45,
                        "wh": 80.0,
                        "ws": 15.0,
                        "wu": 8.0,

                        # Debug markers
                        "publish_markers": True,
                        "hard_marker_threshold": 0.7,
                        "unknown_marker_threshold": 0.6,
                    }
                ],
            )
        ]
    )
