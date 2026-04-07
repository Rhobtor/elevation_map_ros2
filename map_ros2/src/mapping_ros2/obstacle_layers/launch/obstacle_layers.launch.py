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
                        "input_map_topic": "/elevation_map_raw_post",
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

                        # Temporal: alpha controla la "memoria" del mapa de obstáculos.
                        # EMA: pe(t) = alpha*pe(t-1) + (1-alpha)*inst
                        #
                        # El problema con objetos MÓVILES:
                        # Con alpha=0.85 (anterior), cuando el objeto se va, la celda
                        # tarda ~20 frames (2s a 10Hz) en limpiar pe hasta <0.05.
                        # Un objeto que pasa a 1 m/s y celdas de 0.4m ocupa cada celda
                        # ~0.4s = ~4 frames → la celda no da tiempo a limpiarse antes
                        # de que el objeto avance → se crea un "rastro de muro fantasma".
                        #
                        # Con alpha=0.55:
                        #   - Tiempo hasta pe<0.05 desde pe=1.0: ~5 frames (0.5s) ✓
                        #   - Obstáculos FIJOS: llegan a pe≈0.45 en 3 frames y se mantienen ✓
                        #   - Tradeoff: el mapa "parpadea" ligeramente más, pero no deja rastro.
                        #
                        # Para objetos lentos (<0.3 m/s) o escenas totalmente estáticas,
                        # se puede subir alpha de vuelta a 0.75-0.85.
                        "alpha": 0.55,

                        # Sensor noise robustness
                        # variance_artifact_mult: variance > this × variance_max is treated
                        #   as a sensor artifact (LiDAR dust, ZED light change).
                        #   At 4×: quality 1.0→0.2 linearly, so a sudden glare spike barely
                        #   registers in persist_evidence.
                        "variance_artifact_mult": 4.0,
                        # spike_persist_warmup: persist_evidence level before a spike can
                        #   boost confidence. With alpha=0.85 a real object reaches 0.30
                        #   in ~3 frames (~0.3 s at 10 Hz). A 1-frame glint stays at 0.03.
                        "spike_persist_warmup": 0.30,

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
