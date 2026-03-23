"""
Stage A: Semantic Graph Construction
Launch all nodes needed to build the semantic room graph from /map.
Pipeline: map_ingestor -> segmentation -> graph_builder -> pose_gen ->
          exploration -> navigator -> perception -> OCR/VLM -> fusion -> store
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    core_config = os.path.join(
        get_package_share_directory('semantic_map_core'),
        'config', 'core_params.yaml'
    )
    perception_config = os.path.join(
        get_package_share_directory('semantic_map_perception'),
        'config', 'perception_params.yaml'
    )

    return LaunchDescription([
        # --- Core pipeline ---
        Node(
            package='semantic_map_core',
            executable='map_ingestor_node',
            name='map_ingestor_node',
            parameters=[core_config],
            output='screen',
        ),
        Node(
            package='semantic_map_core',
            executable='region_segmentation_node',
            name='region_segmentation_node',
            parameters=[core_config],
            output='screen',
        ),
        Node(
            package='semantic_map_core',
            executable='topological_graph_builder_node',
            name='topological_graph_builder_node',
            parameters=[core_config],
            output='screen',
        ),
        Node(
            package='semantic_map_core',
            executable='pose_generator_node',
            name='pose_generator_node',
            parameters=[core_config],
            output='screen',
        ),
        Node(
            package='semantic_map_core',
            executable='exploration_planner_node',
            name='exploration_planner_node',
            parameters=[core_config],
            output='screen',
        ),
        Node(
            package='semantic_map_core',
            executable='navigator_executor_node',
            name='navigator_executor_node',
            parameters=[core_config],
            output='screen',
        ),

        # --- Perception pipeline ---
        Node(
            package='semantic_map_perception',
            executable='perception_collector_node',
            name='perception_collector_node',
            parameters=[perception_config],
            output='screen',
        ),
        Node(
            package='semantic_map_perception',
            executable='ocr_node',
            name='ocr_node',
            parameters=[perception_config],
            output='screen',
        ),
        Node(
            package='semantic_map_perception',
            executable='vlm_labeler_node',
            name='vlm_labeler_node',
            parameters=[perception_config],
            output='screen',
        ),

        # --- Fusion + storage ---
        Node(
            package='semantic_map_core',
            executable='semantic_fusion_node',
            name='semantic_fusion_node',
            parameters=[core_config],
            output='screen',
        ),
        Node(
            package='semantic_map_core',
            executable='graph_store_node',
            name='graph_store_node',
            parameters=[core_config],
            output='screen',
        ),

        # --- Visualization ---
        Node(
            package='semantic_map_core',
            executable='graph_visualizer_node',
            name='graph_visualizer_node',
            output='screen',
        ),
    ])
