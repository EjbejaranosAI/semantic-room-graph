"""
Full Pipeline: Stage A + Stage B
Launch all nodes for both semantic graph construction and
instruction-to-goal inference in a single launch.
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
    query_config = os.path.join(
        get_package_share_directory('semantic_map_query'),
        'config', 'query_params.yaml'
    )

    return LaunchDescription([
        # ============================================================
        # Stage A: Semantic Graph Construction
        # ============================================================
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

        # ============================================================
        # Stage B: Instruction-to-Goal Inference
        # ============================================================
        Node(
            package='semantic_map_query',
            executable='instruction_parser_node',
            name='instruction_parser_node',
            parameters=[query_config],
            output='screen',
        ),
        Node(
            package='semantic_map_query',
            executable='semantic_retriever_node',
            name='semantic_retriever_node',
            parameters=[query_config],
            output='screen',
        ),
        Node(
            package='semantic_map_query',
            executable='goal_resolver_node',
            name='goal_resolver_node',
            parameters=[query_config],
            output='screen',
        ),
        Node(
            package='semantic_map_query',
            executable='task_navigator_node',
            name='task_navigator_node',
            parameters=[query_config],
            output='screen',
        ),
    ])
