"""
Stage B: Instruction-to-Goal Inference
Launch the query pipeline nodes. Assumes a semantic graph is available
(either from Stage A or loaded from JSON).
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    query_config = os.path.join(
        get_package_share_directory('semantic_map_query'),
        'config', 'query_params.yaml'
    )
    core_config = os.path.join(
        get_package_share_directory('semantic_map_core'),
        'config', 'core_params.yaml'
    )

    return LaunchDescription([
        # Graph store (to load persisted graph)
        Node(
            package='semantic_map_core',
            executable='graph_store_node',
            name='graph_store_node',
            parameters=[core_config],
            output='screen',
        ),

        # --- Query pipeline ---
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
