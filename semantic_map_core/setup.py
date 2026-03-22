from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_map_core'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ebejarano',
    maintainer_email='ebejarano@example.com',
    description='Core nodes for semantic room graph construction',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'map_ingestor_node = semantic_map_core.map_ingestor_node:main',
            'region_segmentation_node = semantic_map_core.region_segmentation_node:main',
            'topological_graph_builder_node = semantic_map_core.topological_graph_builder_node:main',
            'pose_generator_node = semantic_map_core.pose_generator_node:main',
            'exploration_planner_node = semantic_map_core.exploration_planner_node:main',
            'navigator_executor_node = semantic_map_core.navigator_executor_node:main',
            'semantic_fusion_node = semantic_map_core.semantic_fusion_node:main',
            'graph_store_node = semantic_map_core.graph_store_node:main',
        ],
    },
)
