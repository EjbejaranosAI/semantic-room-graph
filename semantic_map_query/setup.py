from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_map_query'

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
    description='Query nodes for instruction-to-goal inference',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'instruction_parser_node = semantic_map_query.instruction_parser_node:main',
            'semantic_retriever_node = semantic_map_query.semantic_retriever_node:main',
            'goal_resolver_node = semantic_map_query.goal_resolver_node:main',
            'task_navigator_node = semantic_map_query.task_navigator_node:main',
        ],
    },
)
