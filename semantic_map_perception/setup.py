from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'semantic_map_perception'

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
    description='Perception nodes for semantic room graph pipeline',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_collector_node = semantic_map_perception.perception_collector_node:main',
            'ocr_node = semantic_map_perception.ocr_node:main',
            'vlm_labeler_node = semantic_map_perception.vlm_labeler_node:main',
        ],
    },
)
