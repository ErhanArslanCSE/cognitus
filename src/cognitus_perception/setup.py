from setuptools import setup
import os
from glob import glob

package_name = 'cognitus_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='COGNITUS Team',
    maintainer_email='cognitus@example.com',
    description='Visual perception and scene understanding for COGNITUS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'perception_node = cognitus_perception.perception_node:main',
            'scene_graph_builder = cognitus_perception.scene_graph_builder:main',
            'anomaly_detector = cognitus_perception.anomaly_detector:main',
        ],
    },
)
