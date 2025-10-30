from setuptools import setup
import os
from glob import glob

package_name = 'cognitus_voice'

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
    description='Voice interface for COGNITUS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stt_node = cognitus_voice.stt_node:main',
            'tts_node = cognitus_voice.tts_node:main',
            'audio_monitor = cognitus_voice.audio_monitor:main',
        ],
    },
)
