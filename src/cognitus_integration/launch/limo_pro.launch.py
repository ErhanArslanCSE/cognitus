#!/usr/bin/env python3
"""
COGNITUS LIMO Pro Launch
Full system on real LIMO Pro hardware
Uses pre-installed LIMO drivers + Navigation2
All nodes with use_sim_time=false
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
import os


def generate_launch_description():
    nodes = []

    nodes.append(LogInfo(msg=['═══════════════════════════════════════════════']))
    nodes.append(LogInfo(msg=['  🤖 COGNITUS on LIMO Pro']))
    nodes.append(LogInfo(msg=['═══════════════════════════════════════════════']))

    # Try to start LIMO base drivers (pre-installed on LIMO Pro)
    try:
        limo_bringup = get_package_share_directory('limo_bringup')
        limo_launch = os.path.join(limo_bringup, 'launch', 'limo_base.launch.py')

        if os.path.exists(limo_launch):
            nodes.append(LogInfo(msg=['✓ Starting LIMO drivers...']))
            nodes.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(limo_launch)
                )
            )
        else:
            nodes.append(LogInfo(msg=['⚠ LIMO launch file not found']))
    except PackageNotFoundError:
        nodes.append(LogInfo(msg=['⚠ limo_bringup not found (normal in Docker)']))

    # === PERCEPTION (3 nodes) ===
    nodes.extend([
        Node(
            package='cognitus_perception',
            executable='perception_node',
            name='perception',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        Node(
            package='cognitus_perception',
            executable='scene_graph_builder',
            name='scene_graph',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        Node(
            package='cognitus_perception',
            executable='anomaly_detector',
            name='anomaly',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])

    # === MEMORY ===
    nodes.append(
        Node(
            package='cognitus_memory',
            executable='memory_manager_node',
            name='memory_manager',
            output='screen',
            parameters=[{'use_sim_time': False}],
        )
    )

    # === VOICE (3 nodes) ===
    nodes.extend([
        Node(
            package='cognitus_voice',
            executable='stt_node',
            name='stt',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        Node(
            package='cognitus_voice',
            executable='tts_node',
            name='tts',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),

        Node(
            package='cognitus_voice',
            executable='audio_monitor',
            name='audio_monitor',
            output='screen',
            parameters=[{'use_sim_time': False}],
        ),
    ])

    # === COGNITION (BRAIN) ===
    nodes.append(
        Node(
            package='cognitus_cognition',
            executable='brain_node',
            name='cognitus_brain',
            output='screen',
            parameters=[{'use_sim_time': False}],
        )
    )

    # === BEHAVIORS ===
    nodes.append(
        Node(
            package='cognitus_behaviors',
            executable='behavior_controller_node',
            name='behavior_controller',
            output='screen',
            parameters=[{'use_sim_time': False}],
        )
    )

    nodes.append(LogInfo(msg=['✓ COGNITUS fully operational (LIMO Pro mode)']))
    nodes.append(LogInfo(msg=['Check status: ros2 topic echo /brain/status']))
    nodes.append(LogInfo(msg=['View topics: ros2 topic list']))

    return LaunchDescription(nodes)
