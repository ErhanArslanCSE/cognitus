#!/usr/bin/env python3
"""
COGNITUS Demo Launch
Complete system with all nodes
Works in Docker and LIMO Pro
"""

from launch import LaunchDescription
from launch.actions import LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['═══════════════════════════════════════════════']),
        LogInfo(msg=['  🚀 COGNITUS Full System Launch']),
        LogInfo(msg=['═══════════════════════════════════════════════']),

        # === PERCEPTION ===
        Node(
            package='cognitus_perception',
            executable='perception_node',
            name='perception',
            output='screen',
        ),

        Node(
            package='cognitus_perception',
            executable='scene_graph_builder',
            name='scene_graph',
            output='screen',
        ),

        Node(
            package='cognitus_perception',
            executable='anomaly_detector',
            name='anomaly',
            output='screen',
        ),

        # === MEMORY ===
        Node(
            package='cognitus_memory',
            executable='memory_manager_node',
            name='memory_manager',
            output='screen',
        ),

        # === VOICE ===
        Node(
            package='cognitus_voice',
            executable='stt_node',
            name='stt',
            output='screen',
        ),

        Node(
            package='cognitus_voice',
            executable='tts_node',
            name='tts',
            output='screen',
        ),

        Node(
            package='cognitus_voice',
            executable='audio_monitor',
            name='audio_monitor',
            output='screen',
        ),

        # === COGNITION (BRAIN) ===
        Node(
            package='cognitus_cognition',
            executable='brain_node',
            name='cognitus_brain',
            output='screen',
        ),

        # === BEHAVIORS ===
        Node(
            package='cognitus_behaviors',
            executable='behavior_controller_node',
            name='behavior_controller',
            output='screen',
        ),

        LogInfo(msg=['✓ All 9 nodes launched']),
        LogInfo(msg=['Monitor: ros2 node list']),
        LogInfo(msg=['Echo brain: ros2 topic echo /brain/status']),
    ])
