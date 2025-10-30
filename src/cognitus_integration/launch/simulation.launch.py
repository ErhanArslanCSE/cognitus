#!/usr/bin/env python3
"""
COGNITUS Simulation Launch
Full system with Gazebo simulation
All nodes with use_sim_time=true
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=['═══════════════════════════════════════════════']),
        LogInfo(msg=['  🌍 COGNITUS Simulation Launch']),
        LogInfo(msg=['═══════════════════════════════════════════════']),

        # Start Gazebo
        LogInfo(msg=['Starting Gazebo...']),
        ExecuteProcess(
            cmd=['gazebo', '--verbose'],
            output='screen',
        ),

        # === PERCEPTION (3 nodes) ===
        Node(
            package='cognitus_perception',
            executable='perception_node',
            name='perception',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='cognitus_perception',
            executable='scene_graph_builder',
            name='scene_graph',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='cognitus_perception',
            executable='anomaly_detector',
            name='anomaly',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # === MEMORY ===
        Node(
            package='cognitus_memory',
            executable='memory_manager_node',
            name='memory_manager',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # === VOICE (3 nodes) ===
        Node(
            package='cognitus_voice',
            executable='stt_node',
            name='stt',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='cognitus_voice',
            executable='tts_node',
            name='tts',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        Node(
            package='cognitus_voice',
            executable='audio_monitor',
            name='audio_monitor',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # === COGNITION (BRAIN) ===
        Node(
            package='cognitus_cognition',
            executable='brain_node',
            name='cognitus_brain',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        # === BEHAVIORS ===
        Node(
            package='cognitus_behaviors',
            executable='behavior_controller_node',
            name='behavior_controller',
            output='screen',
            parameters=[{'use_sim_time': True}],
        ),

        LogInfo(msg=['✓ All 9 nodes launched (simulation mode)']),
        LogInfo(msg=['Check nodes: ros2 node list']),
        LogInfo(msg=['Check topics: ros2 topic list']),
    ])
