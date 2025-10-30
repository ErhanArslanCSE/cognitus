#!/usr/bin/env python3
"""
COGNITUS Audio Monitor Node
Monitors environmental sounds

Features:
- Continuous audio level monitoring
- Loud sound detection
- Sound activity analysis
- Alerts brain to unusual sounds
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import queue


class AudioMonitor(Node):
    """
    Audio Monitor Node
    Monitors environmental sounds, detects unusual audio events
    """

    def __init__(self):
        super().__init__('audio_monitor')

        self.get_logger().info('='*50)
        self.get_logger().info('  🔉 COGNITUS Audio Monitor Initializing')
        self.get_logger().info('='*50)

        # Parameters
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('monitor_interval', 1.0)
        self.declare_parameter('loud_threshold', 5000)
        self.declare_parameter('quiet_threshold', 100)

        self.sample_rate = self.get_parameter('sample_rate').value
        self.monitor_interval = self.get_parameter('monitor_interval').value
        self.loud_threshold = self.get_parameter('loud_threshold').value
        self.quiet_threshold = self.get_parameter('quiet_threshold').value

        # State
        self.audio_queue = queue.Queue()
        self.audio_levels = []
        self.baseline_level = None
        self.pyaudio = None
        self.stream = None

        # Setup audio monitoring
        self._setup_audio()

        # Publish audio events to brain
        self.event_pub = self.create_publisher(
            String,
            '/audio/event',
            10
        )

        # Publish audio levels
        self.level_pub = self.create_publisher(
            String,
            '/audio/level',
            10
        )

        # Timer for monitoring
        self.timer = self.create_timer(
            self.monitor_interval,
            self._monitor_callback
        )

        self.get_logger().info('✓ Audio monitor ready')
        self.get_logger().info(f'  Loud threshold: {self.loud_threshold}')
        self.get_logger().info(f'  Quiet threshold: {self.quiet_threshold}')

    def _setup_audio(self):
        """Setup audio capture"""
        try:
            import pyaudio

            self.pyaudio = pyaudio.PyAudio()

            # Open microphone stream
            self.stream = self.pyaudio.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self.sample_rate,
                input=True,
                frames_per_buffer=1024,
                stream_callback=self._audio_callback
            )

            self.stream.start_stream()
            self.get_logger().info('✓ Audio monitoring active')

        except ImportError:
            self.get_logger().warn('⚠ pyaudio not installed')
            self.get_logger().info('  Running without audio monitoring')
            self.pyaudio = None
            self.stream = None
        except Exception as e:
            self.get_logger().warn(f'⚠ Audio setup failed: {e}')
            self.pyaudio = None
            self.stream = None

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """Audio stream callback"""
        audio_chunk = np.frombuffer(in_data, dtype=np.int16)
        self.audio_queue.put(audio_chunk)

        import pyaudio
        return (in_data, pyaudio.paContinue)

    def _monitor_callback(self):
        """Monitor audio levels"""
        if self.audio_queue.empty():
            return

        # Collect all queued audio
        audio_chunks = []
        while not self.audio_queue.empty():
            try:
                audio_chunks.append(self.audio_queue.get_nowait())
            except queue.Empty:
                break

        if not audio_chunks:
            return

        # Calculate audio level
        audio_data = np.concatenate(audio_chunks)
        audio_level = np.abs(audio_data).mean()

        # Store for baseline calculation
        self.audio_levels.append(audio_level)
        if len(self.audio_levels) > 100:  # Keep last 100 measurements
            self.audio_levels.pop(0)

        # Calculate baseline (median of recent levels)
        if len(self.audio_levels) > 10:
            self.baseline_level = np.median(self.audio_levels)

        # Publish current level
        level_msg = String()
        level_msg.data = f'level={audio_level:.0f}'
        self.level_pub.publish(level_msg)

        # Detect unusual sounds
        self._detect_unusual(audio_level)

    def _detect_unusual(self, current_level):
        """Detect unusual audio events"""
        # Loud sound detection
        if current_level > self.loud_threshold:
            event_msg = String()
            event_msg.data = f"Loud sound detected (level={current_level:.0f})"
            self.event_pub.publish(event_msg)

            self.get_logger().warn(f'🔊 LOUD SOUND: {current_level:.0f}')

        # Sudden change detection (if baseline established)
        elif self.baseline_level is not None:
            if current_level > self.baseline_level * 3:
                event_msg = String()
                event_msg.data = f"Sudden sound increase (level={current_level:.0f}, baseline={self.baseline_level:.0f})"
                self.event_pub.publish(event_msg)

                self.get_logger().info(f'🔔 Sound spike: {current_level:.0f} (baseline: {self.baseline_level:.0f})')

        # Very quiet detection
        elif current_level < self.quiet_threshold and self.baseline_level is not None:
            if current_level < self.baseline_level * 0.2:
                # Unusually quiet
                self.get_logger().info(f'🔇 Very quiet: {current_level:.0f}', throttle_duration_sec=60.0)

    def __del__(self):
        """Cleanup"""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.pyaudio:
            self.pyaudio.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = AudioMonitor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Audio monitor shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
