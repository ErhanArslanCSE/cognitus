#!/usr/bin/env python3
"""
COGNITUS STT Node
Speech-to-Text using Whisper Tiny

Captures audio from microphone, detects speech,
transcribes to text, sends commands to brain

Features:
- Wake word detection ("Hey Cognitus")
- Voice Activity Detection (VAD)
- Whisper Tiny inference
- Graceful degradation (works without Whisper for testing)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import numpy as np
import queue
import threading


class STTNode(Node):
    """
    Speech-to-Text Node
    Microphone → Whisper → Brain
    """

    def __init__(self):
        super().__init__('stt_node')

        self.get_logger().info('='*50)
        self.get_logger().info('  🎤 COGNITUS STT Initializing')
        self.get_logger().info('='*50)

        # Parameters
        self.declare_parameter('model_size', 'tiny')
        self.declare_parameter('language', 'en')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('chunk_duration', 3.0)
        self.declare_parameter('wake_word_enabled', True)
        self.declare_parameter('wake_words', ['hey cognitus', 'cognitus', 'robot'])

        self.model_size = self.get_parameter('model_size').value
        self.language = self.get_parameter('language').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.chunk_duration = self.get_parameter('chunk_duration').value
        self.wake_word_enabled = self.get_parameter('wake_word_enabled').value
        self.wake_words = self.get_parameter('wake_words').value

        # State
        self.audio_queue = queue.Queue()
        self.is_listening = False
        self.model = None
        self.pyaudio = None
        self.stream = None

        # Load Whisper model
        self._load_whisper()

        # Setup audio capture
        self._setup_audio()

        # Publish transcribed commands to brain
        self.command_pub = self.create_publisher(
            String,
            '/voice/command',
            10
        )

        # Publish STT status
        self.status_pub = self.create_publisher(
            String,
            '/voice/stt_status',
            10
        )

        # Timer for processing audio
        self.timer = self.create_timer(0.5, self._process_audio)

        # For testing without microphone
        if self.model is None or self.stream is None:
            self.get_logger().warn('Running in TEST mode (no Whisper or audio)')
            self.test_timer = self.create_timer(20.0, self._test_mode)

        self.get_logger().info('✓ STT ready')
        self.get_logger().info(f'  Model: Whisper {self.model_size}')
        self.get_logger().info(f'  Language: {self.language}')
        self.get_logger().info(f'  Wake word: {self.wake_word_enabled}')

    def _load_whisper(self):
        """Load Whisper model"""
        try:
            import whisper
            self.model = whisper.load_model(self.model_size)
            self.get_logger().info(f'✓ Whisper {self.model_size} loaded')
        except ImportError:
            self.get_logger().warn('⚠ openai-whisper not installed')
            self.get_logger().info('  Install: pip install openai-whisper')
            self.model = None
        except Exception as e:
            self.get_logger().warn(f'⚠ Whisper load failed: {e}')
            self.model = None

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
            self.get_logger().info('✓ Microphone active')

        except ImportError:
            self.get_logger().warn('⚠ pyaudio not installed')
            self.get_logger().info('  Install: pip install pyaudio')
            self.pyaudio = None
            self.stream = None
        except Exception as e:
            self.get_logger().warn(f'⚠ Audio setup failed: {e}')
            self.get_logger().info('  Check microphone connection')
            self.pyaudio = None
            self.stream = None

    def _audio_callback(self, in_data, frame_count, time_info, status):
        """Audio stream callback"""
        # Add audio to queue for processing
        audio_chunk = np.frombuffer(in_data, dtype=np.int16)
        self.audio_queue.put(audio_chunk)

        import pyaudio
        return (in_data, pyaudio.paContinue)

    def _process_audio(self):
        """Process audio queue"""
        if self.model is None or self.audio_queue.empty():
            return

        # Collect chunks for transcription duration
        chunks_needed = int(self.sample_rate * self.chunk_duration / 1024)
        audio_chunks = []

        try:
            for _ in range(min(chunks_needed, self.audio_queue.qsize())):
                audio_chunks.append(self.audio_queue.get_nowait())
        except queue.Empty:
            return

        if not audio_chunks:
            return

        # Concatenate audio
        audio_data = np.concatenate(audio_chunks)

        # Simple VAD - check if audio has enough energy
        audio_energy = np.abs(audio_data).mean()
        if audio_energy < 100:  # Threshold for silence
            return

        # Transcribe with Whisper
        self._transcribe(audio_data)

    def _transcribe(self, audio_data):
        """Transcribe audio with Whisper"""
        try:
            # Convert to float32 and normalize
            audio_float = audio_data.astype(np.float32) / 32768.0

            # Whisper inference
            result = self.model.transcribe(
                audio_float,
                language=self.language,
                fp16=False
            )

            text = result['text'].strip()

            if not text:
                return

            self.get_logger().info(f'🎤 Heard: "{text}"')

            # Check wake word
            if self.wake_word_enabled:
                if not self._check_wake_word(text):
                    self.get_logger().info('  (Ignoring - no wake word)')
                    return

            # Send command to brain
            self._send_command(text)

        except Exception as e:
            self.get_logger().error(f'Transcription error: {e}')

    def _check_wake_word(self, text):
        """Check if text contains wake word"""
        text_lower = text.lower()
        for wake_word in self.wake_words:
            if wake_word in text_lower:
                self.get_logger().info('  ✓ Wake word detected')
                # Remove wake word from text
                return True
        return False

    def _send_command(self, text):
        """Send command to brain"""
        msg = String()
        msg.data = text
        self.command_pub.publish(msg)

        # Update status
        status = String()
        status.data = f'transcribed: {text[:50]}'
        self.status_pub.publish(status)

    def _test_mode(self):
        """Test mode - simulates voice commands"""
        test_commands = [
            "Hey Cognitus, what do you see?",
            "Hey Cognitus, is anything unusual?",
            "Cognitus, tell me what you remember",
        ]

        import random
        command = random.choice(test_commands)

        self.get_logger().info(f'🎤 [TEST MODE] Simulated: "{command}"')
        self._send_command(command)

    def __del__(self):
        """Cleanup"""
        if self.stream:
            self.stream.stop_stream()
            self.stream.close()
        if self.pyaudio:
            self.pyaudio.terminate()


def main(args=None):
    rclpy.init(args=args)
    node = STTNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('STT shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
