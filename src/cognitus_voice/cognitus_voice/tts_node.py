#!/usr/bin/env python3
"""
COGNITUS TTS Node
Text-to-Speech using Piper TTS

Receives text from brain, converts to speech,
plays through speakers

Features:
- Piper TTS (fast, lightweight)
- CPU-optimized (no GPU needed)
- Multiple voices support
- Graceful degradation (works without Piper for testing)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import subprocess
import os
import tempfile


class TTSNode(Node):
    """
    Text-to-Speech Node
    Brain → Piper → Speaker
    """

    def __init__(self):
        super().__init__('tts_node')

        self.get_logger().info('='*50)
        self.get_logger().info('  🔊 COGNITUS TTS Initializing')
        self.get_logger().info('='*50)

        # Parameters
        self.declare_parameter('voice', 'en_US-lessac-medium')
        self.declare_parameter('speaking_rate', 1.0)
        self.declare_parameter('sample_rate', 22050)
        self.declare_parameter('piper_path', '/usr/local/bin/piper')

        self.voice = self.get_parameter('voice').value
        self.speaking_rate = self.get_parameter('speaking_rate').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.piper_path = self.get_parameter('piper_path').value

        # State
        self.piper_available = False

        # Check if Piper is available
        self._check_piper()

        # Subscribe to brain responses
        self.response_sub = self.create_subscription(
            String,
            '/brain/response',
            self._response_callback,
            10
        )

        # Publish TTS status
        self.status_pub = self.create_publisher(
            String,
            '/voice/tts_status',
            10
        )

        self.get_logger().info('✓ TTS ready')
        self.get_logger().info(f'  Voice: {self.voice}')
        self.get_logger().info(f'  Rate: {self.speaking_rate}x')
        self.get_logger().info(f'  Piper available: {self.piper_available}')

    def _check_piper(self):
        """Check if Piper TTS is available"""
        # Method 1: Check if piper binary exists
        if os.path.exists(self.piper_path):
            self.piper_available = True
            self.get_logger().info(f'✓ Piper found at {self.piper_path}')
            return

        # Method 2: Try piper-tts Python package
        try:
            import piper
            self.piper_available = True
            self.get_logger().info('✓ Piper Python package available')
            return
        except ImportError:
            pass

        # Method 3: Check if piper command exists
        try:
            result = subprocess.run(
                ['which', 'piper'],
                capture_output=True,
                text=True,
                timeout=1
            )
            if result.returncode == 0:
                self.piper_path = result.stdout.strip()
                self.piper_available = True
                self.get_logger().info(f'✓ Piper found: {self.piper_path}')
                return
        except:
            pass

        # Not available
        self.get_logger().warn('⚠ Piper TTS not installed')
        self.get_logger().info('  Install: pip install piper-tts')
        self.get_logger().info('  OR download from: https://github.com/rhasspy/piper')
        self.get_logger().info('  Running in TEST mode (text only)')

    def _response_callback(self, msg):
        """Receive response from brain and speak it"""
        text = msg.data

        self.get_logger().info(f'🔊 Speaking: "{text}"')

        if self.piper_available:
            # Real TTS
            self._synthesize_speech(text)
        else:
            # Test mode - just log
            self.get_logger().info(f'  [TEST MODE - Would say]: {text}')

        # Update status
        status = String()
        status.data = f'spoken: {text[:50]}'
        self.status_pub.publish(status)

    def _synthesize_speech(self, text):
        """Synthesize and play speech with Piper"""
        try:
            # Create temporary WAV file
            with tempfile.NamedTemporaryFile(suffix='.wav', delete=False) as tmp_file:
                wav_path = tmp_file.name

            # Method 1: Try Piper binary
            if os.path.exists(self.piper_path):
                # Piper command line
                # echo "text" | piper --model voice.onnx --output_file output.wav
                model_path = f'models/piper/{self.voice}.onnx'

                if not os.path.exists(model_path):
                    self.get_logger().warn(f'⚠ Voice model not found: {model_path}')
                    self.get_logger().info('  Falling back to simple TTS')
                    self._fallback_tts(text)
                    return

                # Run Piper
                process = subprocess.Popen(
                    ['echo', text],
                    stdout=subprocess.PIPE
                )

                subprocess.run(
                    [self.piper_path, '--model', model_path, '--output_file', wav_path],
                    stdin=process.stdout,
                    check=True,
                    timeout=5
                )

                # Play WAV file
                self._play_wav(wav_path)

            else:
                # Method 2: Python package
                self._fallback_tts(text)

            # Cleanup
            if os.path.exists(wav_path):
                os.remove(wav_path)

        except subprocess.TimeoutExpired:
            self.get_logger().error('TTS timeout')
        except Exception as e:
            self.get_logger().error(f'TTS error: {e}')
            self._fallback_tts(text)

    def _play_wav(self, wav_path):
        """Play WAV file through speakers"""
        try:
            # Try aplay (Linux)
            subprocess.run(['aplay', wav_path], check=True, timeout=10)
        except FileNotFoundError:
            try:
                # Try paplay (PulseAudio)
                subprocess.run(['paplay', wav_path], check=True, timeout=10)
            except:
                self.get_logger().error('No audio player found (aplay or paplay)')
        except Exception as e:
            self.get_logger().error(f'Audio playback error: {e}')

    def _fallback_tts(self, text):
        """Fallback TTS using espeak (simpler)"""
        try:
            # espeak is lighter than Piper, good fallback
            subprocess.run(
                ['espeak', '-s', str(int(150 * self.speaking_rate)), text],
                check=True,
                timeout=10
            )
        except FileNotFoundError:
            self.get_logger().warn('⚠ espeak not installed either')
            self.get_logger().info('  Install: sudo apt-get install espeak')
        except Exception as e:
            self.get_logger().error(f'Fallback TTS error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = TTSNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('TTS shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
