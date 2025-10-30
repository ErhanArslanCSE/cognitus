#!/usr/bin/env python3
"""
COGNITUS Brain Node
Central decision-making system using LLM

Receives information from all modules:
  - Perception: Scene descriptions
  - Voice: User commands
  - Memory: Retrieved memories
  - Audio: Sound events

Makes decisions and sends:
  - Responses to user (via TTS)
  - Commands to behaviors
  - Queries to memory

Features:
  - Generic LLM backend (config-selectable)
  - Context management
  - Multi-source information fusion
  - Decision logging
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from .llm_interface import LLMInterface
from .context_manager import ContextManager


class CognitBrainNode(Node):
    """
    COGNITUS Brain - Central Intelligence
    """

    def __init__(self):
        super().__init__('cognitus_brain')

        self.get_logger().info('='*60)
        self.get_logger().info('  🧠 COGNITUS BRAIN Initializing')
        self.get_logger().info('='*60)

        # Parameters
        self.declare_parameter('llm_model_name', 'microsoft/Phi-3.5-mini-instruct')
        self.declare_parameter('llm_quantization', '4bit')
        self.declare_parameter('max_tokens', 150)
        self.declare_parameter('temperature', 0.7)
        self.declare_parameter('system_prompt', '')
        self.declare_parameter('max_conversation_history', 10)
        self.declare_parameter('enable_llm', True)

        model_name = self.get_parameter('llm_model_name').value
        quantization = self.get_parameter('llm_quantization').value
        max_tokens = self.get_parameter('max_tokens').value
        temperature = self.get_parameter('temperature').value
        system_prompt = self.get_parameter('system_prompt').value
        max_history = self.get_parameter('max_conversation_history').value
        enable_llm = self.get_parameter('enable_llm').value

        # Initialize context manager
        self.context = ContextManager(
            system_prompt=system_prompt if system_prompt else None,
            max_history=max_history
        )
        self.get_logger().info('✓ Context manager initialized')

        # Initialize LLM
        self.llm = None
        if enable_llm:
            try:
                self.llm = LLMInterface(
                    model_name=model_name,
                    quantization=quantization,
                    max_tokens=max_tokens,
                    temperature=temperature,
                    logger=self.get_logger()
                )
                self.get_logger().info('✓ LLM loaded successfully')
            except Exception as e:
                self.get_logger().error(f'❌ LLM load failed: {e}')
                self.get_logger().warn('  Continuing without LLM (simple responses)')
                self.llm = None
        else:
            self.get_logger().info('  LLM disabled (config: enable_llm=false)')

        # === INPUTS TO BRAIN ===

        # From perception
        self.perception_sub = self.create_subscription(
            String,
            '/perception/scene_description',
            self._perception_callback,
            10
        )

        # From voice
        self.voice_sub = self.create_subscription(
            String,
            '/voice/command',
            self._voice_callback,
            10
        )

        # From memory (query responses)
        self.memory_response_sub = self.create_subscription(
            String,
            '/memory/query_response',
            self._memory_response_callback,
            10
        )

        # From memory (important events)
        self.memory_event_sub = self.create_subscription(
            String,
            '/memory/event',
            self._memory_event_callback,
            10
        )

        # From audio monitor
        self.audio_event_sub = self.create_subscription(
            String,
            '/audio/event',
            self._audio_event_callback,
            10
        )

        # From perception (anomalies)
        self.anomaly_sub = self.create_subscription(
            String,
            '/perception/anomaly',
            self._anomaly_callback,
            10
        )

        # === OUTPUTS FROM BRAIN ===

        # To voice/TTS
        self.response_pub = self.create_publisher(
            String,
            '/brain/response',
            10
        )

        # To behaviors
        self.command_pub = self.create_publisher(
            String,
            '/brain/command',
            10
        )

        # To memory (queries)
        self.memory_query_pub = self.create_publisher(
            String,
            '/brain/memory_query',
            10
        )

        # System status
        self.status_pub = self.create_publisher(
            String,
            '/brain/status',
            10
        )

        # Timer for periodic status
        self.status_timer = self.create_timer(10.0, self._status_callback)

        self.get_logger().info('='*60)
        self.get_logger().info('✓ BRAIN fully operational')
        self.get_logger().info('='*60)

        self._publish_status('operational')

    # === INPUT CALLBACKS ===

    def _perception_callback(self, msg):
        """Update scene from perception"""
        scene = msg.data
        self.context.update_scene(scene)
        self.get_logger().info(f'👁️  Vision: {scene}', throttle_duration_sec=5.0)

    def _voice_callback(self, msg):
        """Process user command"""
        command = msg.data
        self.get_logger().info(f'🗣️  User: {command}')

        # Process command
        self._process_command(command)

    def _memory_response_callback(self, msg):
        """Receive memory query response"""
        try:
            response = json.loads(msg.data)
            self.get_logger().info(f'💭 Memory response: {response.get("total_results", 0)} results')

            # Add to context
            if response.get('episodic_memory'):
                for mem in response['episodic_memory'][:3]:
                    self.context.add_memory(mem.get('description', ''))
        except:
            self.get_logger().error('Memory response parse error')

    def _memory_event_callback(self, msg):
        """Important event from memory"""
        event = msg.data
        self.get_logger().info(f'💭 Memory event: {event}')
        # Could trigger proactive response

    def _audio_event_callback(self, msg):
        """Audio event from audio monitor"""
        event = msg.data
        self.get_logger().warn(f'🔊 Audio event: {event}')
        # Could alert user or take action

    def _anomaly_callback(self, msg):
        """Anomaly from perception"""
        anomaly = msg.data
        self.get_logger().warn(f'🚨 Anomaly: {anomaly}')

        # Alert user
        response = f"Alert: {anomaly}"
        self._respond(response)

    # === CORE PROCESSING ===

    def _process_command(self, command):
        """
        CENTRAL DECISION MAKING
        Process command with full context and generate response
        """
        self.get_logger().info('🧠 Processing command...')

        # Check if query needs memory search
        if self._needs_memory_search(command):
            self._query_memory(command)

        # Build prompt with full context
        prompt = self.context.build_prompt(command)

        # Generate response
        if self.llm:
            # Use LLM
            response = self.llm.generate(prompt)
        else:
            # Simple fallback (without LLM)
            response = self._simple_response(command)

        # Send response
        self._respond(response)

        # Update conversation history
        self.context.add_turn(command, response)

        # Check if command requires action
        self._check_action_needed(command, response)

    def _needs_memory_search(self, command):
        """Check if command needs memory search"""
        memory_keywords = ['remember', 'recall', 'when', 'where was', 'what happened', 'find']
        return any(keyword in command.lower() for keyword in memory_keywords)

    def _query_memory(self, query):
        """Query memory system"""
        msg = String()
        msg.data = query
        self.memory_query_pub.publish(msg)

        self.get_logger().info(f'  Querying memory: {query[:50]}')

    def _simple_response(self, command):
        """Simple response when LLM not available"""
        command_lower = command.lower()

        if 'see' in command_lower or 'look' in command_lower:
            return f"Currently, {self.context.current_scene}"

        elif 'where' in command_lower or 'location' in command_lower:
            return f"I am at {self.context.current_location}"

        elif 'remember' in command_lower or 'memory' in command_lower:
            if self.context.recent_memories:
                return f"I remember: {self.context.recent_memories[-1]}"
            else:
                return "Let me check my memory"

        else:
            return f"I understand: {command}. Current scene: {self.context.current_scene}"

    def _check_action_needed(self, command, response):
        """Check if command requires robot action"""
        action_keywords = ['move', 'go', 'navigate', 'check', 'patrol', 'follow']

        command_lower = command.lower()
        for keyword in action_keywords:
            if keyword in command_lower:
                # Send command to behaviors
                cmd = String()
                cmd.data = command
                self.command_pub.publish(cmd)

                self.get_logger().info(f'  → Sent to behaviors: {command}')
                break

    def _respond(self, text):
        """Send response to user (via TTS)"""
        msg = String()
        msg.data = text
        self.response_pub.publish(msg)

        self.get_logger().info(f'🧠 Response: {text}')

    def _status_callback(self):
        """Periodic status update"""
        stats = self.context.get_context_stats()
        status_data = {
            'llm_loaded': self.llm is not None,
            'context_stats': stats
        }

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.status_pub.publish(status_msg)

    def _publish_status(self, status):
        """Publish brain status"""
        msg = String()
        msg.data = f'brain:{status}'
        self.status_pub.publish(msg)

    def __del__(self):
        """Cleanup"""
        if self.llm:
            self.llm.unload()


def main(args=None):
    rclpy.init(args=args)
    node = CognitBrainNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Brain shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
