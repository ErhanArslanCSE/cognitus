#!/usr/bin/env python3
"""
COGNITUS Behavior Controller Node
Simple executor for primitive commands from brain

Receives:
    /brain/command - Primitive commands from brain

Publishes:
    /cmd_vel - Motor control
    /behavior/status - Execution status

Features:
- Executes primitive actions only
- Uses Navigation2 for navigation (LIMO Pro pre-installed)
- Safety monitoring
- Status reporting
- Brain does the intelligence, this just executes
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from std_msgs.msg import String
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import NavigateToPose
import re

from .primitive_actions import PrimitiveActions
from .safety_monitor import SafetyMonitor


class BehaviorControllerNode(Node):
    """
    Behavior Controller - Dumb executor
    Brain is smart, this is simple
    """

    def __init__(self):
        super().__init__('behavior_controller')

        self.get_logger().info('='*50)
        self.get_logger().info('  🎯 COGNITUS Behaviors Initializing')
        self.get_logger().info('='*50)

        # Parameters
        self.declare_parameter('obstacle_threshold', 0.3)
        self.declare_parameter('battery_threshold', 20.0)
        self.declare_parameter('default_speed', 0.2)
        self.declare_parameter('default_angular_speed', 0.5)

        obstacle_thresh = self.get_parameter('obstacle_threshold').value
        battery_thresh = self.get_parameter('battery_threshold').value
        self.default_speed = self.get_parameter('default_speed').value
        self.default_angular_speed = self.get_parameter('default_angular_speed').value

        # Velocity publisher (LIMO Pro's topic)
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # Primitive actions
        self.actions = PrimitiveActions(
            cmd_vel_publisher=self.cmd_vel_pub,
            logger=self.get_logger()
        )
        self.get_logger().info('✓ Primitive actions ready')

        # Safety monitor
        self.safety = SafetyMonitor(logger=self.get_logger())
        self.safety.obstacle_distance = obstacle_thresh
        self.safety.battery_threshold = battery_thresh
        self.get_logger().info('✓ Safety monitor initialized')

        # Navigation2 action client (LIMO Pro pre-installed)
        self.nav_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose'
        )
        self.get_logger().info('✓ Navigation2 client ready')

        # Subscribe to brain commands
        self.command_sub = self.create_subscription(
            String,
            '/brain/command',
            self._command_callback,
            10
        )

        # Subscribe to LIDAR for safety
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self._scan_callback,
            10
        )

        # Publish status
        self.status_pub = self.create_publisher(
            String,
            '/behavior/status',
            10
        )

        # Publish safety status
        self.safety_pub = self.create_publisher(
            String,
            '/behavior/safety',
            10
        )

        # Timer for safety monitoring
        self.safety_timer = self.create_timer(1.0, self._safety_callback)

        self.get_logger().info('='*50)
        self.get_logger().info('✓ Behaviors ready - awaiting commands')
        self.get_logger().info('='*50)

    def _scan_callback(self, msg):
        """Update LIDAR data for safety"""
        self.safety.update_lidar(msg)

    def _command_callback(self, msg):
        """Execute command from brain"""
        command = msg.data
        self.get_logger().info(f'🎯 Command received: {command}')

        # Parse and execute
        self._execute_command(command)

    def _execute_command(self, command):
        """
        Parse and execute primitive command

        Expected formats:
        - "stop"
        - "move_forward [speed]"
        - "rotate [angle]"
        - "navigate_to_pose x y [theta]"
        """
        command = command.strip().lower()

        # Check safety first
        safe, reason = self.safety.is_safe_to_move()

        if not safe and 'stop' not in command:
            self._report_status(f'blocked: {reason}')
            self.get_logger().warn(f'⚠ Command blocked: {reason}')
            return

        try:
            # Parse command
            if command == 'stop':
                self.actions.stop()
                self._report_status('stopped')

            elif command.startswith('move_forward'):
                speed = self._extract_number(command, self.default_speed)
                self.actions.move_forward(speed)
                self._report_status(f'moving forward at {speed}m/s')

            elif command.startswith('move_backward'):
                speed = self._extract_number(command, self.default_speed)
                self.actions.move_backward(speed)
                self._report_status(f'moving backward at {speed}m/s')

            elif command.startswith('rotate_left'):
                speed = self._extract_number(command, self.default_angular_speed)
                self.actions.rotate_left(speed)
                self._report_status(f'rotating left at {speed}rad/s')

            elif command.startswith('rotate_right'):
                speed = self._extract_number(command, self.default_angular_speed)
                self.actions.rotate_right(speed)
                self._report_status(f'rotating right at {speed}rad/s')

            elif command.startswith('rotate'):
                angle = self._extract_number(command, 90.0)
                self.actions.rotate_to_angle(angle)
                self._report_status(f'rotating to {angle}°')

            elif command.startswith('navigate_to_pose'):
                # Parse: navigate_to_pose x y [theta]
                coords = self._extract_coordinates(command)
                if coords:
                    self._navigate_to_pose(coords)
                else:
                    self.get_logger().error('Invalid navigate_to_pose format')
                    self._report_status('error: invalid coordinates')

            else:
                self.get_logger().warn(f'Unknown command: {command}')
                self._report_status(f'unknown command: {command}')

        except Exception as e:
            self.get_logger().error(f'Command execution error: {e}')
            self._report_status(f'error: {str(e)}')

    def _extract_number(self, text, default):
        """Extract first number from text"""
        numbers = re.findall(r'[-+]?\d*\.?\d+', text)
        return float(numbers[0]) if numbers else default

    def _extract_coordinates(self, text):
        """
        Extract coordinates from navigate_to_pose command

        Format: "navigate_to_pose 1.5 2.3 [0.0]"
        Returns: (x, y, theta) or None
        """
        numbers = re.findall(r'[-+]?\d*\.?\d+', text)

        if len(numbers) >= 2:
            x = float(numbers[0])
            y = float(numbers[1])
            theta = float(numbers[2]) if len(numbers) >= 3 else 0.0
            return (x, y, theta)

        return None

    def _navigate_to_pose(self, coords):
        """
        Navigate to pose using Navigation2

        Args:
            coords: (x, y, theta) tuple
        """
        x, y, theta = coords

        # Create goal
        goal_pose = self.actions.create_pose_goal(x, y, theta)

        # Create Navigation2 goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f'  Navigating to: x={x:.2f}, y={y:.2f}, θ={theta:.2f}')

        # Send goal to Navigation2
        self.nav_client.wait_for_server(timeout_sec=2.0)

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self._nav_feedback_callback
        )

        send_goal_future.add_done_callback(self._nav_response_callback)

        self._report_status(f'navigating to ({x:.2f}, {y:.2f})')

    def _nav_feedback_callback(self, feedback_msg):
        """Navigation feedback"""
        # Could log progress, distance remaining, etc.
        pass

    def _nav_response_callback(self, future):
        """Navigation goal response"""
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected')
            self._report_status('navigation rejected')
            return

        self.get_logger().info('Navigation goal accepted')

        # Get result
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._nav_result_callback)

    def _nav_result_callback(self, future):
        """Navigation result"""
        result = future.result().result

        # Check result (Navigation2 specific)
        self.get_logger().info('Navigation completed')
        self._report_status('navigation complete')

    def _safety_callback(self):
        """Periodic safety monitoring"""
        status = self.safety.get_safety_status()

        # Publish safety status
        safety_msg = String()
        safety_msg.data = str(status)
        self.safety_pub.publish(safety_msg)

        # Emergency stop if needed
        if status['emergency_stop']:
            self.actions.stop()

    def _report_status(self, status):
        """Report behavior status"""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Behaviors shutting down')
        # Stop robot on shutdown
        node.actions.stop()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
