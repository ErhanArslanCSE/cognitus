"""
Primitive Actions - Basic robot movements
Low-level actions that behaviors can execute
"""

from geometry_msgs.msg import Twist, PoseStamped
import math


class PrimitiveActions:
    """
    Primitive action executor
    Provides simple movement commands
    """

    def __init__(self, cmd_vel_publisher, logger=None):
        """
        Args:
            cmd_vel_publisher: ROS2 publisher for /cmd_vel
            logger: ROS logger (optional)
        """
        self.cmd_vel_pub = cmd_vel_publisher
        self.logger = logger

    def _log(self, msg):
        """Log message"""
        if self.logger:
            self.logger.info(msg)

    def stop(self):
        """Stop all movement"""
        twist = Twist()
        # All zeros = stop
        self.cmd_vel_pub.publish(twist)
        self._log('  Action: STOP')

    def move_forward(self, speed=0.2, duration=None):
        """
        Move forward at given speed

        Args:
            speed: Linear velocity (m/s)
            duration: Time to move (seconds), None = continuous
        """
        twist = Twist()
        twist.linear.x = speed
        self.cmd_vel_pub.publish(twist)
        self._log(f'  Action: MOVE_FORWARD (speed={speed})')

    def move_backward(self, speed=0.2):
        """Move backward"""
        twist = Twist()
        twist.linear.x = -speed
        self.cmd_vel_pub.publish(twist)
        self._log(f'  Action: MOVE_BACKWARD (speed={speed})')

    def rotate_left(self, angular_speed=0.5):
        """Rotate counter-clockwise"""
        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)
        self._log(f'  Action: ROTATE_LEFT (speed={angular_speed})')

    def rotate_right(self, angular_speed=0.5):
        """Rotate clockwise"""
        twist = Twist()
        twist.angular.z = -angular_speed
        self.cmd_vel_pub.publish(twist)
        self._log(f'  Action: ROTATE_RIGHT (speed={angular_speed})')

    def rotate_to_angle(self, target_angle_degrees):
        """
        Rotate to specific angle

        Args:
            target_angle_degrees: Target angle in degrees

        Note: Requires IMU feedback for precise control
        This is a simplified version
        """
        # Convert to radians
        angular_speed = 0.5 if target_angle_degrees > 0 else -0.5

        twist = Twist()
        twist.angular.z = angular_speed
        self.cmd_vel_pub.publish(twist)

        self._log(f'  Action: ROTATE_TO_ANGLE ({target_angle_degrees}°)')

    def strafe_left(self, speed=0.2):
        """
        Strafe left (mecanum mode only)
        LIMO Pro supports mecanum wheels
        """
        twist = Twist()
        twist.linear.y = speed  # Lateral movement
        self.cmd_vel_pub.publish(twist)
        self._log(f'  Action: STRAFE_LEFT (speed={speed})')

    def strafe_right(self, speed=0.2):
        """Strafe right (mecanum mode)"""
        twist = Twist()
        twist.linear.y = -speed
        self.cmd_vel_pub.publish(twist)
        self._log(f'  Action: STRAFE_RIGHT (speed={speed})')

    def create_pose_goal(self, x, y, theta=0.0, frame_id='map'):
        """
        Create PoseStamped for Navigation2

        Args:
            x, y: Target position (meters)
            theta: Target orientation (radians)
            frame_id: Reference frame

        Returns:
            PoseStamped message
        """
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.logger.get_clock().now().to_msg() if self.logger else None

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0

        # Convert theta to quaternion (simplified, only yaw)
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)

        return pose
