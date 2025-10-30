"""
Safety Monitor - Real-time safety checks
Monitors robot state and prevents dangerous actions
"""

from sensor_msgs.msg import LaserScan
import numpy as np


class SafetyMonitor:
    """
    Safety monitoring system
    - Obstacle detection
    - Battery monitoring
    - Emergency stop
    """

    def __init__(self, logger=None):
        """
        Args:
            logger: ROS logger (optional)
        """
        self.logger = logger

        # Safety state
        self.obstacle_detected = False
        self.battery_low = False
        self.emergency_stop = False

        # Thresholds (configurable)
        self.obstacle_distance = 0.3  # meters
        self.battery_threshold = 20.0  # percent
        self.min_safe_distance = 0.15  # meters (absolute minimum)

        # Latest sensor data
        self.latest_scan = None
        self.battery_percentage = 100.0

    def _log(self, msg, level='info'):
        """Log message"""
        if self.logger:
            if level == 'warn':
                self.logger.warn(msg)
            elif level == 'error':
                self.logger.error(msg)
            else:
                self.logger.info(msg)

    def update_lidar(self, scan_msg):
        """Update LIDAR data"""
        self.latest_scan = scan_msg
        self._check_obstacles()

    def update_battery(self, battery_percent):
        """Update battery level"""
        self.battery_percentage = battery_percent

        # Check battery
        if battery_percent < self.battery_threshold:
            if not self.battery_low:
                self.battery_low = True
                self._log(f'⚠ Battery low: {battery_percent}%', 'warn')
        else:
            self.battery_low = False

    def _check_obstacles(self):
        """Check for obstacles from LIDAR"""
        if self.latest_scan is None:
            return

        # Get minimum distance from LIDAR scan
        ranges = np.array(self.latest_scan.ranges)

        # Filter out inf and 0 values
        valid_ranges = ranges[(ranges > 0) & (ranges < float('inf'))]

        if len(valid_ranges) == 0:
            return

        min_distance = np.min(valid_ranges)

        # Check if obstacle too close
        if min_distance < self.obstacle_distance:
            if not self.obstacle_detected:
                self.obstacle_detected = True
                self._log(f'🚨 Obstacle detected: {min_distance:.2f}m', 'warn')
        else:
            if self.obstacle_detected:
                self.obstacle_detected = False
                self._log('✓ Path clear')

        # Emergency: Very close obstacle
        if min_distance < self.min_safe_distance:
            if not self.emergency_stop:
                self.emergency_stop = True
                self._log(f'🛑 EMERGENCY STOP: Obstacle at {min_distance:.2f}m', 'error')

    def is_safe_to_move(self):
        """
        Check if safe to move

        Returns:
            (bool, str): (safe, reason)
        """
        if self.emergency_stop:
            return False, "Emergency stop active"

        if self.obstacle_detected:
            return False, "Obstacle too close"

        if self.battery_low:
            return False, "Battery low"

        return True, "Safe"

    def is_safe_for_action(self, action_type):
        """
        Check if specific action is safe

        Args:
            action_type: 'move', 'rotate', 'navigate', etc.

        Returns:
            (bool, str): (safe, reason)
        """
        safe, reason = self.is_safe_to_move()

        # Additional checks per action type
        if action_type == 'move' or action_type == 'navigate':
            # Moving requires clear path
            if not safe:
                return False, reason

        elif action_type == 'rotate':
            # Rotation allowed even with obstacle (just slower)
            if self.emergency_stop:
                return False, "Emergency stop active"
            # Battery and other checks still apply

        return True, "Safe"

    def clear_emergency(self):
        """Clear emergency stop (manual reset)"""
        self.emergency_stop = False
        self._log('Emergency stop cleared')

    def get_safety_status(self):
        """Get current safety status"""
        return {
            'obstacle_detected': self.obstacle_detected,
            'battery_low': self.battery_low,
            'emergency_stop': self.emergency_stop,
            'battery_percentage': self.battery_percentage,
            'safe_to_move': self.is_safe_to_move()[0]
        }
