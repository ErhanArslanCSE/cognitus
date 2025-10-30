#!/usr/bin/env python3
"""
COGNITUS Anomaly Detector
Detects unusual objects or scene changes

Subscribes:
    /perception/detected_objects (String)
    /perception/scene_graph (String)

Publishes:
    /perception/anomaly (String) - to brain
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast
from collections import Counter


class AnomalyDetector(Node):
    def __init__(self):
        super().__init__('anomaly_detector')

        self.get_logger().info('='*50)
        self.get_logger().info('  🚨 COGNITUS Anomaly Detector Initializing')
        self.get_logger().info('='*50)

        # Learning state
        self.seen_objects = Counter()  # Object frequency
        self.normal_scene_size = []  # Typical object counts
        self.observation_count = 0
        self.learning_period = 100  # Observations to learn "normal"

        # Subscribers
        self.objects_sub = self.create_subscription(
            String,
            '/perception/detected_objects',
            self._objects_callback,
            10
        )

        # Publishers
        self.anomaly_pub = self.create_publisher(
            String,
            '/perception/anomaly',
            10
        )

        self.get_logger().info('✓ Anomaly detector ready')
        self.get_logger().info(f'  Learning period: {self.learning_period} observations')

    def _objects_callback(self, msg):
        """Analyze objects for anomalies"""
        try:
            objects = ast.literal_eval(msg.data) if msg.data != '[]' else []

            self.observation_count += 1

            # Learning phase
            if self.observation_count <= self.learning_period:
                self._learn_normal(objects)
                if self.observation_count == self.learning_period:
                    self.get_logger().info('✓ Learning complete - anomaly detection active')
                return

            # Detection phase
            anomalies = self._detect_anomalies(objects)

            if anomalies:
                # Alert brain
                alert = f"Anomaly detected: {anomalies}"
                msg = String()
                msg.data = alert
                self.anomaly_pub.publish(msg)

                self.get_logger().warn(f'🚨 {alert}')

        except Exception as e:
            self.get_logger().error(f'Anomaly detection error: {e}')

    def _learn_normal(self, objects):
        """Learn what's normal during learning phase"""
        # Count object frequencies
        for obj in objects:
            self.seen_objects[obj['label']] += 1

        # Track scene complexity
        self.normal_scene_size.append(len(objects))

    def _detect_anomalies(self, objects):
        """Detect if current scene is anomalous"""
        anomalies = []

        # Check for unusual objects (seen rarely)
        for obj in objects:
            label = obj['label']
            frequency = self.seen_objects.get(label, 0)

            # If object seen less than 5% of time, it's unusual
            if frequency < (self.observation_count * 0.05):
                anomalies.append(f"Unusual object: {label}")

        # Check for unusual scene size
        if self.normal_scene_size:
            avg_size = sum(self.normal_scene_size) / len(self.normal_scene_size)
            current_size = len(objects)

            # If 2x larger or smaller than normal
            if current_size > avg_size * 2:
                anomalies.append(f"Too many objects ({current_size} vs avg {avg_size:.1f})")
            elif current_size < avg_size * 0.5 and avg_size > 2:
                anomalies.append(f"Too few objects ({current_size} vs avg {avg_size:.1f})")

        return ", ".join(anomalies) if anomalies else None


def main(args=None):
    rclpy.init(args=args)
    node = AnomalyDetector()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Anomaly detector shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
