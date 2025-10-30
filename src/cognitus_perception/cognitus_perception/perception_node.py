#!/usr/bin/env python3
"""
COGNITUS Perception Node
Real-time object detection with 3D localization

Subscribes:
    /camera/color/image_raw (sensor_msgs/Image)
    /camera/depth/image_raw (sensor_msgs/Image)

Publishes:
    /perception/objects (cognitus_perception/DetectedObject[])
    /perception/scene_description (std_msgs/String) - to brain
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import numpy as np


class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')

        self.get_logger().info('='*50)
        self.get_logger().info('  👁️  COGNITUS Perception Initializing')
        self.get_logger().info('='*50)

        # Parameters
        self.declare_parameter('detection_fps', 5)
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('use_sim_time', False)
        self.declare_parameter('fx', 554.0)
        self.declare_parameter('fy', 554.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.declare_parameter('min_depth', 0.3)
        self.declare_parameter('max_depth', 10.0)
        self.declare_parameter('depth_scale', 1000.0)

        fps = self.get_parameter('detection_fps').value
        self.conf_threshold = self.get_parameter('confidence_threshold').value

        # Camera intrinsics (from config)
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value
        self.min_depth = self.get_parameter('min_depth').value
        self.max_depth = self.get_parameter('max_depth').value
        self.depth_scale = self.get_parameter('depth_scale').value

        # CV Bridge
        self.bridge = CvBridge()

        # State
        self.rgb_image = None
        self.depth_image = None
        self.model = None
        self.detection_count = 0

        # Load YOLOv8
        self._load_yolo()

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self._rgb_callback,
            10
        )

        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self._depth_callback,
            10
        )

        # Publishers
        self.scene_desc_pub = self.create_publisher(
            String,
            '/perception/scene_description',
            10
        )

        self.objects_pub = self.create_publisher(
            String,
            '/perception/detected_objects',
            10
        )

        # Detection timer
        self.timer = self.create_timer(1.0/fps, self._detection_loop)

        self.get_logger().info(f'✓ Perception ready (FPS: {fps}, Threshold: {self.conf_threshold})')
        self.get_logger().info(f'  Model loaded: {self.model is not None}')

    def _load_yolo(self):
        """Load YOLOv8 model"""
        try:
            from ultralytics import YOLO
            self.model = YOLO('yolov8n.pt')
            self.get_logger().info('✓ YOLOv8n model loaded')
        except ImportError:
            self.get_logger().warn('⚠ ultralytics not installed')
            self.get_logger().info('  Install: pip install ultralytics')
            self.get_logger().info('  Running in NO-DETECTION mode')
        except Exception as e:
            self.get_logger().warn(f'⚠ YOLOv8 load failed: {e}')
            self.get_logger().info('  Will attempt to download on first use')

    def _rgb_callback(self, msg):
        """Store RGB image"""
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        except Exception as e:
            self.get_logger().error(f'RGB conversion error: {e}')

    def _depth_callback(self, msg):
        """Store depth image"""
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, 'passthrough')
        except Exception as e:
            self.get_logger().error(f'Depth conversion error: {e}')

    def _detection_loop(self):
        """Main detection loop"""
        if self.rgb_image is None:
            if self.detection_count == 0:
                self.get_logger().info('Waiting for camera images...')
            self.detection_count += 1
            return

        # Run detection
        detected_objects = []

        if self.model is not None:
            try:
                # YOLOv8 inference
                results = self.model(
                    self.rgb_image,
                    conf=self.conf_threshold,
                    verbose=False
                )

                # Extract detections
                for result in results:
                    boxes = result.boxes
                    for box in boxes:
                        # Get detection info
                        cls_id = int(box.cls[0])
                        label = result.names[cls_id]
                        conf = float(box.conf[0])
                        xyxy = box.xyxy[0].cpu().numpy()

                        # Compute 3D position
                        pos_3d = self._compute_3d_position(xyxy)

                        detected_objects.append({
                            'label': label,
                            'confidence': conf,
                            'position_3d': pos_3d,
                            'bbox': xyxy
                        })

            except Exception as e:
                self.get_logger().error(f'Detection error: {e}')

        # Publish results
        self._publish_results(detected_objects)

        # Log periodically
        self.detection_count += 1
        if self.detection_count % 30 == 0:
            self.get_logger().info(f'Detections: {len(detected_objects)} objects')

    def _compute_3d_position(self, bbox):
        """Compute 3D position from depth"""
        if self.depth_image is None:
            return None

        try:
            # Get bbox center
            x1, y1, x2, y2 = bbox
            cx = int((x1 + x2) / 2)
            cy = int((y1 + y2) / 2)

            # Get depth value
            depth_value = self.depth_image[cy, cx]

            if depth_value == 0 or np.isnan(depth_value):
                return None

            # Convert to meters (using configured scale)
            depth_m = float(depth_value) / self.depth_scale

            # Check depth range
            if depth_m < self.min_depth or depth_m > self.max_depth:
                return None

            # Project to 3D
            x_3d = (cx - self.cx) * depth_m / self.fx
            y_3d = (cy - self.cy) * depth_m / self.fy
            z_3d = depth_m

            return {'x': x_3d, 'y': y_3d, 'z': z_3d}

        except Exception as e:
            self.get_logger().error(f'3D computation error: {e}')
            return None

    def _publish_results(self, objects):
        """Publish detection results"""
        # Natural language description for brain
        if objects:
            labels = [obj['label'] for obj in objects[:5]]
            scene_desc = f"I see: {', '.join(labels)}"

            # Add 3D info if available
            objects_with_pos = [obj for obj in objects if obj['position_3d'] is not None]
            if objects_with_pos:
                nearest = min(objects_with_pos, key=lambda o: o['position_3d']['z'])
                scene_desc += f". Nearest: {nearest['label']} at {nearest['position_3d']['z']:.1f}m"
        else:
            scene_desc = "No objects detected"

        # Publish to brain
        msg = String()
        msg.data = scene_desc
        self.scene_desc_pub.publish(msg)

        # Publish structured objects
        obj_msg = String()
        obj_msg.data = str(objects)
        self.objects_pub.publish(obj_msg)


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Perception shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
