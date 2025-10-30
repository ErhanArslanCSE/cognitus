#!/usr/bin/env python3
"""
COGNITUS Scene Graph Builder
Builds spatial scene graph from detected objects

Subscribes:
    /perception/detected_objects (String)

Publishes:
    /perception/scene_graph (String) - to memory
    /perception/spatial_relations (String) - to brain
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import ast


class SceneGraphBuilder(Node):
    def __init__(self):
        super().__init__('scene_graph_builder')

        self.get_logger().info('='*50)
        self.get_logger().info('  🗺️  COGNITUS Scene Graph Initializing')
        self.get_logger().info('='*50)

        # Parameters
        self.declare_parameter('spatial_threshold', 0.5)  # meters

        self.spatial_threshold = self.get_parameter('spatial_threshold').value

        # State
        self.current_objects = []

        # Subscribers
        self.objects_sub = self.create_subscription(
            String,
            '/perception/detected_objects',
            self._objects_callback,
            10
        )

        # Publishers
        self.scene_graph_pub = self.create_publisher(
            String,
            '/perception/scene_graph',
            10
        )

        self.relations_pub = self.create_publisher(
            String,
            '/perception/spatial_relations',
            10
        )

        self.get_logger().info(f'✓ Scene Graph ready (threshold: {self.spatial_threshold}m)')

    def _objects_callback(self, msg):
        """Process detected objects and build scene graph"""
        try:
            # Parse objects (from string representation)
            self.current_objects = ast.literal_eval(msg.data) if msg.data != '[]' else []

            if not self.current_objects:
                return

            # Compute spatial relations
            relations = self._compute_spatial_relations()

            # Build scene graph
            scene_graph = {
                'timestamp': self.get_clock().now().seconds_nanoseconds()[0],
                'objects': self.current_objects,
                'relations': relations,
                'total_objects': len(self.current_objects)
            }

            # Publish scene graph to memory
            graph_msg = String()
            graph_msg.data = str(scene_graph)
            self.scene_graph_pub.publish(graph_msg)

            # Publish natural language relations to brain
            if relations:
                rel_desc = self._describe_relations(relations)
                rel_msg = String()
                rel_msg.data = rel_desc
                self.relations_pub.publish(rel_msg)

        except Exception as e:
            self.get_logger().error(f'Scene graph error: {e}')

    def _compute_spatial_relations(self):
        """Compute spatial relations between objects"""
        relations = []

        # Get objects with 3D positions
        objects_3d = [obj for obj in self.current_objects if obj.get('position_3d')]

        # Compare each pair
        for i, obj1 in enumerate(objects_3d):
            for obj2 in objects_3d[i+1:]:
                relation = self._get_relation(obj1, obj2)
                if relation:
                    relations.append(relation)

        return relations

    def _get_relation(self, obj1, obj2):
        """Determine spatial relation between two objects"""
        pos1 = obj1['position_3d']
        pos2 = obj2['position_3d']

        # Compute distance
        dx = pos2['x'] - pos1['x']
        dy = pos2['y'] - pos1['y']
        dz = pos2['z'] - pos1['z']

        distance = (dx**2 + dy**2 + dz**2)**0.5

        # Determine relation type
        relation_type = None

        # Vertical relations
        if abs(dy) > 0.2:  # 20cm vertical difference
            if dy > 0:
                relation_type = 'above'
            else:
                relation_type = 'below'

        # Horizontal relations
        elif abs(dx) > 0.1:  # 10cm horizontal difference
            if dx > 0:
                relation_type = 'left_of'
            else:
                relation_type = 'right_of'

        # Near relation
        elif distance < self.spatial_threshold:
            relation_type = 'near'

        if relation_type:
            return {
                'source': obj1['label'],
                'target': obj2['label'],
                'type': relation_type,
                'distance': distance,
                'confidence': min(obj1['confidence'], obj2['confidence'])
            }

        return None

    def _describe_relations(self, relations):
        """Create natural language description of relations"""
        if not relations:
            return "No spatial relations detected"

        # Take most confident relations
        top_relations = sorted(relations, key=lambda r: r['confidence'], reverse=True)[:3]

        descriptions = []
        for rel in top_relations:
            desc = f"{rel['source']} is {rel['type']} {rel['target']}"
            descriptions.append(desc)

        return "; ".join(descriptions)


def main(args=None):
    rclpy.init(args=args)
    node = SceneGraphBuilder()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Scene graph shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
