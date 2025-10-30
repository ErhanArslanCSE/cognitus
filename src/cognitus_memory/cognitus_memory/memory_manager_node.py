#!/usr/bin/env python3
"""
COGNITUS Memory Manager Node
3-tier hierarchical memory system:
  - Working memory (deque, 10 min)
  - Episodic memory (SQLite, 7 days)
  - Long-term memory (JSON, compressed)

Subscribes:
    /perception/scene_graph - Scene updates
    /brain/memory_query - Query requests

Publishes:
    /memory/event - Important events to brain
    /memory/query_response - Query results to brain
    /memory/stats - Memory statistics
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json

from .working_memory import WorkingMemory
from .episodic_memory import EpisodicMemory
from .long_term_memory import LongTermMemory


class MemoryManagerNode(Node):
    """
    Memory Manager - 3-tier hierarchical memory
    """

    def __init__(self):
        super().__init__('memory_manager_node')

        self.get_logger().info('='*50)
        self.get_logger().info('  💭 COGNITUS Memory Initializing')
        self.get_logger().info('='*50)

        # Parameters
        self.declare_parameter('working_memory_size', 600)
        self.declare_parameter('episodic_retention_days', 7)
        self.declare_parameter('episodic_db_path', 'memory_db/episodic.db')
        self.declare_parameter('long_term_dir', 'memory_db/long_term')
        self.declare_parameter('consolidation_interval', 60.0)
        self.declare_parameter('importance_threshold_episodic', 0.6)
        self.declare_parameter('importance_threshold_alert', 0.8)

        working_size = self.get_parameter('working_memory_size').value
        retention_days = self.get_parameter('episodic_retention_days').value
        db_path = self.get_parameter('episodic_db_path').value
        lt_dir = self.get_parameter('long_term_dir').value
        consolidation_interval = self.get_parameter('consolidation_interval').value
        self.threshold_episodic = self.get_parameter('importance_threshold_episodic').value
        self.threshold_alert = self.get_parameter('importance_threshold_alert').value

        # Initialize 3-tier memory
        self.get_logger().info('Initializing memory tiers...')

        self.working = WorkingMemory(max_size=working_size)
        self.get_logger().info(f'  ✓ Working memory: {working_size} entries')

        self.episodic = EpisodicMemory(
            db_path=db_path,
            retention_days=retention_days
        )
        self.get_logger().info(f'  ✓ Episodic memory: SQLite ({retention_days} days)')

        self.long_term = LongTermMemory(storage_dir=lt_dir)
        self.get_logger().info('  ✓ Long-term memory: JSON compressed')

        # Subscribe to scene updates
        self.scene_sub = self.create_subscription(
            String,
            '/perception/scene_graph',
            self._scene_callback,
            10
        )

        # Subscribe to memory queries from brain
        self.query_sub = self.create_subscription(
            String,
            '/brain/memory_query',
            self._query_callback,
            10
        )

        # Publish important events to brain
        self.event_pub = self.create_publisher(
            String,
            '/memory/event',
            10
        )

        # Publish query responses to brain
        self.response_pub = self.create_publisher(
            String,
            '/memory/query_response',
            10
        )

        # Publish memory statistics
        self.stats_pub = self.create_publisher(
            String,
            '/memory/stats',
            10
        )

        # Timer for consolidation
        self.consolidation_timer = self.create_timer(
            consolidation_interval,
            self._consolidation_callback
        )

        # Timer for statistics
        self.stats_timer = self.create_timer(30.0, self._stats_callback)

        self.get_logger().info('✓ Memory Manager ready')
        self.get_logger().info('='*50)

    def _scene_callback(self, msg):
        """Store scene update in working memory"""
        try:
            # Parse scene data safely
            import ast
            scene_data = ast.literal_eval(msg.data)  # Safe parsing

            # Store in working memory
            self.working.store(scene_data)

            # Check importance
            importance = self._calculate_importance(scene_data)

            # Promote to episodic if important
            if importance > self.threshold_episodic:
                description = scene_data.get('description', 'Scene update')

                episode_id = self.episodic.store(
                    description=description,
                    data=scene_data,
                    importance=importance
                )

                # Notify brain if very important
                if importance > self.threshold_alert:
                    event_msg = String()
                    event_msg.data = f"Important: {description}"
                    self.event_pub.publish(event_msg)

                    self.get_logger().info(f'🔔 Important event stored (id={episode_id}, imp={importance:.2f})')

        except Exception as e:
            self.get_logger().error(f'Scene storage error: {e}')

    def _calculate_importance(self, scene_data):
        """
        Calculate importance score for scene

        Factors:
        - Number of objects (novelty)
        - Anomalies present
        - Spatial relations complexity
        """
        importance = 0.5  # Base importance

        # Check for anomalies
        if 'anomaly' in str(scene_data):
            importance += 0.3

        # Check for many objects
        objects = scene_data.get('objects', [])
        if len(objects) > 5:
            importance += 0.1

        # Check for relations
        relations = scene_data.get('relations', [])
        if len(relations) > 3:
            importance += 0.1

        return min(importance, 1.0)

    def _query_callback(self, msg):
        """Handle memory query from brain"""
        query = msg.data
        self.get_logger().info(f'🔍 Memory query: {query}')

        try:
            results = self._search_all_tiers(query)

            # Send response to brain
            response = {
                'query': query,
                'working_memory': results['working'],
                'episodic_memory': results['episodic'],
                'total_results': len(results['working']) + len(results['episodic'])
            }

            response_msg = String()
            response_msg.data = json.dumps(response)
            self.response_pub.publish(response_msg)

            self.get_logger().info(f'  Found {response["total_results"]} results')

        except Exception as e:
            self.get_logger().error(f'Query error: {e}')

    def _search_all_tiers(self, query):
        """Search across all memory tiers"""
        # Search working memory
        working_results = self.working.search(query)

        # Search episodic memory
        episodic_results = self.episodic.search(query, limit=10)

        # TODO: Search long-term patterns

        return {
            'working': working_results[-5:] if working_results else [],
            'episodic': episodic_results
        }

    def _consolidation_callback(self):
        """
        Periodic memory consolidation
        - Promote important working → episodic
        - Compress old episodic → long-term
        - Clean up old data
        """
        try:
            # Get old working memory entries
            oldest = self.working.get_oldest()
            if oldest:
                # Could promote to episodic if important
                pass

            # Clean old episodic entries
            deleted = self.episodic.cleanup_old()
            if deleted > 0:
                self.get_logger().info(f'Cleaned {deleted} old episodes')

            # Compress episodic to long-term (daily)
            recent_episodes = self.episodic.get_recent(hours=24*2, limit=1000)  # Last 2 days
            if len(recent_episodes) > 100:  # If enough data
                self.long_term.compress_episodes(recent_episodes)

        except Exception as e:
            self.get_logger().error(f'Consolidation error: {e}')

    def _stats_callback(self):
        """Publish memory statistics"""
        try:
            working_size = self.working.get_size()
            episodic_stats = self.episodic.get_stats()
            long_term_size = self.long_term.get_storage_size()

            stats = {
                'working_memory': {
                    'size': working_size,
                    'max': self.working.max_size
                },
                'episodic_memory': episodic_stats,
                'long_term_memory': {
                    'size_mb': round(long_term_size, 2)
                }
            }

            stats_msg = String()
            stats_msg.data = json.dumps(stats)
            self.stats_pub.publish(stats_msg)

            # Log summary occasionally
            self.get_logger().info(
                f'Memory: Working={working_size}, '
                f'Episodic={episodic_stats["total_episodes"]}, '
                f'LT={long_term_size:.1f}MB',
                throttle_duration_sec=60.0
            )

        except Exception as e:
            self.get_logger().error(f'Stats error: {e}')

    def __del__(self):
        """Cleanup on shutdown"""
        if hasattr(self, 'episodic'):
            self.episodic.close()


def main(args=None):
    rclpy.init(args=args)
    node = MemoryManagerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Memory shutting down')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
