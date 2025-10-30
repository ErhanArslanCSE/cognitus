"""
Episodic Memory - Medium-term storage (7 days)
Uses SQLite for lightweight persistent storage
Built-in to Python, zero dependencies
"""

import sqlite3
import json
from datetime import datetime, timedelta
import os


class EpisodicMemory:
    """
    Episodic memory using SQLite
    - Persistent storage
    - Fast SQL queries
    - Lightweight (~MB range)
    - Zero extra dependencies
    """

    def __init__(self, db_path='memory_db/episodic.db', retention_days=7):
        self.db_path = db_path
        self.retention_days = retention_days

        # Create directory
        os.makedirs(os.path.dirname(db_path), exist_ok=True)

        # Connect to SQLite
        self.conn = sqlite3.connect(db_path, check_same_thread=False)
        self.conn.row_factory = sqlite3.Row

        # Initialize
        self._create_tables()

    def _create_tables(self):
        """Create database schema"""
        cursor = self.conn.cursor()

        # Main episodes table
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS episodes (
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                timestamp TEXT NOT NULL,
                description TEXT NOT NULL,
                data TEXT,
                importance REAL DEFAULT 0.5,
                access_count INTEGER DEFAULT 0,
                tags TEXT,
                created_at TEXT DEFAULT CURRENT_TIMESTAMP
            )
        ''')

        # Indexes for performance
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_timestamp ON episodes(timestamp)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_importance ON episodes(importance)')
        cursor.execute('CREATE INDEX IF NOT EXISTS idx_description ON episodes(description)')

        self.conn.commit()

    def store(self, description, data, importance=0.5, tags=None):
        """
        Store new episode

        Args:
            description: Natural language description
            data: Episode data (dict)
            importance: Importance score (0.0-1.0)
            tags: Optional tags (list)

        Returns:
            episode_id: ID of stored episode
        """
        cursor = self.conn.cursor()

        cursor.execute('''
            INSERT INTO episodes (timestamp, description, data, importance, tags)
            VALUES (?, ?, ?, ?, ?)
        ''', (
            datetime.now().isoformat(),
            description,
            json.dumps(data),
            importance,
            json.dumps(tags) if tags else None
        ))

        self.conn.commit()
        return cursor.lastrowid

    def search(self, keyword, limit=10):
        """Search episodes by keyword"""
        cursor = self.conn.cursor()

        cursor.execute('''
            SELECT * FROM episodes
            WHERE description LIKE ? OR data LIKE ?
            ORDER BY importance DESC, timestamp DESC
            LIMIT ?
        ''', (f'%{keyword}%', f'%{keyword}%', limit))

        results = []
        for row in cursor.fetchall():
            episode = self._row_to_dict(row)
            results.append(episode)

            # Update access count
            cursor.execute('UPDATE episodes SET access_count = access_count + 1 WHERE id = ?', (row['id'],))

        self.conn.commit()
        return results

    def get_recent(self, hours=24, limit=100):
        """Get recent episodes"""
        cursor = self.conn.cursor()
        cutoff = (datetime.now() - timedelta(hours=hours)).isoformat()

        cursor.execute('''
            SELECT * FROM episodes
            WHERE timestamp > ?
            ORDER BY timestamp DESC
            LIMIT ?
        ''', (cutoff, limit))

        return [self._row_to_dict(row) for row in cursor.fetchall()]

    def get_important(self, threshold=0.7, limit=50):
        """Get important episodes"""
        cursor = self.conn.cursor()

        cursor.execute('''
            SELECT * FROM episodes
            WHERE importance >= ?
            ORDER BY importance DESC, timestamp DESC
            LIMIT ?
        ''', (threshold, limit))

        return [self._row_to_dict(row) for row in cursor.fetchall()]

    def cleanup_old(self):
        """Remove episodes older than retention period"""
        cursor = self.conn.cursor()
        cutoff = (datetime.now() - timedelta(days=self.retention_days)).isoformat()

        cursor.execute('DELETE FROM episodes WHERE timestamp < ?', (cutoff,))
        deleted = cursor.rowcount

        # Vacuum to reclaim space
        cursor.execute('VACUUM')

        self.conn.commit()
        return deleted

    def get_stats(self):
        """Get memory statistics"""
        cursor = self.conn.cursor()

        cursor.execute('SELECT COUNT(*) as total FROM episodes')
        total = cursor.fetchone()['total']

        cursor.execute('SELECT AVG(importance) as avg_imp FROM episodes')
        avg_importance = cursor.fetchone()['avg_imp'] or 0.0

        cursor.execute('SELECT COUNT(*) as important FROM episodes WHERE importance > 0.7')
        important_count = cursor.fetchone()['important']

        # File size
        db_size_mb = os.path.getsize(self.db_path) / (1024 * 1024) if os.path.exists(self.db_path) else 0

        return {
            'total_episodes': total,
            'important_episodes': important_count,
            'avg_importance': round(avg_importance, 3),
            'db_size_mb': round(db_size_mb, 2)
        }

    def _row_to_dict(self, row):
        """Convert SQLite row to dict"""
        return {
            'id': row['id'],
            'timestamp': row['timestamp'],
            'description': row['description'],
            'data': json.loads(row['data']) if row['data'] else {},
            'importance': row['importance'],
            'access_count': row['access_count'],
            'tags': json.loads(row['tags']) if row['tags'] else []
        }

    def close(self):
        """Close database connection"""
        self.conn.close()
