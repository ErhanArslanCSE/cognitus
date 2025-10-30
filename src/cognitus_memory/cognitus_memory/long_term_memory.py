"""
Long-term Memory - Compressed storage (weeks/months)
Uses JSON files for extreme compression
Lightweight, human-readable
"""

import json
import os
from datetime import datetime, timedelta
from collections import defaultdict


class LongTermMemory:
    """
    Long-term compressed memory
    - Daily summaries
    - Pattern storage
    - JSON file-based
    - Human-readable
    """

    def __init__(self, storage_dir='memory_db/long_term'):
        """
        Args:
            storage_dir: Directory for JSON files
        """
        self.storage_dir = storage_dir
        os.makedirs(storage_dir, exist_ok=True)

        self.daily_summaries = {}
        self.learned_patterns = {}

    def store_daily_summary(self, date, summary_data):
        """Store compressed daily summary"""
        file_path = os.path.join(self.storage_dir, f'daily_{date}.json')

        summary = {
            'date': date,
            'timestamp': datetime.now().isoformat(),
            'total_events': summary_data.get('total_events', 0),
            'key_events': summary_data.get('key_events', []),
            'objects_seen': summary_data.get('objects_seen', {}),
            'anomalies': summary_data.get('anomalies', []),
            'summary': summary_data.get('summary', '')
        }

        with open(file_path, 'w') as f:
            json.dump(summary, f, indent=2)

        self.daily_summaries[date] = summary

    def get_daily_summary(self, date):
        """Retrieve daily summary"""
        if date in self.daily_summaries:
            return self.daily_summaries[date]

        # Load from file
        file_path = os.path.join(self.storage_dir, f'daily_{date}.json')
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                summary = json.load(f)
                self.daily_summaries[date] = summary
                return summary

        return None

    def get_recent_summaries(self, days=7):
        """Get summaries for last N days"""
        summaries = []
        for i in range(days):
            date = (datetime.now() - timedelta(days=i)).strftime('%Y-%m-%d')
            summary = self.get_daily_summary(date)
            if summary:
                summaries.append(summary)

        return summaries

    def store_pattern(self, pattern_name, pattern_data):
        """Store learned pattern"""
        file_path = os.path.join(self.storage_dir, 'patterns.json')

        # Load existing patterns
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                patterns = json.load(f)
        else:
            patterns = {}

        # Add new pattern
        patterns[pattern_name] = {
            'data': pattern_data,
            'learned_at': datetime.now().isoformat(),
            'confidence': pattern_data.get('confidence', 0.5)
        }

        # Save
        with open(file_path, 'w') as f:
            json.dump(patterns, f, indent=2)

        self.learned_patterns = patterns

    def get_pattern(self, pattern_name):
        """Retrieve learned pattern"""
        if pattern_name in self.learned_patterns:
            return self.learned_patterns[pattern_name]

        # Load from file
        file_path = os.path.join(self.storage_dir, 'patterns.json')
        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                patterns = json.load(f)
                self.learned_patterns = patterns
                return patterns.get(pattern_name)

        return None

    def get_all_patterns(self):
        """Get all learned patterns"""
        file_path = os.path.join(self.storage_dir, 'patterns.json')

        if os.path.exists(file_path):
            with open(file_path, 'r') as f:
                self.learned_patterns = json.load(f)

        return self.learned_patterns

    def compress_episodes(self, episodes):
        """Compress episodes into daily summary"""
        if not episodes:
            return None

        # Group by date
        by_date = defaultdict(list)
        for ep in episodes:
            date = ep['timestamp'][:10]  # YYYY-MM-DD
            by_date[date].append(ep)

        # Create summaries
        for date, day_episodes in by_date.items():
            # Count objects
            objects_seen = defaultdict(int)
            anomalies = []

            for ep in day_episodes:
                # Parse data
                data_str = ep.get('data', '{}')
                try:
                    data = json.loads(data_str) if isinstance(data_str, str) else data_str

                    # Count objects
                    for obj in data.get('objects', []):
                        objects_seen[obj.get('label', 'unknown')] += 1

                    # Collect anomalies
                    if 'anomaly' in data:
                        anomalies.append(data['anomaly'])
                except:
                    pass

            # Create summary
            summary = {
                'total_events': len(day_episodes),
                'objects_seen': dict(objects_seen),
                'anomalies': anomalies,
                'key_events': [ep['description'] for ep in day_episodes[:10]],
                'summary': f"{len(day_episodes)} events observed"
            }

            self.store_daily_summary(date, summary)

    def get_storage_size(self):
        """Get total storage size in MB"""
        total_size = 0
        for root, dirs, files in os.walk(self.storage_dir):
            for f in files:
                fp = os.path.join(root, f)
                total_size += os.path.getsize(fp)

        return total_size / (1024 * 1024)  # MB
