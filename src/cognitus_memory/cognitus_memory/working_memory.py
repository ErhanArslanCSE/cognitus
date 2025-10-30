"""
Working Memory - Short-term storage (10 minutes)
Uses Python deque for efficient FIFO storage
Lightweight, zero dependencies
"""

from collections import deque
from datetime import datetime


class WorkingMemory:
    """
    Short-term memory buffer
    - Stores recent observations (10 min)
    - Fast access, no persistence
    - Auto-evicts old entries
    """

    def __init__(self, max_size=600):
        """
        Args:
            max_size: Maximum entries (default: 600 = 10 min @ 1Hz)
        """
        self.buffer = deque(maxlen=max_size)
        self.max_size = max_size

    def store(self, data):
        """Store new observation"""
        entry = {
            'timestamp': datetime.now().isoformat(),
            'data': data
        }
        self.buffer.append(entry)

    def get_recent(self, count=10):
        """Get N most recent entries"""
        return list(self.buffer)[-count:]

    def get_all(self):
        """Get all entries in working memory"""
        return list(self.buffer)

    def search(self, keyword):
        """Simple keyword search"""
        results = []
        for entry in self.buffer:
            if keyword.lower() in str(entry['data']).lower():
                results.append(entry)
        return results

    def get_size(self):
        """Get current size"""
        return len(self.buffer)

    def clear(self):
        """Clear all entries"""
        self.buffer.clear()

    def get_oldest(self):
        """Get oldest entry"""
        return self.buffer[0] if self.buffer else None

    def get_newest(self):
        """Get newest entry"""
        return self.buffer[-1] if self.buffer else None
