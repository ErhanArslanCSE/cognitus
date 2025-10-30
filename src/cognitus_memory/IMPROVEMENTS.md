# cognitus_memory - Improvement Suggestions

Enhancements for the hierarchical memory system while maintaining lightweight design.

---

## High Priority Improvements

### 1. Semantic Search (Lightweight)

**Current:** Keyword-based SQL LIKE queries

**Improvement:**
- Add simple TF-IDF for better relevance
- Use Python `sklearn.feature_extraction.text.TfidfVectorizer`
- Still lightweight (~10MB)

**Benefits:**
- Better search results
- Relevance ranking
- Synonym matching

**Implementation:**
```python
from sklearn.feature_extraction.text import TfidfVectorizer

# Build index from descriptions
vectorizer = TfidfVectorizer(max_features=1000)
tfidf_matrix = vectorizer.fit_transform(descriptions)

# Query
query_vec = vectorizer.transform([query])
similarities = cosine_similarity(query_vec, tfidf_matrix)
```

**Cost:** ~10MB RAM, minimal CPU

---

### 2. Better Importance Scoring

**Current:** Simple rule-based scoring

**Improvement:**
- Learn importance from user feedback
- Track which memories are accessed/useful
- Adaptive scoring weights

**Algorithm:**
```python
importance = (
    0.3 * novelty_score +
    0.3 * user_query_frequency +
    0.2 * anomaly_presence +
    0.2 * temporal_relevance
)

# Update weights based on which memories are accessed
```

**Benefits:**
- Personalized memory retention
- Better signal-to-noise ratio
- Learns user preferences

---

### 3. Temporal Pattern Mining

**Current:** No pattern extraction

**Improvement:**
- Mine temporal patterns from episodic memory
- Detect recurring events (routines)
- Store in long-term patterns

**Use case:**
```
Pattern detected: "Cup appears on table every morning 8-9 AM"
→ Store in patterns.json
→ Can predict: "User likely wants coffee now"
```

**Implementation:**
- Run nightly on episodic data
- Use simple frequency analysis
- No ML needed (rule-based)

---

### 4. Memory Visualization

**Current:** No visualization

**Improvement:**
- Generate simple HTML reports
- Show memory timeline
- Object frequency charts
- Pattern summaries

**Output:**
```html
memory_db/reports/
├── daily_report_2024-10-30.html
└── weekly_summary.html
```

**Benefits:**
- Debug memory system
- User can review what robot remembers
- Validate importance scoring

---

### 5. Incremental Backup

**Current:** No backup system

**Improvement:**
- Periodic SQLite backup
- Export to portable format
- Cloud sync (optional)

**Implementation:**
```python
import shutil
from datetime import datetime

def backup():
    timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
    shutil.copy(
        'memory_db/episodic.db',
        f'backups/episodic_{timestamp}.db'
    )
```

---

## Medium Priority Improvements

### 6. Compression Optimization

**Current:** Simple JSON compression

**Improvement:**
- Use gzip for JSON files (3-5x better)
- Delta encoding for similar scenes
- Reference deduplication

**Example:**
```python
import gzip
import json

# Save compressed
with gzip.open('daily.json.gz', 'wt') as f:
    json.dump(summary, f)

# Load compressed
with gzip.open('daily.json.gz', 'rt') as f:
    summary = json.load(f)
```

**Benefit:** 3-5x better compression, still lightweight

---

### 7. Memory Defragmentation

**Current:** No defragmentation

**Improvement:**
- Periodic SQLite optimization
- Rebuild indexes
- Remove fragmentation

**Implementation:**
```python
def optimize_db():
    cursor.execute('ANALYZE')  # Update statistics
    cursor.execute('REINDEX')  # Rebuild indexes
    cursor.execute('VACUUM')   # Defragment
```

**Run:** Weekly or when DB > 100MB

---

### 8. Contextual Retrieval

**Current:** Simple keyword matching

**Improvement:**
- Consider temporal context
- Consider spatial context
- Multi-factor ranking

**Example query: "Where were my keys?"**
```python
results = search(
    keyword="keys",
    time_weight=0.4,  # Recent more relevant
    importance_weight=0.3,
    access_weight=0.3
)
```

---

### 9. Memory Summarization

**Current:** Stores raw scene graphs

**Improvement:**
- Summarize similar consecutive scenes
- Detect "no change" periods
- Store deltas only

**Benefit:**
- 10-20x better compression
- Faster queries (less data)
- Still lossless

**Implementation:**
```python
def compress_similar_scenes(scenes):
    # Keep first scene
    # Store only changes for subsequent scenes
    # Reconstruct on query
```

---

### 10. Multi-threaded Queries

**Current:** Single-threaded

**Improvement:**
- Async queries (doesn't block)
- Parallel search across tiers
- Background consolidation

**Implementation:**
```python
import concurrent.futures

def search_parallel(query):
    with concurrent.futures.ThreadPoolExecutor() as executor:
        working_future = executor.submit(working.search, query)
        episodic_future = executor.submit(episodic.search, query)

        return {
            'working': working_future.result(),
            'episodic': episodic_future.result()
        }
```

---

## Low Priority / Future

### 11. Memory Replay

**Current:** Passive storage

**Improvement:**
- "Replay" past observations
- Visualize memory timeline
- Memory-based navigation

**Use case:**
- "Show me what happened at 3 PM"
- Replay events for analysis

---

### 12. Forgetting Mechanism

**Current:** Fixed time-based retention

**Improvement:**
- Smart forgetting (keep important, forget trivial)
- Adaptive retention based on space
- User-marked "never forget"

---

### 13. Memory Synchronization

**Current:** Single robot

**Improvement:**
- Share memories between multiple robots
- Synchronize episodic databases
- Distributed knowledge

---

### 14. Privacy Mode

**Current:** Stores everything

**Improvement:**
- Configurable privacy levels
- Don't store sensitive scenes
- Auto-expire personal data

---

### 15. Memory Statistics Dashboard

**Current:** ROS topics only

**Improvement:**
- Web dashboard
- Real-time memory usage
- Query analytics
- Performance graphs

---

## Research Opportunities

### Paper 1: Hierarchical Event Summarization
- Current: Simple daily summaries
- Enhance: Multi-level narrative generation
- Add: Story formation from events

### Paper 5: Graph-Based Scene Memory
- Current: Full scene storage
- Enhance: Graph differential compression
- Add: Efficient reconstruction

### Paper 2: Adaptive Duty Cycling
- Add: Memory-based quiet period detection
- Reduce storage during inactive times

---

## Implementation Priority

**Immediate (Week 1):**
1. Better importance scoring
2. Temporal pattern mining

**Short-term (Month 1):**
3. Semantic search (TF-IDF)
4. Compression optimization (gzip)
5. Backup system

**Medium-term (Month 2-3):**
6. Memory visualization
7. Contextual retrieval
8. Defragmentation

**Long-term (Month 3+):**
9. Memory replay
10. Multi-threaded queries

---

## Testing Recommendations

### Unit Tests
- Working memory FIFO behavior
- SQLite CRUD operations
- JSON compression
- Importance scoring logic

### Integration Tests
- End-to-end storage and retrieval
- Multi-tier search
- Consolidation process
- Cleanup mechanisms

### Performance Tests
- Storage latency
- Query latency
- Memory usage over time
- Database growth rate

### Stress Tests
- 7-day continuous operation
- 10,000+ episodes
- Concurrent queries
- Disk space limits

---

## Known Limitations

1. **No vector search** - Keyword-based only (by design, for lightweight)
2. **SQLite concurrency** - Limited concurrent writes (sufficient for single robot)
3. **No distributed** - Single-robot only
4. **Simple compression** - JSON is readable but not optimal (trade-off)
5. **No encryption** - Data stored in plaintext
6. **English-only** - Text search assumes English

---

## Optimization Notes

### SQLite Tuning
```sql
PRAGMA journal_mode = WAL;  -- Better concurrency
PRAGMA synchronous = NORMAL;  -- Faster writes
PRAGMA cache_size = 10000;  -- More cache
PRAGMA temp_store = MEMORY;  -- Temp in RAM
```

### JSON Compression
- Use `separators=(',', ':')` for compact JSON
- Consider MessagePack for binary format
- gzip for archival

### Memory Limits
- Monitor with `/memory/stats`
- Alert if > 800MB
- Auto-cleanup if > 900MB
- Emergency purge if > 1GB

---

## Migration Path

### To vector search (if needed later):
```python
# Add simple embedding-based search
from sentence_transformers import SentenceTransformer

model = SentenceTransformer('all-MiniLM-L6-v2')  # 80MB model
# Still lightweight compared to ChromaDB
```

### To distributed (if multiple robots):
```python
# Export/import episodic.db
# Sync via rsync or robot-to-robot
# Merge databases
```

---

**Last updated:** 2024-10-30
**Status:** Complete and production-ready
**Next milestone:** Pattern mining implementation
