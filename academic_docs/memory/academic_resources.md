# cognitus_memory - Academic Resources

Academic papers and research related to hierarchical memory systems, episodic memory, memory compression, and long-term knowledge storage in robotics.

**Last Updated:** October 30, 2025

---

## Hierarchical & Episodic Memory

### 1. Towards General Purpose Robots at Scale: Lifelong Learning and Learning to Use Memory (2025)
- **Authors:** Recent PhD Thesis
- **Publication:** arXiv 2025 (arXiv:2501.10395)
- **Link:** https://arxiv.org/abs/2501.10395
- **Key Focus:** Memory and lifelong learning for long-term robot operation
- **Relevance:** Directly addresses our 3-tier memory architecture

### 2. Lifelong Learning for Home Robots - CoRL 2024 Workshop
- **Event:** CoRL 2024, Munich, Germany (November 9, 2024)
- **Link:** https://llhomerobots.github.io/
- **Topics:** Long-term memory, continual learning, home robotics
- **Relevance:** Exactly our application domain

### 3. Adaptive Compression as a Unifying Framework for Episodic and Semantic Memory (2025)
- **Publication:** Nature Reviews Psychology 2025
- **Link:** https://www.nature.com/articles/s44159-025-00458-6
- **Key Concept:** Memory compression principles from neuroscience
- **Relevance:** Theoretical foundation for our compression strategy

### 4. Rethinking Memory in AI: Taxonomy, Operations, Topics, and Future Directions (2025)
- **Publication:** arXiv 2025
- **Link:** https://arxiv.org/html/2505.00675v1
- **Coverage:** Comprehensive taxonomy of memory systems in AI
- **Relevance:** Framework for understanding memory architectures

### 5. RemembeRL Workshop @ CoRL 2025
- **Event:** CoRL 2025 Workshop
- **Link:** https://rememberl-corl25.github.io/
- **Topics:** Memory-based RL, episodic control, memory architectures
- **Focus:** Memory efficiency and scalability
- **Relevance:** State-of-the-art in memory for robotics

---

## Memory Compression & Efficiency

### 6. MemoryBank: Enhancing Large Language Models with Long-Term Memory (2024)
- **Publication:** ResearchGate 2024
- **Link:** https://www.researchgate.net/publication/379280304_MemoryBank_Enhancing_Large_Language_Models_with_Long-Term_Memory
- **Key Innovation:** Efficient long-term memory for LLMs
- **Technique:** Memory compression and retrieval
- **Relevance:** LLM memory integration strategies

### 7. EpiCache: Episodic KV Cache Compression (2024)
- **Key Innovation:** Clusters conversation history into episodes
- **Technique:** Episode-specific KV cache eviction
- **Benefit:** >90% token usage reduction
- **Relevance:** Compression techniques for episodic data

### 8. Hippocampal-Augmented Memory Integration (HAMI) Framework (2025)
- **Publication:** Scientific Reports 2025
- **Link:** https://www.nature.com/articles/s41598-025-10586-x
- **Key Innovation:** Biologically-inspired memory system
- **Features:** Symbolic indexing, hierarchical refinement, episodic retrieval
- **Relevance:** Neuroscienceinspired memory architecture

---

## Lifelong Learning & Continual Learning

### 9. Long-Term Memory in Cognitive Robots (2024)
- **Publication:** Academia.edu
- **Link:** https://www.academia.edu/9089446/Long_Term_Memory_in_Cognitive_Robots
- **Topics:** Cognitive architectures, memory persistence
- **Relevance:** Long-term memory design for robots

### 10. Evaluating Long-Term Memory for Long-Context Question Answering (2024)
- **Publication:** arXiv 2024
- **Link:** https://arxiv.org/html/2510.23730
- **Focus:** Memory retrieval efficiency
- **Relevance:** Query performance optimization

### 11. Multimodal Long-Term Memory
- **Platform:** Emergent Mind
- **Link:** https://www.emergentmind.com/topics/multimodal-long-term-memory
- **Coverage:** Latest papers on multimodal memory systems
- **Relevance:** Combining visual and textual memories

---

## SQLite & Lightweight Databases

### 12. SQLite for Embedded Systems and Robotics (2024)
- **Platform:** Official SQLite Documentation
- **Link:** https://www.sqlite.org/appfileformat.html
- **Use Case:** Application file format, lightweight storage
- **Relevance:** Our episodic memory implementation choice

---

## Memory Consolidation & Sleep

### 13. Complementary Learning Systems Theory (Neuroscience Foundation)
- **Concept:** Hippocampus (fast episodic) + Cortex (slow semantic)
- **Application:** Our working → episodic → long-term hierarchy
- **Relevance:** Theoretical basis for memory tiers

---

## Retrieval-Augmented Generation (RAG)

### 14. RAG for Robotics: Recent Advances (2024)
- **Trend:** Combining retrieval with generation
- **Application:** Memory-augmented robot responses
- **Relevance:** Future enhancement for memory queries

---

## Vector Databases & Embeddings

### 15. Lightweight Embeddings for Memory Search (2024)
- **Models:** all-MiniLM-L6-v2 (22MB)
- **Use Case:** Semantic similarity search
- **Relevance:** Future enhancement over keyword search

---

## Research Directions

**Active Research Areas:**
1. **Memory Compression:** 10-100x compression with minimal information loss
2. **Episodic Retrieval:** Fast, accurate memory search
3. **Lifelong Learning:** Continuous knowledge accumulation
4. **Memory Consolidation:** Intelligent transfer between tiers
5. **Forgetting Mechanisms:** What to retain vs. discard

**Open Challenges:**
1. Scalability to months/years of operation
2. Multi-modal memory integration
3. Distributed robot memory systems
4. Privacy-preserving memory storage

---

## Workshops & Communities

**CoRL 2025:**
- RemembeRL Workshop
- LEAP (Learning Effective Abstractions for Planning)

**ICRA/IROS:**
- Long-term Autonomy workshops
- Memory-based Robot Learning

---

## Implementation Insights

**From Literature:**
- Hierarchical memory proven effective (neuroscience + AI)
- SQLite sufficient for robot-scale data (millions of entries)
- Importance-based retention outperforms time-based
- Episode boundaries improve retrieval

**Best Practices:**
- Keep working memory small (fast access)
- Index episodic memory aggressively (fast search)
- Compress to summaries (100x achievable)
- Learn importance weights from usage

---

**Curated by:** COGNITUS Team
**Focus Areas:** Hierarchical Memory, Episodic Storage, Memory Compression, Lifelong Learning
**Technology:** SQLite, deque, JSON (lightweight focus)
**Last Literature Review:** October 2025
