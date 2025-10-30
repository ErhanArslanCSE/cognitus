# cognitus_cognition - Academic Resources

Academic papers and research related to LLMs for robotics, small language models, decision making, context management, and embodied AI.

**Last Updated:** October 30, 2025

---

## Small Language Models (SLMs) for Edge/Robotics

### 1. Phi-3 Technical Report: A Highly Capable Language Model Locally on Your Phone (2024)
- **Authors:** Microsoft Research
- **Link:** https://arxiv.org/html/2404.14219v4
- **Model:** Phi-3-mini (3.8B parameters)
- **Key Achievement:** 12 tokens/s on iPhone A16 chip (4-bit)
- **Relevance:** Foundation model for our brain

### 2. Phi-3.5 Small Language Models (2024)
- **Authors:** Microsoft
- **Link:** https://huggingface.co/microsoft/Phi-3.5-mini-instruct
- **Models:** Phi-3.5-mini (3.8B), Phi-3.5-MoE (42B), Phi-3.5-vision (4.2B)
- **Context:** 128K tokens
- **Relevance:** Our recommended LLM backend

### 3. Phi-4: Small Language Models That Pack a Punch (2024)
- **Link:** https://techcommunity.microsoft.com/blog/educatordeveloperblog/phi-4-small-language-models-that-pack-a-punch/4464167
- **Parameters:** 14B (standard), 3.8B (mini)
- **Performance:** Outperforms 5x larger models on math/reasoning
- **Relevance:** Latest Microsoft SLM, future upgrade option

### 4. BitNet b1.58: The 1st 1-Bit LLM (2024)
- **Authors:** Microsoft Research
- **Link:** https://github.com/microsoft/BitNet
- **Parameters:** 2B with 1.58-bit quantization
- **Size:** 0.4GB (smallest ever)
- **Latency:** 29ms CPU inference
- **Relevance:** Ultra-lightweight LLM option

### 5. Demystifying Small Language Models for Edge Deployment (2025)
- **Conference:** ACL 2025
- **Link:** https://aclanthology.org/2025.acl-long.718.pdf
- **Focus:** SLM benchmarks for edge devices
- **Models Evaluated:** Phi-3, Gemma, Qwen, etc.
- **Relevance:** Choosing right model for Jetson

### 6. The Case for Using Small Language Models (2025)
- **Publication:** Harvard Business Review, September 2025
- **Link:** https://hbr.org/2025/09/the-case-for-using-small-language-models
- **Argument:** SLMs sufficient for most tasks, lower cost/latency
- **Relevance:** Justification for Phi-3.5 over larger models

---

## Embodied AI & LLMs for Robotics

### 7. Foundation Models in Robotics: Applications, Challenges, and the Future (2025)
- **Authors:** Roya Firoozi, Mac Schwager, Jiajun Wu et al.
- **Publication:** IJRR 2025
- **Link:** https://journals.sagepub.com/doi/10.1177/02783649241281508
- **Coverage:** Comprehensive survey of foundation models in robotics
- **Topics:** LLMs, VLMs, planning, control
- **Relevance:** State-of-the-art overview

### 8. Embodied AI: From LLMs to World Models (2025)
- **Authors:** Tsinghua University
- **Link:** https://mn.cs.tsinghua.edu.cn/xinwang/PDF/papers/2025_Embodied AI from LLMs to World Models.pdf
- **Coverage:** LLM evolution to embodied agents
- **Relevance:** Theoretical framework for brain module

### 9. Embodied Large Language Models Enable Robots to Complete Complex Tasks (2025)
- **Publication:** Nature Machine Intelligence 2025
- **Link:** https://www.nature.com/articles/s42256-025-01005-x
- **Framework:** ELLMER (Embodied LLM-Enabled Robot)
- **Approach:** GPT-4 + RAG for long-horizon tasks
- **Relevance:** LLM+memory integration strategies

### 10. Embodied AI with Foundation Models (2025)
- **Link:** https://arxiv.org/pdf/2505.20503
- **Coverage:** Foundation models for embodied systems
- **Application:** Robotic perception, planning, control
- **Relevance:** Comprehensive overview

### 11. Embodied AI Agents: Modeling the World (2025)
- **Link:** https://arxiv.org/pdf/2506.22355
- **Date:** July 2025
- **Focus:** Evolution of AI agents with LLMs/VLMs
- **Platforms:** Smart glasses, VR, robots, humanoids
- **Relevance:** Broader context for embodied AI

### 12. Exploring Embodied Multimodal Large Models (2025)
- **Link:** https://arxiv.org/html/2502.15336v1
- **Date:** February 2025
- **Models:** EMLMs integrating vision, language, audio
- **Relevance:** Future multimodal brain enhancement

---

## Context Management & Memory-Augmented LLMs

### 13. MemoryBank: Enhancing LLMs with Long-Term Memory (2024)
- **Link:** https://www.researchgate.net/publication/379280304
- **Key Innovation:** External memory for LLMs
- **Benefit:** Extended context beyond token limits
- **Relevance:** Our context_manager.py design

### 14. Rethinking Memory in AI (2025)
- **Link:** https://arxiv.org/html/2505.00675v1
- **Coverage:** Taxonomy of memory systems
- **Topics:** Working, episodic, semantic memory
- **Relevance:** Theoretical framework for memory integration

---

## LLM Optimization for Edge Devices

### 15. Small and Mighty: NVIDIA Accelerates Microsoft's Phi-3 (2024)
- **Organization:** NVIDIA
- **Link:** https://blogs.nvidia.com/blog/microsoft-open-phi-3-mini-language-models/
- **Focus:** Optimizing Phi-3 for Jetson and edge devices
- **Techniques:** Quantization, TensorRT
- **Relevance:** Deploying Phi-3.5 on Jetson Orin

### 16. ONNX Runtime Supports Phi-3 Models Across Platforms (2024)
- **Link:** https://onnxruntime.ai/blogs/accelerating-phi-3
- **Key Innovation:** Cross-platform Phi-3 deployment
- **Formats:** ONNX for optimization
- **Relevance:** Alternative deployment method

### 17. Phi-4 Quantization and Inference Speedup (2025)
- **Link:** https://techcommunity.microsoft.com/blog/azure-ai-foundry-blog/phi-4-quantization-and-inference-speedup/4360047
- **Techniques:** 4-bit, 8-bit quantization
- **Speedup:** Significant inference improvements
- **Relevance:** Quantization strategies

---

## Generative AI at the Edge

### 18. Generative AI at the Edge: Challenges and Opportunities (2024)
- **Publication:** ACM Queue
- **Link:** https://queue.acm.org/detail.cfm?id=3733702
- **Topics:** SLMs on Jetson, resource constraints, embodied SLMs
- **Key Quote:** "Future embodied SLMs might integrate language, vision, and action in a small footprint"
- **Relevance:** Exactly our use case

---

## LLM for Robot Planning & Control

### 19. GitHub: Awesome-LLM-Robotics
- **Link:** https://github.com/GT-RIPL/Awesome-LLM-Robotics
- **Content:** Comprehensive list of LLM+robotics papers
- **Coverage:** Planning, control, manipulation, navigation
- **Updated:** Continuously through 2024-2025
- **Relevance:** Complete research compendium

### 20. RoboEXP: Action-Conditioned Scene Graph via Interactive Exploration (2024)
- **Conference:** CoRL 2024
- **Key Innovation:** LLM + explicit memory for exploration
- **Relevance:** Memory-augmented LLM for robotics

---

## Workshops & Communities

**CoRL 2025:**
- RemembeRL: Memory-based robot learning
- LEAP: Learning effective abstractions

**ICRA/IROS 2024-2025:**
- Foundation Models for Robotics
- LLM-based Robot Planning

---

## Model Comparison (from literature)

| Model | Params | Size (4-bit) | Latency | Context | Best For |
|-------|--------|--------------|---------|---------|----------|
| BitNet b1.58 | 2B | 0.4GB | 29ms | ~2-4K | Speed |
| Phi-3.5-mini | 3.8B | 2GB | 200ms | 128K | Quality |
| Phi-4-mini | 3.8B | 2GB | 200ms | 4K | Reasoning |
| Gemma-2B | 2B | 1GB | 150ms | 8K | Alternative |

**Recommendation from research:** Phi-3.5-mini for 128K context (best for long conversations)

---

## Prompt Engineering for Robotics

**From Literature:**
- System prompts crucial for robot behavior
- Include capabilities explicitly
- Reference current observations
- Maintain conversation history
- Retrieve relevant memories

**Best Practices:**
- Concise responses (2-3 sentences)
- Spatial language ("on the table")
- Uncertainty expression ("I think...")
- Action-oriented output

---

## Quantization Techniques

**From Papers:**
- 4-bit NF4: Best quality/size trade-off
- 8-bit: Better quality, larger size
- 1-bit (BitNet): Smallest, good quality
- AWQ/GPTQ: Advanced quantization methods

**For Jetson (from NVIDIA):**
- TensorRT-LLM: Optimized inference
- FP16/INT8 mixed precision
- Custom kernels for Jetson

---

## Benchmarks (Jetson-specific, from community)

**Phi-3-mini on Jetson AGX Orin:**
- ~15-20 tokens/second (4-bit)
- ~2.5GB memory usage
- Stable for continuous operation

**Expected on Jetson Orin Nano:**
- ~10-15 tokens/second (4-bit)
- ~2.5GB memory usage
- Sufficient for conversational use

---

## Research Opportunities

**For COGNITUS cognition module:**
1. Multi-step reasoning with chain-of-thought
2. Memory-augmented generation (RAG)
3. Vision-language integration
4. Task decomposition with LLMs
5. Context compression techniques

**Related Papers:**
- Paper 7: Self-Delegating Task Decomposition
- Paper 8: Multi-Persona Behavioral Adaptation
- Paper 3: Abductive Reasoning from Evidence

---

**Curated by:** COGNITUS Team
**Focus Areas:** Small LLMs, Edge Deployment, Embodied AI, Context Management
**Technology:** Phi-3.5-mini / BitNet b1.58
**Last Literature Review:** October 2025
