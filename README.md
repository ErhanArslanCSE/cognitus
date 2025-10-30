# COGNITUS

**Contextual Object and Goal Navigation with Intelligent Task Understanding and Spatial reasoning**

An autonomous robotics framework for intelligent scene understanding, spatial reasoning, and self-supervised task discovery.

---

## 🚀 Quick Start (Like NextJS!)

### Development (Docker)
```bash
cd cognitus/
make dev
```

### Simulation (Gazebo)
```bash
make sim
```

### Production (LIMO Pro)
```bash
# Copy to robot
scp -r cognitus/ agilex@<robot-ip>:~/

# Run on robot
ssh agilex@<robot-ip>
cd cognitus && make start
```

**Same code works everywhere!**

---

## 🧠 System Architecture

### Central Brain Design

```
COGNITUS BRAIN (cognitus_cognition)
         ↓
    CENTRAL DECISION MAKER
         ↓
    ┌────┴────┬────────┬─────────┐
    ↓         ↓        ↓         ↓
PERCEPTION  MEMORY  VOICE  BEHAVIORS
  (eyes)   (memory) (ears)  (actions)
    ↓         ↓        ↓         ↓
Information flows TO brain
    ↓
Brain makes decisions
    ↓
Commands flow FROM brain
```

### Modules

**🧠 cognitus_cognition** - BRAIN
- Central decision-making
- Coordinates all modules
- Processes information
- Makes decisions

**👁️ cognitus_perception** - VISION
- Object detection (YOLOv8)
- Scene understanding
- Sends "what I see" to brain

**💭 cognitus_memory** - MEMORY
- Stores observations
- Hierarchical compression
- Responds to brain queries

**🗣️ cognitus_voice** - VOICE
- Speech-to-text (Whisper)
- Text-to-speech (Piper)
- User interaction

**🎯 cognitus_behaviors** - ACTIONS
- Executes brain commands
- Robot movement
- Task execution

**🔗 cognitus_integration** - COORDINATOR
- Launches all nodes
- Environment detection
- System coordination

---

## 📦 Complete Package List

```
src/
├── cognitus_perception/      ✅ Vision & object detection
├── cognitus_memory/          ✅ Hierarchical memory
├── cognitus_voice/           ✅ STT & TTS
├── cognitus_behaviors/       ✅ Task execution
├── cognitus_cognition/       ✅ Central brain
└── cognitus_integration/     ✅ Launch coordinator
```

---

## 🔄 Data Flow

```
Camera → Perception → "I see: cup, table" → BRAIN
                                              ↓
User → Voice/STT → "What do you see?" ──────→ BRAIN
                                              ↓
Memory → "Important event" ───────────────→ BRAIN
                                              ↓
                                          BRAIN DECIDES
                                              ↓
                    ┌─────────────────────────┴──────────┐
                    ↓                                    ↓
              "Currently, I see..."              "move forward"
                    ↓                                    ↓
              Voice/TTS → Speaker              Behaviors → Motors
```

---

## 🛠️ Commands

```bash
make dev      # Start development (Docker)
make sim      # Start simulation (Gazebo)
make start    # Start on LIMO Pro
make stop     # Stop Docker
make clean    # Clean artifacts
```

---

## 📁 What's Included

### 7 ROS2 Nodes
1. `brain_node` - Central brain (cognitus_cognition)
2. `perception_node` - Object detection (cognitus_perception)
3. `scene_graph_node` - Scene graph builder (cognitus_perception)
4. `memory_manager_node` - Memory system (cognitus_memory)
5. `stt_node` - Speech-to-text (cognitus_voice)
6. `tts_node` - Text-to-speech (cognitus_voice)
7. `behavior_controller_node` - Task execution (cognitus_behaviors)

### 3 Launch Files
1. `demo.launch.py` - Simple demo
2. `simulation.launch.py` - Gazebo simulation
3. `limo_pro.launch.py` - LIMO Pro hardware

### 5 Config Files
- perception_config.yaml
- memory_config.yaml
- voice_config.yaml
- behavior_config.yaml
- cognition_config.yaml

---

## 📚 Documentation

- **[START.md](START.md)** - Quick start guide
- **[PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)** - Complete overview
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** - System architecture
- **[docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md)** - Implementation guide
- **[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)** - Deployment procedures
- **[academic_docs/](academic_docs/)** - Research papers (9 contributions)

---

## ⚡ Technical Specs

**Hardware:**
- LIMO Pro with Jetson Orin Nano
- RGB-D Camera (Orbbec DaBai)
- 2D LIDAR (EAI T-mini Pro)
- IMU (HI226)

**Software:**
- ROS2 Foxy
- Ubuntu 20.04
- Python 3.8

**AI Models:**
- YOLOv8n (object detection)
- Whisper Tiny (STT)
- Piper TTS (TTS)
- Phi-3-mini (LLM - optional)

---

## 🎯 Key Features

✅ **Works Everywhere** - Same code in Docker and LIMO Pro
✅ **Auto-Detection** - Automatically detects environment
✅ **Central Brain** - All decisions flow through cognitus_cognition
✅ **Modular** - Each component independent and testable
✅ **Production Ready** - Error handling, logging, configuration

---

## 🔬 Research Contributions

This project includes **9 novel research contributions** in embodied AI.

See [academic_docs/research_papers.md](academic_docs/research_papers.md) for details.

Target venues: ICRA, IROS, CoRL, HRI, RSS

---

## 📊 Project Status

✅ **COMPLETE & READY**

- ✅ All 6 packages implemented
- ✅ All 7 nodes functional
- ✅ All 3 launch files ready
- ✅ Docker environment tested
- ✅ LIMO Pro deployment ready
- ✅ Documentation complete

---

## 🤝 Contributing

This is an academic research project. Contributions welcome!

---

## 📄 License

MIT License - See [LICENSE](LICENSE)

---

**COGNITUS** - Teaching robots to understand, learn, and adapt through observation.
