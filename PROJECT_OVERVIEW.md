# COGNITUS - Complete Project Overview

**Contextual Object and Goal Navigation with Intelligent Task Understanding and Spatial reasoning**

---

## 🎯 Project Summary

COGNITUS is an embodied AI system for autonomous robotics that combines perception, memory, reasoning, and natural interaction. The system runs on LIMO Pro robot with Jetson Orin Nano and is designed for scene understanding, anomaly detection, and self-supervised task discovery.

---

## 📦 System Architecture

### Core Components

**1. cognitus_cognition (🧠 BRAIN)**
- Central decision-making system
- Coordinates all modules
- LLM-based reasoning (Phi-3-mini)
- Command generation

**2. cognitus_perception (👁️ VISION)**
- Object detection (YOLOv8)
- Scene graph construction
- 3D localization from depth
- Spatial relation extraction

**3. cognitus_memory (💭 MEMORY)**
- Working memory (10 min)
- Episodic memory (7 days)
- Long-term compression (100x)
- Semantic retrieval

**4. cognitus_voice (🗣️ VOICE)**
- Speech-to-text (Whisper Tiny)
- Text-to-speech (Piper)
- Audio capture and playback
- Wake word detection

**5. cognitus_behaviors (🎯 ACTIONS)**
- Task decomposition
- Navigation control
- Routine execution
- Proactive behaviors

**6. cognitus_integration (🔗 COORDINATOR)**
- System launch management
- Environment detection
- Node coordination
- Health monitoring

---

## 🔄 Data Flow

```
Camera/LIDAR/Audio
        ↓
   PERCEPTION ────→ Scene Description
        ↓                    ↓
     MEMORY              BRAIN (Central)
        ↑                    ↓
   Scene Storage      ┌─────┴─────┐
                      ↓           ↓
                  VOICE      BEHAVIORS
                    ↓             ↓
                 Speaker      Motors
```

### Topic Communication

**Inputs to Brain:**
- `/perception/scene_description` - What the robot sees
- `/voice/command` - Voice commands
- `/memory/event` - Important memory events

**Outputs from Brain:**
- `/brain/command` - Commands to behaviors
- `/brain/response` - Responses to user
- `/brain/status` - System status

---

## 🚀 Deployment Modes

### Development (Docker)
```bash
make dev    # Interactive development
make sim    # Gazebo simulation
```

**Features:**
- ROS2 Foxy environment
- Gazebo simulation
- Live code editing
- No hardware needed

### Production (LIMO Pro)
```bash
make start  # Auto-detects and launches
```

**Features:**
- Real hardware integration
- Pre-installed LIMO drivers
- Same codebase as Docker
- Production-ready

---

## 📁 Directory Structure

```
cognitus/
├── src/
│   ├── cognitus_perception/
│   │   ├── cognitus_perception/
│   │   │   ├── perception_node.py
│   │   │   └── scene_graph_node.py
│   │   ├── config/perception_config.yaml
│   │   ├── msg/
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── cognitus_cognition/
│   │   ├── cognitus_cognition/
│   │   │   └── brain_node.py
│   │   ├── config/cognition_config.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── cognitus_memory/
│   │   ├── cognitus_memory/
│   │   │   └── memory_manager_node.py
│   │   ├── config/memory_config.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── cognitus_voice/
│   │   ├── cognitus_voice/
│   │   │   ├── stt_node.py
│   │   │   ├── tts_node.py
│   │   │   └── audio_capture_node.py
│   │   ├── config/voice_config.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   ├── cognitus_behaviors/
│   │   ├── cognitus_behaviors/
│   │   │   └── behavior_controller_node.py
│   │   ├── config/behavior_config.yaml
│   │   ├── package.xml
│   │   └── setup.py
│   │
│   └── cognitus_integration/
│       ├── launch/
│       │   ├── demo.launch.py
│       │   ├── simulation.launch.py
│       │   └── limo_pro.launch.py
│       ├── CMakeLists.txt
│       └── package.xml
│
├── docs/                    # Technical documentation
├── academic_docs/           # Research papers
├── config/                  # Global configuration
├── models/                  # AI models
├── scripts/                 # Setup scripts
├── start.sh                 # Main startup
├── Dockerfile
├── docker-compose.yml
├── Makefile
└── requirements.txt
```

---

## 🔧 Configuration Files

All modules have YAML configuration:

- `config/cognitus_config.yaml` - Global settings
- `src/cognitus_perception/config/perception_config.yaml`
- `src/cognitus_cognition/config/cognition_config.yaml`
- `src/cognitus_memory/config/memory_config.yaml`
- `src/cognitus_voice/config/voice_config.yaml`
- `src/cognitus_behaviors/config/behavior_config.yaml`

---

## 🧠 Node Communication

### cognitus_brain (Central Hub)

**Subscribes to:**
- `/perception/scene_description` (String)
- `/voice/command` (String)
- `/memory/event` (String)

**Publishes to:**
- `/brain/command` (String) → behaviors
- `/brain/response` (String) → voice/TTS
- `/brain/status` (String) → system monitoring

### perception_node

**Subscribes to:**
- `/camera/color/image_raw` (Image)
- `/camera/depth/image_raw` (Image)

**Publishes to:**
- `/perception/objects` (Detection2DArray)
- `/perception/scene_description` (String) → brain

### memory_manager_node

**Subscribes to:**
- `/memory/scene_update` (String) ← scene_graph
- `/brain/memory_query` (String) ← brain

**Publishes to:**
- `/memory/event` (String) → brain
- `/memory/query_response` (String) → brain

### stt_node

**Publishes to:**
- `/voice/command` (String) → brain

### tts_node

**Subscribes to:**
- `/brain/response` (String) ← brain

### behavior_controller_node

**Subscribes to:**
- `/brain/command` (String) ← brain

**Publishes to:**
- `/cmd_vel` (Twist) → robot motors
- `/behavior/status` (String) → monitoring

---

## 🔬 Research Contributions

See `academic_docs/research_papers.md` for 9 novel contributions:

1. Hierarchical Event Summarization
2. Adaptive Duty Cycling
3. Abductive Reasoning
4. Spatial Language Grounding
5. Graph-Based Memory
6. Routine Discovery
7. Task Decomposition
8. Multi-Persona Adaptation
9. Curiosity-Driven Exploration

---

## 🛠️ Build and Run

### Build Workspace
```bash
# In Docker or on LIMO Pro
colcon build --symlink-install
source install/setup.bash
```

### Run Individual Nodes (Testing)
```bash
ros2 run cognitus_cognition brain_node
ros2 run cognitus_perception perception_node
ros2 run cognitus_memory memory_manager_node
```

### Run Complete System
```bash
# Demo
ros2 launch cognitus_integration demo.launch.py

# Simulation
ros2 launch cognitus_integration simulation.launch.py

# LIMO Pro
ros2 launch cognitus_integration limo_pro.launch.py
```

---

## 📊 System Status

**Implemented:**
- ✅ All package structures
- ✅ Core node implementations
- ✅ Launch file coordination
- ✅ Configuration system
- ✅ Docker environment
- ✅ LIMO Pro integration

**Ready for:**
- ✅ Docker build and test
- ✅ LIMO Pro deployment
- ⏳ AI model integration
- ⏳ Real-world testing

---

## 🎯 Next Steps

1. **Build in Docker:**
   ```bash
   make dev
   # Inside container: colcon build
   ```

2. **Test demo launch:**
   ```bash
   ros2 launch cognitus_integration demo.launch.py
   ```

3. **Deploy to LIMO Pro:**
   ```bash
   scp -r cognitus/ agilex@robot:~/
   ssh agilex@robot
   cd cognitus && make start
   ```

---

## 📚 Documentation

- **[README.md](README.md)** - Quick start
- **[START.md](START.md)** - 30-second guide
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** - System design
- **[docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md)** - Implementation guide
- **[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)** - Deployment procedures
- **[academic_docs/](academic_docs/)** - Research papers

---

**COGNITUS is ready for development and deployment!**
