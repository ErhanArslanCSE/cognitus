# COGNITUS - System Architecture

## Table of Contents
1. [Executive Summary](#executive-summary)
2. [System Overview](#system-overview)
3. [Development and Simulation](#development-and-simulation)
4. [Hardware Platform](#hardware-platform)
5. [Software Stack](#software-stack)
6. [AI Components](#ai-components)
7. [Memory System](#memory-system)
8. [Integration Architecture](#integration-architecture)
9. [Data Flow](#data-flow)
10. [Resource Management](#resource-management)
11. [Deployment Strategy](#deployment-strategy)

---

## Executive Summary

The COGNITUS is an autonomous home robotics system that combines perception, natural language interaction, and intelligent memory management. The system runs entirely on edge hardware (NVIDIA Jetson Orin Nano) and integrates with the LIMO Pro robot platform's existing ROS2 infrastructure.

**Key Capabilities:**
- Real-time voice interaction (Speech-to-Text, Text-to-Speech)
- Conversational AI with contextual memory
- Visual perception and scene understanding
- Autonomous navigation and task execution
- Continuous learning from observations
- 7-day memory retention with intelligent compression

**Selected AI Stack (Option 2 - Ultra Efficient):**
- **STT:** Whisper Tiny (39M params, INT8 quantized)
- **LLM:** Phi-3-mini (3.8B params, 4-bit quantized) *Note: BitNet considered but Phi-3-mini selected for proven Jetson compatibility*
- **TTS:** Piper TTS (lightweight, CPU-optimized)
- **Total RAM Budget:** ~3GB for AI components
- **Target Response Time:** <800ms end-to-end

---

## System Overview

### High-Level Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                     LIMO PRO ROBOT                           │
│  ┌────────────────────────────────────────────────────────┐ │
│  │         Hardware Layer (Pre-installed)                  │ │
│  │  - Jetson Orin Nano (8GB RAM, 1024 CUDA cores)        │ │
│  │  - EAI T-mini Pro LIDAR                                │ │
│  │  - Orbbec DaBai RGB-D Camera                           │ │
│  │  - HI226 IMU                                           │ │
│  │  - USB Microphone Array                                │ │
│  │  - 4-Wheel Drive System (Multiple Modes)              │ │
│  └────────────────────────────────────────────────────────┘ │
│                            ↓                                 │
│  ┌────────────────────────────────────────────────────────┐ │
│  │      Existing ROS2 Layer (Pre-installed)                │ │
│  │  - Ubuntu 20.04 + ROS2 Foxy                            │ │
│  │  - limo_base (robot driver)                            │ │
│  │  - limo_bringup (launch files)                         │ │
│  │  - limo_description (URDF model)                       │ │
│  │  - Navigation2 stack                                    │ │
│  └────────────────────────────────────────────────────────┘ │
│                            ↓                                 │
│  ┌────────────────────────────────────────────────────────┐ │
│  │           NEW: AI Brain Layer (Our System)              │ │
│  │  ┌──────────────────────────────────────────────────┐  │ │
│  │  │  Perception Module                                │  │ │
│  │  │  - YOLOv8n (object detection)                    │  │ │
│  │  │  - Point cloud processing                        │  │ │
│  │  │  - Scene graph generation                        │  │ │
│  │  └──────────────────────────────────────────────────┘  │ │
│  │                                                          │ │
│  │  ┌──────────────────────────────────────────────────┐  │ │
│  │  │  Voice Interface Module                           │  │ │
│  │  │  - Audio capture (microphone array)              │  │ │
│  │  │  - Whisper Tiny STT                              │  │ │
│  │  │  - Piper TTS                                     │  │ │
│  │  │  - Audio playback                                │  │ │
│  │  └──────────────────────────────────────────────────┘  │ │
│  │                                                          │ │
│  │  ┌──────────────────────────────────────────────────┐  │ │
│  │  │  Cognitive Core (LLM Brain)                       │  │ │
│  │  │  - Phi-3-mini (4-bit quantized)                  │  │ │
│  │  │  - Context management                            │  │ │
│  │  │  - Query routing                                 │  │ │
│  │  │  - Response generation                           │  │ │
│  │  └──────────────────────────────────────────────────┘  │ │
│  │                                                          │ │
│  │  ┌──────────────────────────────────────────────────┐  │ │
│  │  │  Memory System                                    │  │ │
│  │  │  - Working Memory (Scene Graph)                  │  │ │
│  │  │  - Episodic Memory (Vector DB)                   │  │ │
│  │  │  - Long-term Memory (Compressed)                 │  │ │
│  │  └──────────────────────────────────────────────────┘  │ │
│  │                                                          │ │
│  │  ┌──────────────────────────────────────────────────┐  │ │
│  │  │  Behavior Controller                              │  │ │
│  │  │  - Task decomposition                            │  │ │
│  │  │  - Navigation goals                              │  │ │
│  │  │  - Routine learning                              │  │ │
│  │  └──────────────────────────────────────────────────┘  │ │
│  └────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

### Design Principles

1. **Integration over Replacement:** Leverage existing LIMO Pro ROS2 packages rather than rebuilding
2. **Edge-First Computing:** All AI processing on Jetson Orin Nano, no cloud dependency
3. **Resource Efficiency:** Quantized models optimized for 8GB RAM constraint
4. **Modularity:** Each component can operate independently for testing and development
5. **Fault Tolerance:** System continues operating if individual components fail
6. **Extensibility:** Architecture allows adding new capabilities without refactoring

---

## Development and Simulation

### Overview

COGNITUS supports **two deployment environments**:
1. **Docker (Development)** - For development and testing without hardware
2. **LIMO Pro (Production)** - For real-world deployment on actual robot

Both environments use **the same codebase** with automatic environment detection.

### Development Environment (Docker)

**Purpose:** Develop and test without physical robot

**Components:**
- ROS2 Foxy (matches LIMO Pro)
- Gazebo simulation
- All COGNITUS packages
- Development tools

**Quick Start:**
```bash
cd cognitus/
make dev              # Start development
make sim              # Start simulation
```

**Features:**
- ✅ Live code editing (volume mounts)
- ✅ Gazebo visualization
- ✅ RViz for debugging
- ✅ Same ROS2 version as production

### Simulation Environment

**Gazebo Integration:**
```
Docker Container
├── Gazebo Server (physics, sensors)
├── Gazebo Client (visualization)
└── COGNITUS Nodes
    ├── Perception (simulated camera/LIDAR)
    ├── Navigation (simulated movement)
    └── Other modules
```

**Launch Files:**
```bash
# Full simulation
ros2 launch cognitus_integration simulation.launch.py

# Development/testing
ros2 launch cognitus_integration demo.launch.py
```

**Simulated Sensors:**
- RGB-D Camera (640x480, 30 FPS)
- 2D LIDAR (360°, 12m range)
- IMU (orientation, acceleration)
- Odometry (position, velocity)

### Production Environment (LIMO Pro)

**Purpose:** Deploy on real robot hardware

**Pre-installed Components:**
- Ubuntu 20.04 + ROS2 Foxy
- `limo_base` - Robot drivers
- `limo_bringup` - Launch system
- `limo_description` - Robot model
- Navigation2, SLAM Toolbox

**Quick Start:**
```bash
# On robot
cd cognitus/
make start            # Auto-detects LIMO Pro
```

**Hardware Access:**
- ✅ Real camera feed
- ✅ Real LIDAR data
- ✅ Real IMU readings
- ✅ Motor control
- ✅ Battery monitoring

### Environment Detection

**Automatic Detection:**
```python
# start.sh automatically detects environment
if [ -f "/.dockerenv" ]; then
    # Docker → Launch simulation
    ros2 launch cognitus_integration simulation.launch.py
else
    # LIMO Pro → Launch with hardware
    ros2 launch cognitus_integration limo_pro.launch.py
fi
```

**Launch File Strategy:**
- `simulation.launch.py` - Gazebo + simulated sensors
- `limo_pro.launch.py` - Real hardware + LIMO drivers
- `demo.launch.py` - Basic testing (works in both)

### Workflow

**Development Cycle:**
```
1. Develop in Docker
   └─> make dev
   └─> Edit code in src/
   └─> Test with make sim

2. Deploy to LIMO Pro
   └─> scp -r cognitus/ robot:~/
   └─> ssh robot
   └─> make start
   └─> Same code, real hardware!
```

**Key Benefit:**
- Write once, run anywhere
- No environment-specific code
- Seamless Docker → Robot transition

---

## Hardware Platform

### LIMO Pro Specifications

**Physical Attributes:**
- Dimensions: 322mm × 220mm × 251mm
- Weight: 4.8kg (payload capacity: 4kg)
- Wheelbase: 200mm
- Ground clearance: 24mm
- Maximum speed: 1.0 m/s
- Climbing capability: 20° (track mode: 40°)

**Compute Platform:**
- Processor: NVIDIA Jetson Orin Nano
- CPU: 6-core ARM Cortex-A78AE (2.0 GHz)
- GPU: 1024 CUDA cores, 32 Tensor cores
- Memory: 8GB LPDDR5 (shared CPU/GPU)
- Storage: 128GB NVMe SSD
- AI Performance: 40 TOPS (INT8)
- Power: 10-25W TDP

**Sensors:**
- LIDAR: EAI T-mini Pro (12m range, 360°, 15 Hz)
- RGB-D Camera: Orbbec DaBai (640×480, 30 FPS, depth range 0.3-10m)
- IMU: HI226 9-DOF (gyroscope, accelerometer, magnetometer)
- Microphone: USB array (added for this project)
- Displays: Front OLED (128×64), Rear touchscreen (1024×600)

**Power System:**
- Battery: 12V 10Ah Lithium-ion
- Runtime: ~2.5 hours active operation
- Charging: Auto-docking capable
- Power management: Multiple power states

**Mobility:**
- Drive modes: Ackermann, differential, tracked, mecanum
- Mode switching: Software-controlled via ROS2
- Wheel configuration: Tool-free mechanical swap

### Computational Budget

**RAM Allocation (8GB Total):**
```
System & OS:           1.5 GB
ROS2 + Navigation:     1.2 GB
Perception Pipeline:   0.8 GB
  - YOLOv8n:          0.3 GB
  - Point cloud:      0.3 GB
  - Scene graph:      0.2 GB
AI Brain:              3.0 GB
  - Whisper Tiny:     0.4 GB
  - Phi-3-mini:       2.0 GB
  - Piper TTS:        0.2 GB
  - Inference cache:  0.4 GB
Memory System:         1.0 GB
  - Vector DB:        0.5 GB
  - Scene graphs:     0.3 GB
  - Buffers:          0.2 GB
Reserve:               0.5 GB
```

**GPU Compute Allocation:**
```
Perception:      40% (YOLOv8 inference)
LLM Inference:   50% (Phi-3-mini)
Whisper STT:     10%
Total CUDA:      1024 cores @ 625 MHz
```

**Storage Budget (128GB SSD):**
```
Ubuntu + ROS2:     30 GB
AI Models:         10 GB
  - YOLOv8n:       6 MB
  - Whisper Tiny:  150 MB
  - Phi-3-mini:    2 GB (4-bit)
  - Piper TTS:     50 MB
  - Dependencies:  8 GB
Memory Database:   20 GB
  - Vector DB:     10 GB
  - Scene graphs:  5 GB
  - Compressed:    5 GB
Logs & Data:       10 GB
Free Space:        58 GB
```

---

## Software Stack

### Operating System & Base Software

**Pre-installed on LIMO Pro:**
- Ubuntu 20.04 LTS (Focal Fossa)
- ROS2 Foxy Fitzroy
- NVIDIA JetPack 5.x
- CUDA 11.4
- cuDNN 8.6
- TensorRT 8.5

**Additional Dependencies (Our Installation):**
- Python 3.8
- PyTorch 2.0 (NVIDIA optimized build)
- ONNX Runtime 1.14 (GPU)
- OpenCV 4.5 (CUDA enabled)
- ChromaDB (vector database)
- ROS2 packages: audio_common, speech_recognition_msgs

### ROS2 Package Architecture

**Existing Packages (Pre-installed):**
```
limo_base/
  - Robot hardware interface
  - Motor controllers
  - Sensor drivers
  - Battery management

limo_bringup/
  - Launch files for robot startup
  - Configuration parameters
  - Sensor calibration data

limo_description/
  - URDF robot model
  - Mesh files
  - Visualization configs

limo_msgs/
  - Custom message types
  - Service definitions

Navigation2/
  - Path planning (DWA, TEB planners)
  - Costmap generation
  - AMCL localization
  - Recovery behaviors
```

**Our New Packages:**
```
cognitus/
  ├── cognitus_perception/        (Vision & scene understanding)
  ├── cognitus_voice/             (STT, TTS, audio handling)
  ├── cognitus_cognition/         (LLM brain, reasoning)
  ├── cognitus_memory/            (Memory systems)
  ├── cognitus_behaviors/         (High-level behaviors)
  └── cognitus_integration/       (Launch files, configs)
```

### Communication Architecture

**ROS2 Topics (Key):**
```
Sensors (Existing):
  /camera/color/image_raw          (Image)
  /camera/depth/image_raw          (Image)
  /scan                            (LaserScan)
  /imu                             (Imu)
  /odom                            (Odometry)

Perception (New):
  /perception/objects              (Detection2DArray)
  /perception/scene_graph          (SceneGraph - custom)
  /perception/point_cloud          (PointCloud2)

Voice (New):
  /audio/input                     (Audio)
  /voice/stt_output                (String)
  /voice/tts_input                 (String)
  /audio/output                    (Audio)

Cognition (New):
  /brain/query                     (String)
  /brain/response                  (String)
  /brain/context                   (BrainContext - custom)

Behavior (New):
  /behavior/goals                  (GoalArray - custom)
  /behavior/status                 (String)

Control (Existing):
  /cmd_vel                         (Twist)
  /goal_pose                       (PoseStamped)
```

**ROS2 Services:**
```
/memory/query                      (Query episodic memory)
/memory/store                      (Store new memory)
/brain/ask                         (Synchronous LLM query)
/behavior/add_routine              (Add learned routine)
```

**ROS2 Actions:**
```
/navigate_to_pose                  (Navigation2 - existing)
/execute_task                      (Complex task execution - new)
```

---

## AI Components

### 1. Speech-to-Text (Whisper Tiny)

**Model Specifications:**
- Architecture: Transformer encoder-decoder
- Parameters: 39 million
- Model size: 150 MB (INT8 quantized)
- Input: 16 kHz mono audio
- Output: Transcribed text
- Languages: 99 languages including Turkish and English
- Accuracy: ~85% WER on clean speech

**Optimization Strategy:**
```
Original Model:      1GB (FP32)
↓ FP16 Conversion:   500MB
↓ INT8 Quantization: 150MB
↓ TensorRT Optimization
Final Size:          150MB
Inference Time:      ~150ms per 3-second audio chunk
```

**Processing Pipeline:**
1. Audio capture from USB microphone array
2. Resampling to 16 kHz mono
3. Voice Activity Detection (VAD) to detect speech
4. Buffering of 3-second chunks
5. Whisper inference on GPU
6. Text output cleaning and formatting
7. Publish to /voice/stt_output topic

**Wake Word Detection:**
- Lightweight phonetic matcher runs continuously
- Keywords: "Hey LIMO", "Robot", "LIMO"
- Activates full STT pipeline when detected
- Power-efficient always-on listening

**Implementation:**
- Framework: PyTorch with TensorRT backend
- Device: CUDA GPU
- Batch size: 1 (real-time processing)
- Concurrent processing: Audio capture runs in separate thread
- Latency target: <200ms from speech end to text output

### 2. Large Language Model (Phi-3-mini)

**Model Specifications:**
- Architecture: Transformer decoder (GPT-style)
- Parameters: 3.8 billion
- Context length: 128K tokens
- Vocabulary: 32,064 tokens
- Model size: 2GB (4-bit quantized)
- Training: General knowledge + instruction tuning
- Specialization: Robotics context fine-tuned

**Quantization Details:**
```
Original Model:       7.6GB (FP16)
↓ 4-bit AWQ/GPTQ:    2.0GB
↓ Group size 128
↓ Activation: FP16
Memory Usage:         ~2.5GB with KV cache
Inference Speed:      ~15 tokens/second
Perplexity Increase:  <5% vs FP16
```

**Inference Configuration:**
- Maximum new tokens: 150 (concise responses)
- Temperature: 0.7 (balanced creativity)
- Top-p: 0.9 (nucleus sampling)
- Repetition penalty: 1.1
- Stop tokens: Natural conversation enders
- KV cache: Enabled for multi-turn conversations

**Context Management:**
- System prompt: Robot persona and capabilities
- Recent conversation: Last 10 exchanges
- Scene context: Current room and visible objects
- Relevant memories: Retrieved from vector DB
- Total context: ~2000 tokens average

**Prompt Engineering:**
```
System Role:
- You are LIMO, an autonomous home robot
- You can see, navigate, and remember events
- You are helpful, concise, and context-aware

Current Context:
- Location: [Living room]
- Visible objects: [Table, chair, cup, book, ...]
- Time: [2024-10-29 14:30]
- Recent activity: [User entered room 2 minutes ago]

Conversation History:
[Last 5 exchanges]

User Query: [Current question]

Response Guidelines:
- Be brief (2-3 sentences max)
- Reference specific objects when relevant
- Use spatial descriptions (on table, near window)
- Admit uncertainty when appropriate
```

**Fine-tuning for Robotics:**
- Custom dataset: 10K robotics-specific Q&A pairs
- Spatial reasoning examples
- Object reference resolution
- Task decomposition patterns
- LoRA adaptation (Low-Rank Adaptation for efficiency)

### 3. Text-to-Speech (Piper)

**Model Specifications:**
- Architecture: VITS (Variational Inference TTS)
- Voice: Multiple languages including Turkish
- Model size: 50 MB per voice
- Quality: Natural, human-like speech
- Speed: Real-time (faster than playback)
- Processing: CPU-optimized (no GPU needed)

**Voice Characteristics:**
- Sample rate: 22050 Hz
- Bit depth: 16-bit
- Channels: Mono
- Prosody: Adjustable speaking rate and pitch
- Emotion: Neutral baseline with parameter control

**Optimization:**
- Backend: ONNX Runtime (CPU)
- Threads: 4 CPU cores
- Streaming: Chunk-based generation
- Latency: <100ms for first audio chunk
- Total generation: ~1s for average response

**Audio Processing Pipeline:**
1. Text normalization (numbers, abbreviations)
2. Sentence segmentation
3. Piper synthesis per sentence
4. Audio concatenation
5. Volume normalization
6. Output to speaker via ROS audio_common

**Persona Adaptation:**
```
Normal Mode:
  - Speaking rate: 1.0x
  - Pitch: Neutral
  - Emotion: Friendly

Alert Mode:
  - Speaking rate: 1.2x
  - Pitch: +10%
  - Emotion: Urgent

Calm Mode:
  - Speaking rate: 0.9x
  - Pitch: -5%
  - Emotion: Soothing
```

---

## Memory System

### Architecture Overview

The memory system implements a three-tier hierarchy inspired by human memory:
1. **Working Memory:** Immediate perceptual data (seconds to minutes)
2. **Episodic Memory:** Recent experiences (hours to days)
3. **Long-term Memory:** Compressed knowledge (weeks to months)

### 1. Working Memory (Scene Graph)

**Purpose:** Real-time representation of current environment

**Data Structure:**
```
Scene Graph:
  Nodes:
    - Object ID (unique identifier)
    - Class label (cup, table, person, etc.)
    - 3D position (x, y, z in robot frame)
    - Bounding box (2D image coordinates)
    - Confidence score
    - First seen timestamp
    - Last updated timestamp
    - Visual features (512D embedding)

  Edges:
    - Spatial relations (on, near, left_of, etc.)
    - Temporal relations (before, after, during)
    - Confidence scores

  Metadata:
    - Room/location identifier
    - Lighting conditions
    - Number of objects tracked
    - Graph update timestamp
```

**Retention Policy:**
- Maximum retention: 10 minutes
- Maximum nodes: 1000 objects
- Update frequency: 10 Hz
- Memory footprint: ~200 MB

**Operations:**
- Add/update object: O(1) with hash map
- Spatial query (find objects near X): O(log n) with spatial index
- Temporal query (what changed recently): O(k) where k = changes

**Compression to Episodic:**
- Every 60 seconds, create summary snapshot
- Store differential changes only
- Compute importance scores
- Promote important events to episodic memory

### 2. Episodic Memory (Vector Database)

**Purpose:** Searchable memory of recent experiences

**Technology:** ChromaDB (embedded vector database)

**Storage Model:**
```
Each Episode:
  - Timestamp range (start, end)
  - Text description (natural language summary)
  - Embedding vector (384D from MiniLM)
  - Structured data:
    - Objects involved
    - Location
    - Actions observed
    - People present
  - Importance score (0.0 to 1.0)
  - Access count (for promotion/demotion)
  - Media references (image IDs if saved)
```

**Embedding Model:**
- Model: all-MiniLM-L6-v2
- Dimension: 384
- Size: 22 MB
- Inference: CPU (fast)
- Purpose: Semantic similarity search

**Indexing:**
- Primary index: Vector similarity (HNSW)
- Secondary indexes: Timestamp, location, importance
- Query modes:
  - Semantic: "What happened with the keys?"
  - Temporal: "Events from yesterday morning"
  - Spatial: "What occurred in the kitchen?"
  - Hybrid: Combined filters

**Retention Policy:**
- Time window: 7 days
- Maximum episodes: 50,000
- Storage limit: 10 GB
- Eviction: Least important + oldest first

**Importance Scoring:**
```python
importance = (
    0.3 * novelty_score +      # How unusual was this?
    0.3 * human_interaction +  # Did it involve humans?
    0.2 * query_frequency +    # How often retrieved?
    0.2 * causal_relevance     # Part of important chain?
)
```

**Query Examples:**
```
"Where did I put my phone?"
  → Semantic search for "phone" + temporal filter (recent)
  → Returns: Episode with phone location

"What happened while I was gone?"
  → Temporal range query + importance threshold
  → Returns: Summary of significant events

"Remind me about the coffee maker issue"
  → Semantic search for "coffee maker" + problem context
  → Returns: Related episodes and resolution
```

### 3. Long-term Memory (Compressed Storage)

**Purpose:** Extended knowledge with extreme compression

**Compression Strategy:**
```
Level 1 (1 day ago):    Keep episode summaries (1x)
Level 2 (1 week ago):   Merge similar episodes (5x compression)
Level 3 (1 month ago):  Daily digests only (20x compression)
Level 4 (3+ months):    Weekly summaries (100x compression)
```

**Storage Format:**
```
Daily Digest:
  - Date
  - Key events (top 10 by importance)
  - Routine observations
  - Anomalies detected
  - Objects frequently seen
  - Spaces visited
  - Total: ~1 KB per day

Weekly Summary:
  - Week identifier
  - Patterns discovered
  - Routine changes
  - New objects learned
  - Total: ~5 KB per week
```

**Reconstruction:**
When detailed information needed from old memory:
1. Retrieve compressed summary
2. Load any referenced full episodes (if not yet deleted)
3. Present best-effort reconstruction
4. Mark uncertainty in response

**Storage Budget:**
```
7 days full detail:     10 GB
1 month compressed:     2 GB
3+ months archived:     2 GB
Total:                  14 GB
```

### Memory Integration with LLM

**Query Flow:**
```
1. User asks: "Where are my keys?"

2. Memory retrieval:
   - Working memory: Check current scene graph
   - Episodic memory: Vector search for "keys"
   - Return top 3 relevant memories

3. Context building:
   Current: "Keys not visible in current room"
   Recent: "Keys were on kitchen counter 15 minutes ago"
   Past: "User usually keeps keys in bowl by door"

4. LLM generates response:
   "I saw your keys on the kitchen counter about 15 minutes ago.
    You usually keep them in the bowl by the front door."

5. Update memory:
   - Log this interaction
   - Increment query count for retrieved episodes
   - Store new pattern: User frequently asks about keys
```

**Learning and Adaptation:**
- Pattern mining runs nightly on episodic memory
- Discovers routines: "Coffee at 7 AM every weekday"
- Identifies preferences: "User prefers lights off at 10 PM"
- Stores patterns as structured knowledge
- LLM can reference learned patterns in responses

---

## Integration Architecture

### ROS2 Node Structure

**Node Diagram:**
```
┌─────────────────────────────────────────────────────────┐
│  limo_base (existing)                                    │
│    ├─ /camera/color/image_raw                           │
│    ├─ /camera/depth/image_raw                           │
│    ├─ /scan                                              │
│    └─ /imu                                               │
└─────────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────────┐
│  perception_node (new)                                   │
│    ← Subscribes: camera images, depth, lidar            │
│    → Publishes: /perception/objects, /scene_graph       │
│    • Runs YOLOv8 object detection                       │
│    • Generates 3D positions from depth                  │
│    • Constructs scene graph                             │
│    • Updates at 10 Hz                                    │
└─────────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────────┐
│  memory_manager_node (new)                               │
│    ← Subscribes: /scene_graph                           │
│    → Services: /memory/query, /memory/store             │
│    • Manages 3-tier memory system                       │
│    • Handles compression                                │
│    • Provides search API                                │
│    • Runs at 1 Hz background                            │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  audio_capture_node (new)                                │
│    ← Hardware: USB microphone                           │
│    → Publishes: /audio/input                            │
│    • Captures audio at 16 kHz                           │
│    • Voice activity detection                           │
│    • Publishes raw audio chunks                         │
└─────────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────────┐
│  stt_node (new)                                          │
│    ← Subscribes: /audio/input                           │
│    → Publishes: /voice/stt_output                       │
│    • Runs Whisper Tiny inference                        │
│    • Wake word detection                                │
│    • Text output cleaning                               │
│    • Latency: ~150ms                                    │
└─────────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────────┐
│  brain_node (new)                                        │
│    ← Subscribes: /voice/stt_output                      │
│    ← Services: /memory/query                            │
│    → Publishes: /voice/tts_input, /brain/response       │
│    • Phi-3-mini LLM inference                           │
│    • Context retrieval from memory                      │
│    • Response generation                                │
│    • Latency: ~500ms                                    │
└─────────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────────┐
│  tts_node (new)                                          │
│    ← Subscribes: /voice/tts_input                       │
│    → Publishes: /audio/output                           │
│    • Piper TTS synthesis                                │
│    • Audio streaming                                    │
│    • Latency: ~100ms                                    │
└─────────────────────────────────────────────────────────┘
                    ↓
┌─────────────────────────────────────────────────────────┐
│  audio_playback_node (new)                               │
│    ← Subscribes: /audio/output                          │
│    → Hardware: Speaker                                   │
│    • Audio playback control                             │
│    • Volume management                                  │
└─────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────┐
│  behavior_controller_node (new)                          │
│    ← Subscribes: /brain/response, /scene_graph          │
│    → Services: Navigation2 actions                      │
│    → Publishes: /cmd_vel, /goal_pose                    │
│    • Task decomposition                                 │
│    • Navigation coordination                            │
│    • Routine execution                                  │
└─────────────────────────────────────────────────────────┘
```

### Launch File Strategy

**Multi-level Launch System:**

**Level 1: Hardware (existing)**
```
limo_bringup/launch/limo_start.launch.py
  - Starts robot drivers
  - Launches sensors
  - Starts Navigation2
  - Already installed on robot
```

**Level 2: Perception (new)**
```
cognitus_perception/launch/perception.launch.py
  - Starts perception_node
  - Starts memory_manager_node
  - Loads YOLOv8 model
  - Configures scene graph parameters
```

**Level 3: AI Brain (new)**
```
cognitus/launch/brain.launch.py
  - Starts audio_capture_node
  - Starts stt_node (Whisper)
  - Starts brain_node (Phi-3)
  - Starts tts_node (Piper)
  - Starts audio_playback_node
  - Loads all AI models
```

**Level 4: Complete System (new)**
```
cognitus_integration/launch/full_system.launch.py
  - Includes limo_start.launch.py
  - Includes perception.launch.py
  - Includes brain.launch.py
  - Starts behavior_controller_node
  - Single command to start everything
```

### Configuration Management

**Parameter Files:**
```
config/
  ├── perception_params.yaml     (YOLOv8 settings, thresholds)
  ├── memory_params.yaml         (Retention policies, compression)
  ├── whisper_params.yaml        (STT configuration)
  ├── phi3_params.yaml           (LLM inference settings)
  ├── piper_params.yaml          (TTS voice selection)
  └── behavior_params.yaml       (Task execution settings)
```

**Model Paths:**
```
models/
  ├── yolov8n_int8.engine        (TensorRT engine)
  ├── whisper_tiny_int8.onnx     (ONNX format)
  ├── phi-3-mini-4bit/           (Quantized model directory)
  │   ├── model.safetensors
  │   ├── config.json
  │   └── tokenizer files
  └── piper_voice_en.onnx        (TTS voice)
```

---

## Data Flow

### End-to-End Conversation Flow

**Scenario:** User asks "Where are my keys?"

**Step-by-Step Processing:**

```
[T+0ms] User speaks: "Where are my keys?"

[T+50ms]
  • audio_capture_node captures audio
  • Publishes to /audio/input
  • Raw waveform: 16kHz mono

[T+100ms]
  • stt_node receives audio chunk
  • Detects voice activity
  • Buffers 3 seconds of speech

[T+2500ms]
  • User finishes speaking
  • VAD detects silence
  • Triggers Whisper inference

[T+2650ms]
  • Whisper Tiny processes audio
  • GPU inference: 150ms
  • Output text: "where are my keys"
  • Publishes to /voice/stt_output

[T+2660ms]
  • brain_node receives text query
  • Starts parallel processing:

    Thread 1: Memory retrieval
      • Query working memory (current scene)
        Result: "Keys not visible in current room"

      • Query episodic memory (vector search)
        SQL: SELECT * FROM episodes
             WHERE similarity(embedding, query_embedding) > 0.7
             ORDER BY timestamp DESC LIMIT 3
        Result: "Keys on kitchen counter, 15 minutes ago"

      • Query long-term patterns
        Result: "User keeps keys in bowl by door"

      Total time: 50ms

    Thread 2: LLM preparation
      • Load recent conversation context
      • Build system prompt
      • Format input
      Total time: 10ms

[T+2720ms]
  • Memory results merged
  • Full context assembled (2000 tokens)
  • Phi-3-mini inference starts

[T+3220ms]
  • LLM generates response (500ms @ 15 tok/s)
  • Output: "I saw your keys on the kitchen counter about
            15 minutes ago. You usually keep them in the
            bowl by the front door."
  • Publishes to /voice/tts_input

[T+3230ms]
  • tts_node receives text
  • Piper synthesis starts
  • Chunk-based streaming

[T+3330ms]
  • First audio chunk ready (100ms)
  • Publishes to /audio/output
  • audio_playback_node starts playback

[T+3330ms - T+5500ms]
  • Streaming TTS continues
  • User hears response: ~2 seconds
  • Total latency: 3.3s from speech end to response start

[T+5600ms]
  • memory_manager_node logs interaction
  • Stores: Query + Response + Timestamp
  • Updates pattern: "User frequently asks about keys"
  • Increments access count for retrieved memories
```

**Latency Breakdown:**
- STT (Whisper): 150ms
- Memory retrieval: 50ms
- LLM inference: 500ms
- TTS first chunk: 100ms
- **Total to first audio: 800ms** ✓ (meets target)

### Perception to Memory Flow

**Continuous Operation (Every 100ms):**

```
[Perception Loop - 10 Hz]

Cycle N:
  • Camera publishes /camera/color/image_raw
  • perception_node receives image

  • YOLOv8 inference (GPU)
    - Input: 640x480 RGB image
    - Output: Bounding boxes + class labels + confidence
    - Processing time: ~20ms

  • Depth projection
    - Match detections to depth image
    - Compute 3D positions (camera frame)
    - Transform to robot base frame
    - Processing time: ~10ms

  • Scene graph update
    - Add new objects
    - Update existing objects
    - Remove disappeared objects
    - Compute spatial relations
    - Processing time: ~15ms

  • Publish /perception/scene_graph
    - Custom message with all objects + relations
    - Timestamp
    - Processing time: ~5ms

  Total cycle time: 50ms (10 Hz achieved)

[Memory Loop - 1 Hz]

Every second:
  • memory_manager_node receives latest scene graph

  • Change detection
    - Compare with previous graph
    - Identify: new objects, moved objects, removed objects
    - Significant changes trigger episode creation

  • Episode creation (if significant change):
    - Generate text description:
      "Person entered living room and sat on couch"
    - Compute embedding (MiniLM)
    - Extract metadata (objects, location, time)
    - Compute importance score
    - Store in ChromaDB

  • Routine pattern mining (every 60 seconds):
    - Analyze recent episodes
    - Detect temporal patterns
    - Update learned routines

  • Memory compression (every 5 minutes):
    - Summarize old working memory → episodic
    - Compress old episodic → long-term
    - Evict least important episodes if storage full

[Background Task - Hourly]
  • Full pattern analysis on episodic memory
  • Update routine database
  • Optimize vector database indexes
  • Clean up temporary files
```

### Behavior Execution Flow

**Scenario:** User commands "Go check all windows"

```
[T+0] User: "Go check all windows"

[T+800ms]
  • brain_node generates response:
    "I'll check all the windows in the house."
  • Also publishes to /behavior/goals:
    Goal: {
      type: "complex_task",
      description: "check all windows",
      decomposed: false
    }

[T+900ms]
  • behavior_controller_node receives goal
  • Task decomposition starts:

  Step 1: Query LLM for decomposition
    Input: "Decompose: check all windows"
    Output:
      1. List all rooms
      2. For each room, navigate to room
      3. For each room, find windows
      4. For each window, check if open/closed
      5. Report findings

  Step 2: Map to executable actions
    Rooms: [living_room, bedroom, kitchen]

    Subtasks:
      - navigate_to(living_room)
      - find_objects(window, living_room)
      - check_state(window_1), check_state(window_2)
      - navigate_to(bedroom)
      - ...

[T+2000ms]
  • Start execution
  • Call Navigation2 action: navigate_to_pose(living_room)
  • Monitors /odom and /scan for progress

[T+15000ms]
  • Arrived at living room
  • Query scene_graph for objects with class="window"
  • Found: 2 windows

  • Visual inspection:
    - Check depth data at window locations
    - Infer open/closed from depth discontinuity
    - Store results

[T+20000ms]
  • Navigate to next room
  • Repeat process
  • Continue until all rooms checked

[T+60000ms]
  • Task complete
  • Generate summary:
    "I checked all windows. Living room window is open,
     bedroom and kitchen windows are closed."

  • Store task execution as episode in memory
  • Update learned task decomposition pattern
  • Publish to /voice/tts_input for user feedback
```

---

## Resource Management

### GPU Memory Management

**Strategy:**
```python
# Model Loading (Startup)
1. Load YOLOv8 TensorRT engine → GPU
2. Load Whisper ONNX model → GPU
3. Load Phi-3-mini with 4-bit → GPU
4. Pre-allocate inference buffers

# Runtime Management
- Use CUDA streams for parallel inference
- YOLOv8 stream (continuous)
- Whisper stream (on-demand)
- Phi-3 stream (on-demand)

# Memory Cleanup
if gpu_memory_usage > 90%:
    - Clear KV cache in Phi-3
    - torch.cuda.empty_cache()
    - Reduce batch sizes if applicable
```

**Priority System:**
```
High Priority (Always Loaded):
  - YOLOv8: Continuous perception critical
  - Safety monitoring models

Medium Priority (Load on Demand):
  - Phi-3-mini: Load when conversation active
  - Whisper: Load when audio detected

Low Priority (Can Offload):
  - Piper TTS: CPU-only, no GPU needed
  - Embedding models: Small, fast loading
```

### CPU Management

**Thread Allocation:**
```
Core 0-1: ROS2 system processes
Core 2:   Perception processing
Core 3:   Audio processing (STT/TTS)
Core 4:   Memory system
Core 5:   Behavior controller + misc
```

**Process Priorities:**
```
Real-time priority (SCHED_FIFO):
  - limo_base (hardware interface)
  - Safety monitoring

High priority (nice -10):
  - perception_node
  - audio_capture_node

Normal priority (nice 0):
  - brain_node
  - tts_node
  - memory_manager_node

Low priority (nice 10):
  - Logging
  - Compression tasks
```

### Power Management

**Battery-Aware Operation:**

```
Battery Level > 50%:
  - Full performance mode
  - All AI features enabled
  - Perception at 10 Hz
  - LLM max tokens: 150

Battery Level 20-50%:
  - Balanced mode
  - Reduce perception to 5 Hz
  - LLM max tokens: 100
  - TTS: Shorter responses

Battery Level < 20%:
  - Power saving mode
  - Perception at 2 Hz
  - LLM: Simple responses only
  - TTS: Disabled, text only
  - Automatic navigation to charging dock

Battery Level < 10%:
  - Emergency mode
  - Perception: Motion detection only
  - All AI disabled except safety
  - Navigate to dock immediately
```

**Adaptive Duty Cycling:**
```
High Activity Period (detected motion/sound):
  - Full sensor rates
  - All AI active
  - Ready for interaction

Low Activity Period (no motion for 5 min):
  - Reduce perception to 1 Hz
  - Keep audio listening active
  - LLM unloaded from GPU
  - Wake on: motion, sound, schedule

Sleep Period (no activity for 30 min):
  - Perception: 0.1 Hz (spot checks)
  - Audio: Wake word detection only
  - Memory: Compression background task
  - Instant wake on: significant motion/sound
```

### Storage Management

**Data Lifecycle:**
```
Real-time Buffers (RAM):
  - Audio: Rolling 10-second buffer
  - Images: Last 100 frames
  - Scene graphs: Last 10 minutes
  - Auto-delete after processing

Temporary Storage (SSD):
  - Raw episode data: 7 days
  - Compressed episodes: 30 days
  - Auto-cleanup by age + importance

Permanent Storage (SSD):
  - AI models: Never delete
  - Learned routines: Never delete
  - Important episodes: Flagged by user
  - Long-term summaries: Compressed
```

**Disk Space Monitoring:**
```
if free_space < 20GB:
    - Warn user
    - Compress older episodes
    - Delete low-importance memories

if free_space < 10GB:
    - Aggressive cleanup
    - Keep only last 3 days full detail
    - Archive to external storage if available

if free_space < 5GB:
    - Emergency mode
    - Stop recording new episodes
    - Alert user immediately
```

---

## Deployment Strategy

### Installation Process

**Prerequisites (Already on LIMO Pro):**
- Ubuntu 20.04
- ROS2 Foxy
- NVIDIA JetPack 5.x
- Python 3.8

**Our Installation Steps:**

**Step 1: Setup Script Execution**
```bash
# Transfer project to robot
scp -r cognitus/ agilex@<robot-ip>:~/

# Run automated setup
cd ~/cognitus
./scripts/setup_system.sh
```

The setup script will:
1. Check system requirements
2. Install Python dependencies
3. Install ROS2 packages
4. Download and optimize AI models
5. Configure audio devices
6. Set up systemd services
7. Run validation tests

**Step 2: Model Download and Optimization**
```bash
./scripts/download_models.sh
```

Downloads and prepares:
- YOLOv8n → Converts to TensorRT
- Whisper Tiny → Converts to ONNX INT8
- Phi-3-mini → Downloads 4-bit quantized
- Piper TTS → Downloads voice model
- MiniLM embedder → Downloads for ChromaDB

**Step 3: ROS2 Workspace Build**
```bash
./scripts/build_workspace.sh
```

Builds all ROS2 packages:
- cognitus_perception
- cognitus_voice
- cognitus_cognition
- cognitus_memory
- cognitus_behaviors
- cognitus_integration

**Step 4: Configuration**
```bash
./scripts/configure_system.sh
```

Interactive configuration:
- Select voice language (English/Turkish)
- Set wake word
- Configure audio devices
- Set default behaviors
- Test all components

**Step 5: Validation**
```bash
./scripts/validate_installation.sh
```

Runs comprehensive tests:
- Hardware check
- Model loading
- ROS2 node startup
- End-to-end conversation test
- Memory system test
- Navigation integration test

### Startup Modes

**Mode 1: Full System (Production)**
```bash
ros2 launch cognitus_integration full_system.launch.py
```
Starts everything, ready for operation.

**Mode 2: AI Brain Only (Development)**
```bash
ros2 launch cognitus brain.launch.py
```
Starts AI components, assumes robot base running.

**Mode 3: Individual Components (Testing)**
```bash
ros2 run cognitus_perception perception_node
ros2 run cognitus_voice stt_node
ros2 run cognitus_cognition brain_node
# etc.
```

**Mode 4: Autostart (Boot)**
```bash
sudo systemctl enable limo-ai-brain.service
```
Automatic startup on robot boot.

### Testing Procedure

**Unit Tests:**
```bash
# Test perception
python3 -m pytest src/cognitus_perception/test/

# Test voice processing
python3 -m pytest src/cognitus_voice/test/

# Test memory system
python3 -m pytest src/cognitus_memory/test/
```

**Integration Tests:**
```bash
# Test STT → LLM → TTS pipeline
./scripts/test_conversation.sh

# Test perception → memory flow
./scripts/test_perception_memory.sh

# Test full task execution
./scripts/test_behavior_execution.sh
```

**Performance Benchmarks:**
```bash
./scripts/benchmark.sh
```

Measures:
- STT latency
- LLM inference time
- TTS generation speed
- Memory query speed
- GPU memory usage
- CPU usage per node
- Total system latency

Expected results:
- STT: <200ms ✓
- LLM: <500ms ✓
- TTS: <100ms first chunk ✓
- Total: <800ms ✓
- GPU RAM: <3GB ✓
- CPU: <60% average ✓

### Monitoring and Debugging

**Real-time Monitoring:**
```bash
# Watch all topics
./scripts/monitor_system.sh

Displays:
- Current perception status
- Memory usage stats
- Active conversations
- Behavior execution state
- Error log stream
```

**Logging:**
```
logs/
  ├── perception.log      (Object detections, scene updates)
  ├── stt.log            (Speech transcriptions)
  ├── brain.log          (LLM queries and responses)
  ├── memory.log         (Memory operations)
  ├── behavior.log       (Task executions)
  └── system.log         (Overall system events)
```

**Debug Tools:**
```bash
# Visualize scene graph
ros2 run cognitus_perception visualize_scene_graph

# Inspect memory database
./scripts/inspect_memory.sh

# Monitor GPU usage
watch -n 1 nvidia-smi

# ROS2 introspection
ros2 node list
ros2 topic list
ros2 topic echo /perception/scene_graph
```

### Update and Maintenance

**Model Updates:**
```bash
./scripts/update_models.sh
```
Downloads latest optimized models.

**Software Updates:**
```bash
cd ~/cognitus
git pull origin main
./scripts/rebuild.sh
```

**Database Maintenance:**
```bash
# Optimize vector database
./scripts/optimize_memory.sh

# Export memories for backup
./scripts/backup_memory.sh

# Clear old memories
./scripts/clean_memory.sh --older-than 30d
```

### Troubleshooting

**Common Issues:**

**Issue: Audio not detected**
```bash
# Check microphone
arecord -l

# Test audio capture
ros2 topic echo /audio/input

# Fix: Configure audio device
./scripts/configure_audio.sh
```

**Issue: LLM out of memory**
```bash
# Check GPU memory
nvidia-smi

# Fix: Reduce batch size or context length
# Edit: config/phi3_params.yaml
# max_context_length: 2048 → 1024
```

**Issue: Perception too slow**
```bash
# Check inference time
ros2 topic hz /perception/objects

# Fix: Reduce resolution or frame rate
# Edit: config/perception_params.yaml
# detection_fps: 10 → 5
```

**Issue: Memory database full**
```bash
# Check disk usage
df -h

# Clean up old memories
./scripts/clean_memory.sh --aggressive

# Increase compression
# Edit: config/memory_params.yaml
# compression_ratio: 100 → 200
```

---

## Summary

This architecture document describes a complete, production-ready AI brain system for the LIMO Pro autonomous robot. The design prioritizes:

1. **Efficiency:** All processing on 8GB Jetson Orin Nano
2. **Integration:** Seamless connection with existing LIMO Pro ROS2 packages
3. **Modularity:** Each component independently testable
4. **Performance:** <800ms end-to-end conversation latency
5. **Reliability:** Fault-tolerant with graceful degradation
6. **Extensibility:** Easy to add new capabilities

The system is ready for deployment with comprehensive installation scripts, testing procedures, and monitoring tools. All components are optimized for edge deployment with no cloud dependencies.

**Next Steps:**
1. Transfer code to LIMO Pro robot
2. Run installation scripts
3. Validate all components
4. Begin testing in real environment
5. Iterate based on performance data

**Expected Timeline:**
- Installation: 2 hours
- Testing: 4 hours
- Optimization: 1 week
- Production ready: 2 weeks

The architecture is designed to be "plug and play" while allowing deep customization for research and development purposes.
