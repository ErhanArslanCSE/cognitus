# COGNITUS - Project Status and Implementation Guide

## Project Overview

This document provides a complete overview of the project structure, what has been created, what needs to be implemented, and how to proceed with development.

## ✅ Completed Components

### 1. Documentation
- `ARCHITECTURE.md` - Complete system architecture (100% complete)
- `README.md` - Project overview and quick start guide (100% complete)
- `PROJECT_STATUS.md` - This file (100% complete)

### 2. Directory Structure
Complete directory tree created with all necessary folders:
```
cognitus/
├── src/              # ROS2 packages (structure created)
├── scripts/          # Setup and utility scripts (partially complete)
├── models/           # AI model storage (empty, populated during setup)
├── config/           # Global configurations (needs YAML files)
├── logs/             # System logs (created at runtime)
└── docs/             # Additional documentation (needs detail docs)
```

### 3. Setup Scripts
- `scripts/setup_system.sh` - Complete installation script (100% complete)
- `scripts/download_models.sh` - Model download and optimization (100% complete)

### 4. Python Dependencies
- `requirements.txt` - All Python packages listed (100% complete)

### 5. ROS2 Package Structure (Perception Example)
- `src/cognitus_perception/package.xml` - Package manifest (100% complete)
- `src/cognitus_perception/setup.py` - Package setup file (100% complete)

## 🚧 Components to Implement

### Priority 1: Core ROS2 Nodes (Critical)

#### 1.1 Perception Package (`src/cognitus_perception/`)

**Files needed:**

`cognitus_perception/perception_node.py` - Main perception node
```python
Purpose: YOLOv8 object detection from camera
Subscribes to: /camera/color/image_raw, /camera/depth/image_raw
Publishes to: /perception/objects, /perception/scene_graph
Key functions:
  - Load YOLOv8 TensorRT engine
  - Process images at 10 Hz
  - 3D localization from depth
  - Publish Detection2DArray messages
```

`cognitus_perception/scene_graph_node.py` - Scene graph builder
```python
Purpose: Build and maintain scene graph from detections
Subscribes to: /perception/objects
Publishes to: /perception/scene_graph
Key functions:
  - Create/update scene graph
  - Compute spatial relations
  - Track object persistence
  - Differential updates
```

`cognitus_perception/visualize_detections.py` - Visualization tool
```python
Purpose: Debug tool to visualize detections
Subscribes to: /perception/objects, /camera/color/image_raw
Displays: Annotated images with bounding boxes
```

**Configuration needed:**

`config/perception_config.yaml`:
```yaml
detection:
  model_path: "../models/yolov8n.engine"
  confidence_threshold: 0.5
  nms_threshold: 0.4
  input_size: [640, 480]
  fps_target: 10

scene_graph:
  max_objects: 1000
  retention_time: 600  # 10 minutes
  spatial_threshold: 0.5  # meters
```

**Launch file:**

`launch/perception.launch.py`:
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cognitus_perception',
            executable='perception_node',
            name='perception',
            parameters=[{'config_file': 'config/perception_config.yaml'}]
        ),
        Node(
            package='cognitus_perception',
            executable='scene_graph_node',
            name='scene_graph'
        )
    ])
```

#### 1.2 Voice Package (`src/cognitus_voice/`)

**Files needed:**

`cognitus_voice/audio_capture_node.py`:
```python
Purpose: Capture audio from microphone
Publishes to: /audio/input (Audio message)
Key functions:
  - PyAudio stream setup
  - VAD (Voice Activity Detection)
  - 16kHz mono resampling
```

`cognitus_voice/stt_node.py`:
```python
Purpose: Speech-to-text using Whisper
Subscribes to: /audio/input
Publishes to: /voice/stt_output (String)
Key functions:
  - Load Whisper ONNX model
  - Wake word detection
  - GPU inference
  - Text cleaning
```

`cognitus_voice/tts_node.py`:
```python
Purpose: Text-to-speech using Piper
Subscribes to: /voice/tts_input (String)
Publishes to: /audio/output (Audio)
Key functions:
  - Load Piper ONNX model
  - Text normalization
  - Streaming audio generation
```

`cognitus_voice/audio_playback_node.py`:
```python
Purpose: Play audio through speakers
Subscribes to: /audio/output
Key functions:
  - PyAudio playback
  - Volume control
```

**Package files:**
- `package.xml` - Copy from cognitus_perception template
- `setup.py` - Modify for cognitus_voice entry points
- `config/voice_config.yaml` - STT/TTS parameters

#### 1.3 Cognition Package (`src/cognitus_cognition/`)

**Files needed:**

`cognitus_cognition/brain_node.py`:
```python
Purpose: LLM-based conversational AI
Subscribes to: /voice/stt_output
Publishes to: /voice/tts_input, /brain/response
Calls services: /memory/query
Key functions:
  - Load Phi-3-mini 4-bit model
  - Context retrieval from memory
  - Prompt engineering
  - Response generation
  - Conversation history management
```

`cognitus_cognition/context_manager.py`:
```python
Purpose: Manage conversation context
Functions:
  - Build system prompt
  - Retrieve relevant memories
  - Format context for LLM
  - Track conversation history
```

**Configuration:**

`config/cognition_config.yaml`:
```yaml
llm:
  model_path: "../models/phi-3-mini-4bit"
  max_context_length: 2048
  max_new_tokens: 150
  temperature: 0.7
  top_p: 0.9

conversation:
  history_length: 10
  context_tokens: 2000
```

#### 1.4 Memory Package (`src/cognitus_memory/`)

**Files needed:**

`cognitus_memory/memory_manager_node.py`:
```python
Purpose: Manage 3-tier memory system
Subscribes to: /perception/scene_graph
Provides services: /memory/query, /memory/store
Key functions:
  - Working memory (scene graph)
  - Episodic memory (ChromaDB)
  - Long-term memory (compression)
  - Importance scoring
  - Pattern mining
```

`cognitus_memory/scene_graph_memory.py`:
```python
Purpose: Working memory implementation
Functions:
  - Store current scene graph
  - Query by object/location/time
  - Differential compression
```

`cognitus_memory/episodic_memory.py`:
```python
Purpose: Vector database interface
Functions:
  - ChromaDB operations
  - Semantic search
  - Episode storage/retrieval
```

`cognitus_memory/compressor.py`:
```python
Purpose: Memory compression
Functions:
  - Hierarchical compression
  - Daily digest generation
  - Importance-based retention
```

#### 1.5 Behaviors Package (`src/cognitus_behaviors/`)

**Files needed:**

`cognitus_behaviors/behavior_controller_node.py`:
```python
Purpose: High-level task execution
Subscribes to: /brain/response, /perception/scene_graph
Publishes to: /cmd_vel, /goal_pose
Calls actions: Navigation2 actions
Key functions:
  - Task decomposition
  - Navigation coordination
  - Routine execution
```

`cognitus_behaviors/task_decomposer.py`:
```python
Purpose: Break complex tasks into subtasks
Functions:
  - LLM-based decomposition
  - Capability checking
  - Execution planning
```

#### 1.6 Integration Package (`src/cognitus_integration/`)

**Files needed:**

`launch/full_system.launch.py`:
```python
Purpose: Launch complete system
Includes:
  - limo_bringup (existing LIMO packages)
  - All AI brain components
  - Visualization tools
```

### Priority 2: Configuration Files

All YAML configuration files need to be created in respective `config/` directories:

1. `src/cognitus_perception/config/perception_config.yaml`
2. `src/cognitus_voice/config/voice_config.yaml`
3. `src/cognitus_cognition/config/cognition_config.yaml`
4. `src/cognitus_memory/config/memory_config.yaml`
5. `src/cognitus_behaviors/config/behavior_config.yaml`

### Priority 3: Support Scripts

`scripts/` directory needs:

1. `validate_installation.sh` - Test all components
2. `monitor_system.sh` - Real-time monitoring
3. `benchmark.sh` - Performance testing
4. `configure_audio.sh` - Audio device setup
5. `backup_memory.sh` - Database backup
6. `clean_memory.sh` - Database cleanup
7. `build_workspace.sh` - ROS2 build automation
8. `test_conversation.sh` - E2E conversation test

### Priority 4: Message Definitions

Custom ROS2 messages needed:

`src/cognitus_perception/msg/SceneGraph.msg`:
```
Header header
SceneNode[] nodes
SceneEdge[] edges
```

`src/cognitus_perception/msg/SceneNode.msg`:
```
uint32 id
string label
geometry_msgs/Point position
float32 confidence
time first_seen
time last_updated
```

`src/cognitus_perception/msg/SceneEdge.msg`:
```
uint32 source_id
uint32 target_id
string relation_type
float32 confidence
```

### Priority 5: Testing

Each package needs tests in `test/` directory:

1. Unit tests for core functions
2. Integration tests for ROS2 nodes
3. Performance benchmarks
4. Mock data for testing without hardware

## 📋 Implementation Roadmap

### Phase 1: Core Functionality (Week 1)

1. Implement perception node with YOLOv8
2. Implement basic voice interface (STT/TTS)
3. Implement simple LLM brain (no memory yet)
4. Create test scripts
5. **Goal:** "Hey LIMO, what do you see?" → "I see a table, chair, and cup"

### Phase 2: Memory System (Week 2)

1. Implement scene graph memory
2. Implement episodic memory (ChromaDB)
3. Integrate memory with LLM
4. Add query interface
5. **Goal:** "Where are my keys?" → "On the kitchen counter 10 minutes ago"

### Phase 3: Behaviors (Week 3)

1. Implement task decomposition
2. Integrate with Navigation2
3. Add routine learning
4. Test complex tasks
5. **Goal:** "Go check all windows" → Robot navigates and reports

### Phase 4: Optimization (Week 4)

1. Performance tuning
2. Memory optimization
3. Battery-aware operation
4. Comprehensive testing
5. **Goal:** System runs 7 days continuously

## 🔧 Development Workflow

### Setting Up Development Environment

1. Transfer project to LIMO Pro:
```bash
scp -r cognitus/ agilex@<robot-ip>:~/
```

2. SSH into robot:
```bash
ssh agilex@<robot-ip>
cd ~/cognitus
```

3. Run setup (first time only):
```bash
./scripts/setup_system.sh
```

### Developing a New Node

1. Create Python file in appropriate package:
```bash
touch src/cognitus_perception/cognitus_perception/my_node.py
```

2. Implement ROS2 node:
```python
import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__('my_node')
        # Initialize

    def callback(self, msg):
        # Process message
        pass

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

3. Add entry point to `setup.py`:
```python
entry_points={
    'console_scripts': [
        'my_node = cognitus_perception.my_node:main',
    ],
},
```

4. Build workspace:
```bash
cd ~/cognitus
colcon build --packages-select cognitus_perception
source install/setup.bash
```

5. Test node:
```bash
ros2 run cognitus_perception my_node
```

### Testing Changes

1. Unit tests:
```bash
python3 -m pytest src/cognitus_perception/test/
```

2. Integration test:
```bash
ros2 launch cognitus_perception perception.launch.py
# In another terminal:
ros2 topic echo /perception/objects
```

3. System test:
```bash
./scripts/test_conversation.sh
```

## 📚 Code Templates and Examples

### ROS2 Node Template

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class TemplateNode(Node):
    def __init__(self):
        super().__init__('template_node')

        # Parameters
        self.declare_parameter('param_name', 'default_value')
        param = self.get_parameter('param_name').value

        # Publishers
        self.publisher = self.create_publisher(String, '/topic_out', 10)

        # Subscribers
        self.subscription = self.create_subscription(
            String,
            '/topic_in',
            self.callback,
            10
        )

        # Timers
        self.timer = self.create_timer(1.0, self.timer_callback)

        self.get_logger().info('Node initialized')

    def callback(self, msg):
        self.get_logger().info(f'Received: {msg.data}')
        # Process message

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello'
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = TemplateNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Launch File Template

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('package_name')
    config_file = os.path.join(pkg_dir, 'config', 'config.yaml')

    # Declare arguments
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Nodes
    node1 = Node(
        package='package_name',
        executable='node_executable',
        name='node_name',
        parameters=[config_file, {'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        node1
    ])
```

### Configuration File Template

```yaml
# config/template_config.yaml

node_name:
  parameters:
    string_param: "value"
    int_param: 42
    float_param: 3.14
    bool_param: true
    list_param: [1, 2, 3]

  topics:
    input: "/input_topic"
    output: "/output_topic"

  processing:
    rate_hz: 10
    buffer_size: 100
    timeout_sec: 5.0
```

## 🎯 Next Steps

1. **Immediate:** Start with Phase 1 - implement core perception node
2. **Test early:** Create test script after each node implementation
3. **Iterate:** Don't aim for perfection, get basic functionality working first
4. **Document:** Add comments and docstrings as you code
5. **Commit often:** Use git to track changes

## 📞 Support and Resources

### ROS2 Documentation
- ROS2 Foxy: https://docs.ros.org/en/foxy/
- Python Client Library: https://docs.ros2.org/foxy/api/rclpy/

### Model Documentation
- YOLOv8: https://docs.ultralytics.com/
- Whisper: https://github.com/openai/whisper
- Phi-3: https://huggingface.co/microsoft/Phi-3-mini-128k-instruct
- Piper TTS: https://github.com/rhasspy/piper

### Troubleshooting
- Check `logs/` directory for error logs
- Use `ros2 topic list` to verify topic names
- Use `ros2 node list` to check running nodes
- Use `ros2 interface show` to check message types

## ✅ Completion Checklist

### Core Functionality
- [ ] Perception node running and detecting objects
- [ ] Scene graph being published
- [ ] Audio capture working
- [ ] STT converting speech to text
- [ ] LLM generating responses
- [ ] TTS producing speech
- [ ] Memory storing and retrieving data
- [ ] Navigation integration working

### Testing
- [ ] Unit tests passing
- [ ] Integration tests passing
- [ ] End-to-end conversation working
- [ ] Performance meeting targets (<800ms latency)
- [ ] Memory usage within limits (<3GB AI)
- [ ] 7-day continuous operation successful

### Documentation
- [ ] All nodes documented
- [ ] Configuration files documented
- [ ] User guide complete
- [ ] Troubleshooting guide complete

### Deployment
- [ ] Installation script tested
- [ ] System service configured
- [ ] Backup procedures documented
- [ ] Update procedures documented

---

**Current Status:** 🟡 Framework Complete, Implementation In Progress

**Estimated Completion:** 4 weeks with full-time development

**Contact:** See README.md for support information
