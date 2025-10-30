# COGNITUS - Deployment Guide

## Overview

COGNITUS supports two deployment modes:
1. **Development (Docker)** - Develop and test without hardware
2. **Production (LIMO Pro)** - Deploy on real robot

---

## Development Deployment (Docker)

### Prerequisites
- Docker and Docker Compose installed
- X11 server for GUI (Linux/Mac) or VcXsrv (Windows)
- 8GB RAM minimum
- 20GB disk space

### Quick Start

```bash
# Navigate to project
cd cognitus/

# Start development environment
make dev

# OR start with simulation
make sim
```

### What Gets Installed
- ROS2 Foxy environment (matches LIMO Pro)
- Gazebo simulation
- All COGNITUS packages
- Development tools

### Testing

```bash
# Inside container
ros2 launch cognitus_integration demo.launch.py

# OR with simulation
ros2 launch cognitus_integration simulation.launch.py

# Check nodes
ros2 node list

# Check topics
ros2 topic list
```

### Development Workflow

1. **Edit code on your machine**
   - `src/` is mounted, changes apply instantly
   - No rebuild needed for Python code

2. **Rebuild for C++ changes**
   ```bash
   # Inside container
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Test in simulation**
   ```bash
   make sim
   ```

4. **Ready? Deploy to robot!**
   ```bash
   scp -r cognitus/ agilex@<robot-ip>:~/
   ```

---

## Production Deployment (LIMO Pro Robot)

### Quick Start Guide

This guide will walk you through deploying the complete AI Brain system on your LIMO Pro robot.

## Prerequisites

### Hardware Checklist
- ✅ LIMO Pro robot with Jetson Orin Nano
- ✅ EAI T-mini Pro LIDAR installed
- ✅ Orbbec DaBai RGB-D camera installed
- ✅ USB microphone array connected
- ✅ Speaker/audio output device
- ✅ WiFi connection configured
- ✅ Robot fully charged (>80% battery recommended)

### Software Checklist (Pre-installed on LIMO Pro)
- ✅ Ubuntu 20.04 LTS
- ✅ ROS2 Foxy
- ✅ NVIDIA JetPack 5.x
- ✅ LIMO base packages (limo_base, limo_bringup, etc.)

## Step-by-Step Deployment

### Step 1: Transfer Project Files

From your development computer:

```bash
# Compress the project
cd /path/to/HumanRobotProject
tar -czf cognitus.tar.gz cognitus/

# Transfer to robot
scp cognitus.tar.gz agilex@<robot-ip>:~/

# SSH into robot
ssh agilex@<robot-ip>

# Extract
cd ~
tar -xzf cognitus.tar.gz
cd cognitus
```

**Note:** Replace `<robot-ip>` with your robot's IP address. You can find it on the robot's rear touchscreen or by running `hostname -I` on the robot.

### Step 2: Run Automated Setup

```bash
# Make scripts executable
chmod +x scripts/*.sh

# Run complete system setup
./scripts/setup_system.sh
```

**What this does:**
1. Checks system requirements (RAM, disk space, CUDA, ROS2)
2. Installs Python dependencies
3. Installs ROS2 packages
4. Downloads AI models (~2.2 GB)
5. Optimizes models for Jetson (TensorRT, ONNX, 4-bit)
6. Builds ROS2 workspace
7. Configures audio devices
8. Initializes memory database
9. (Optional) Sets up autostart service
10. Runs validation tests

**Expected time:** 30-60 minutes (mostly model downloads)

**Note:** You will be prompted:
- "Do you want to enable autostart on boot?" - Answer 'y' if you want the system to start automatically when robot boots

### Step 3: Source the Workspace

```bash
# Add to your .bashrc for convenience
echo "source ~/cognitus/install/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Or source manually each time
source ~/cognitus/install/setup.bash
```

### Step 4: Verify Installation

```bash
# Check that all models were downloaded
ls -lh models/

# Should see:
# - yolov8n.pt, yolov8n.engine (~6 MB)
# - whisper_tiny_int8.onnx (~150 MB)
# - phi-3-mini-4bit/ (~2 GB)
# - piper_voice_en.onnx (~50 MB)
# - minilm-l6-v2/ (~22 MB)

# Check ROS2 packages
ros2 pkg list | grep limo

# Should see:
# - cognitus_perception
# - cognitus_voice
# - cognitus_cognition
# - cognitus_memory
# - cognitus_behaviors
# - cognitus_integration
```

### Step 5: Test Individual Components

Before launching the full system, test each component:

#### Test 1: LIMO Base System (Existing)
```bash
# In terminal 1 - Start robot base
ros2 launch limo_bringup limo_start.launch.py

# In terminal 2 - Check sensors
ros2 topic list | grep camera
ros2 topic list | grep scan
ros2 topic hz /camera/color/image_raw  # Should see ~30 Hz
```

#### Test 2: Perception System
```bash
# In terminal 1 - Start perception
ros2 launch cognitus_perception perception.launch.py

# In terminal 2 - Check detections
ros2 topic echo /perception/objects

# In terminal 3 - Visualize (if display available)
ros2 run cognitus_perception visualize_detections
```

#### Test 3: Voice System
```bash
# Test microphone
arecord -d 5 test.wav
aplay test.wav

# Start voice system
ros2 launch cognitus_voice voice.launch.py

# In another terminal - speak and check
ros2 topic echo /voice/stt_output
# Say "hello" - should see text output
```

#### Test 4: AI Brain
```bash
# Start cognition system
ros2 launch cognitus_cognition brain.launch.py

# In another terminal - send test query
ros2 topic pub /voice/stt_output std_msgs/String "data: 'Hello robot'"

# Check response
ros2 topic echo /voice/tts_input
# Should see generated response
```

### Step 6: Launch Complete System

Once individual tests pass:

```bash
# Option A: Full system launch
ros2 launch cognitus_integration full_system.launch.py

# Option B: With visualization (if display available)
ros2 launch cognitus_integration full_system.launch.py use_rviz:=true

# Option C: Auto-start on boot (if configured in setup)
sudo systemctl start limo-ai-brain.service
```

**What's running:**
- LIMO base drivers (motors, sensors)
- Perception node (YOLOv8 object detection)
- Scene graph builder
- Memory manager
- Audio capture
- Speech-to-text (Whisper)
- LLM brain (Phi-3)
- Text-to-speech (Piper)
- Audio playback
- Behavior controller
- Navigation2 stack

### Step 7: First Interaction

```bash
# Say wake word
"Hey LIMO"

# Wait for acknowledgment (beep or "Yes?")

# Ask a question
"What do you see?"

# Expected response
"I see a table, chair, and a cup on the table."

# Try location query
"Where are you?"

# Expected response
"I'm in the living room near the window."

# Try memory
"Remember this location as home base"

# Expected response
"I've remembered this location as home base."
```

## Configuration and Customization

### Changing Wake Word

Edit `config/voice_config.yaml`:
```yaml
wake_words: ["hey limo", "robot", "limo"]  # Add your own
wake_word_sensitivity: 0.7  # Lower = more sensitive (0.0-1.0)
```

### Changing Voice Language

For Turkish:
```yaml
stt_language: "tr"  # Turkish
tts_voice: "tr_TR-dfki-medium"  # Turkish voice
```

Download Turkish TTS voice:
```bash
cd models/
wget https://huggingface.co/rhasspy/piper-voices/resolve/main/tr/tr_TR/dfki/medium/tr_TR-dfki-medium.onnx
```

### Adjusting Memory Retention

Edit `config/memory_config.yaml`:
```yaml
working_memory:
  retention_seconds: 600  # 10 minutes

episodic_memory:
  retention_days: 7
  max_episodes: 50000

long_term_memory:
  compression_ratio: 100
  retention_days: 90
```

### Adjusting Response Style

Edit `config/cognition_config.yaml`:
```yaml
llm:
  temperature: 0.7  # Higher = more creative (0.0-2.0)
  max_new_tokens: 150  # Shorter = faster, more concise

personality:
  style: "friendly"  # friendly, professional, concise, verbose
  use_names: true  # Address user by name if learned
```

### Power Management

Edit `config/power_config.yaml`:
```yaml
battery_thresholds:
  full_performance: 50  # % battery
  power_saving: 20
  emergency: 10

adaptive_duty_cycle:
  enabled: true
  quiet_period_detection: true
  min_perception_hz: 1
  max_perception_hz: 10
```

## Monitoring and Maintenance

### Real-time System Monitoring

```bash
# Monitor all system stats
./scripts/monitor_system.sh

# Shows:
# - CPU/GPU usage
# - Memory usage
# - Active topics
# - Node status
# - Recent logs
```

### View Logs

```bash
# All logs in one place
tail -f logs/system.log

# Specific components
tail -f logs/perception.log
tail -f logs/brain.log
tail -f logs/memory.log
```

### Check Performance

```bash
# Run benchmark
./scripts/benchmark.sh

# Measures:
# - STT latency
# - LLM inference time
# - TTS generation speed
# - Memory query speed
# - GPU/CPU usage
# - Total end-to-end latency

# Expected results:
# ✓ STT: <200ms
# ✓ LLM: <500ms
# ✓ TTS: <100ms first chunk
# ✓ Total: <800ms
# ✓ GPU RAM: <3GB
```

### Database Maintenance

```bash
# Check memory database size
du -sh memory_db/

# Backup memory
./scripts/backup_memory.sh
# Creates: backups/memory_backup_YYYYMMDD.tar.gz

# Restore from backup
tar -xzf backups/memory_backup_20241029.tar.gz -C memory_db/

# Clean old memories
./scripts/clean_memory.sh --older-than 30d

# Optimize database
./scripts/optimize_memory.sh
```

## Troubleshooting

### Problem: Robot doesn't respond to wake word

**Check:**
```bash
# 1. Is microphone detected?
arecord -l

# 2. Is audio being captured?
ros2 topic hz /audio/input

# 3. Check STT node
ros2 node info /stt_node

# 4. Test microphone directly
arecord -d 3 test.wav && aplay test.wav
```

**Solutions:**
```bash
# Reconfigure audio
./scripts/configure_audio.sh

# Check wake word settings
nano config/voice_config.yaml
# Lower sensitivity: 0.7 → 0.5

# Restart voice system
ros2 launch cognitus_voice voice.launch.py
```

### Problem: Slow or no response from LLM

**Check:**
```bash
# 1. GPU memory
nvidia-smi

# 2. Is brain node running?
ros2 node list | grep brain

# 3. Check for errors
tail -f logs/brain.log
```

**Solutions:**
```bash
# Free GPU memory
ros2 node kill /brain_node
pkill -9 python3
ros2 launch cognitus_cognition brain.launch.py

# Reduce context length
nano config/cognition_config.yaml
# max_context_length: 2048 → 1024
```

### Problem: Poor object detection

**Check:**
```bash
# 1. Camera working?
ros2 topic hz /camera/color/image_raw

# 2. Check lighting
# YOLO works best in good lighting

# 3. Visualize detections
ros2 run cognitus_perception visualize_detections
```

**Solutions:**
```bash
# Lower confidence threshold
nano config/perception_config.yaml
# confidence_threshold: 0.5 → 0.3

# Improve lighting in environment
```

### Problem: System crashes or freezes

**Check:**
```bash
# 1. Memory usage
free -h

# 2. Check logs
dmesg | tail
journalctl -xe

# 3. Temperature
tegrastats
```

**Solutions:**
```bash
# Restart system
sudo systemctl restart limo-ai-brain.service

# Or full reboot
sudo reboot

# If persists, reduce memory usage:
# - Lower perception FPS
# - Reduce LLM context
# - Disable unused features
```

### Problem: Navigation not working

**Check:**
```bash
# 1. Is Navigation2 running?
ros2 node list | grep nav2

# 2. Is LIDAR working?
ros2 topic hz /scan

# 3. Is map being built?
ros2 topic hz /map
```

**Solutions:**
```bash
# Restart navigation
ros2 launch limo_bringup limo_start.launch.py

# Build new map
ros2 launch limo_bringup slam.launch.py
```

## Performance Optimization

### Reduce GPU Memory Usage

1. Lower YOLOv8 input size:
```yaml
# config/perception_config.yaml
input_size: [640, 480] → [416, 416]
```

2. Reduce LLM context:
```yaml
# config/cognition_config.yaml
max_context_length: 2048 → 1024
```

3. Unload unused models:
```bash
# Disable TTS for text-only responses
nano config/voice_config.yaml
# tts_enabled: false
```

### Improve Battery Life

1. Enable adaptive duty cycling:
```yaml
# config/power_config.yaml
adaptive_duty_cycle:
  enabled: true
```

2. Reduce perception rate:
```yaml
# config/perception_config.yaml
fps_target: 10 → 5
```

3. Set power-saving mode:
```bash
sudo nvpmodel -m 1  # Power-saving mode
sudo jetson_clocks --restore  # Restore default clocks
```

### Speed Up Response Time

1. Pre-load models at startup (already done)

2. Use response caching:
```yaml
# config/cognition_config.yaml
response_cache:
  enabled: true
  max_size: 100
```

3. Reduce max tokens for shorter responses:
```yaml
max_new_tokens: 150 → 75
```

## Updating the System

### Update AI Models

```bash
./scripts/update_models.sh
```

### Update Software

```bash
cd ~/cognitus
git pull origin main  # If using git
./scripts/rebuild.sh
```

### Update Configuration

```bash
# Edit configuration files
nano config/voice_config.yaml

# Restart affected components
ros2 launch cognitus_voice voice.launch.py
```

## Advanced Features

### Adding Custom Behaviors

See `docs/BEHAVIORS.md` for detailed guide on creating custom behaviors.

Example:
```python
# src/cognitus_behaviors/cognitus_behaviors/patrol_behavior.py
from cognitus_behaviors.base_behavior import BaseBehavior

class PatrolBehavior(BaseBehavior):
    def execute(self):
        waypoints = self.get_patrol_waypoints()
        for waypoint in waypoints:
            self.navigate_to(waypoint)
            self.scan_area()
        return "Patrol complete"
```

### Multi-Robot Coordination

See `docs/MULTI_ROBOT.md` for setting up multiple robots.

### Cloud Integration (Optional)

See `docs/CLOUD.md` for connecting to cloud services for:
- Remote monitoring
- Model updates
- Complex reasoning offload
- Memory backup

## Support and Community

### Getting Help

1. Check documentation:
   - `ARCHITECTURE.md` - System design
   - `PROJECT_STATUS.md` - Implementation guide
   - `README.md` - Quick start

2. Review logs:
   - `logs/system.log` - All events
   - Component-specific logs

3. GitHub Issues:
   - Report bugs
   - Request features
   - Ask questions

### Contributing

We welcome contributions!

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Test on real hardware
5. Submit pull request

See `CONTRIBUTING.md` for detailed guidelines.

## Safety and Best Practices

### Safety Guidelines

1. **Always supervise** the robot during initial testing
2. **Clear the area** before autonomous navigation
3. **Emergency stop** - Press red button on robot
4. **Battery safety** - Don't over-discharge (<10%)
5. **Charging** - Use only official charger

### Best Practices

1. **Regular backups** - Backup memory weekly
2. **Log monitoring** - Check logs daily for errors
3. **Performance checks** - Run benchmark weekly
4. **Software updates** - Update monthly
5. **Clean sensors** - Clean camera and LIDAR weekly

### Privacy Considerations

1. **Local processing** - All AI runs on device
2. **No cloud** - No data sent to internet (unless enabled)
3. **Memory access** - Only robot has access to memories
4. **Audio recording** - Only when wake word detected
5. **Data deletion** - Use clean_memory.sh to delete old data

## Next Steps

Now that your system is running:

1. **Learn commands** - Try different voice commands
2. **Teach routines** - Let robot observe your routines
3. **Add behaviors** - Implement custom behaviors
4. **Optimize** - Tune for your environment
5. **Contribute** - Share improvements with community

---

**Congratulations!** Your COGNITUS is now operational.

For questions or support, see README.md contact information.

**Happy robotics!** 🤖
