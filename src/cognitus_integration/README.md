# cognitus_integration

**System Integration and Launch Management for COGNITUS**

Coordinates all modules and provides launch files for different environments.

---

## Overview

This package manages system startup and coordination. It provides launch files for:
- Demo mode (testing)
- Simulation mode (Gazebo)
- LIMO Pro mode (real hardware)

**Key feature:** Same launch files work everywhere with auto-detection.

---

## Launch Files

### 1. demo.launch.py
**Purpose:** Quick system test

**What it launches:**
- All 9 COGNITUS nodes
- No Gazebo, no LIMO drivers
- Pure software test

**Usage:**
```bash
ros2 launch cognitus_integration demo.launch.py
```

**When to use:**
- Testing after code changes
- Development without simulation
- Verifying all nodes start

**Nodes (9 total):**
1. perception_node
2. scene_graph_builder
3. anomaly_detector
4. memory_manager_node
5. stt_node
6. tts_node
7. audio_monitor
8. brain_node (COGNITUS brain)
9. behavior_controller_node

### 2. simulation.launch.py
**Purpose:** Full system in Gazebo

**What it launches:**
- Gazebo simulator
- All 9 COGNITUS nodes (use_sim_time=true)
- Simulated sensors

**Usage:**
```bash
ros2 launch cognitus_integration simulation.launch.py
```

**When to use:**
- Algorithm testing without hardware
- Visualization of behaviors
- Safe development environment

**Features:**
- Gazebo 3D visualization
- Simulated LIDAR, camera, IMU
- Navigation testing
- No risk to real robot

### 3. limo_pro.launch.py
**Purpose:** Production deployment on LIMO Pro

**What it launches:**
- LIMO base drivers (limo_bringup if available)
- All 9 COGNITUS nodes (use_sim_time=false)
- Real hardware integration

**Usage:**
```bash
ros2 launch cognitus_integration limo_pro.launch.py
```

**When to use:**
- Deployment on actual LIMO Pro robot
- Real-world testing
- Production operation

**Features:**
- Auto-detects limo_bringup (graceful if not found)
- Uses real sensors (camera, LIDAR, IMU)
- Navigation2 integration
- Real motor control

---

## System Architecture

### Node Organization

```
COGNITUS System (9 nodes)
├── PERCEPTION (3 nodes)
│   ├── perception_node           (Object detection)
│   ├── scene_graph_builder       (Spatial relations)
│   └── anomaly_detector           (Unusual detection)
│
├── MEMORY (1 node)
│   └── memory_manager_node        (3-tier memory)
│
├── VOICE (3 nodes)
│   ├── stt_node                   (Speech-to-text)
│   ├── tts_node                   (Text-to-speech)
│   └── audio_monitor              (Sound monitoring)
│
├── COGNITION (1 node)
│   └── brain_node                 (Central intelligence)
│
└── BEHAVIORS (1 node)
    └── behavior_controller_node   (Action execution)
```

### Data Flow

```
Camera/LIDAR → PERCEPTION → Scene description
                                ↓
Microphone → VOICE/STT → Commands
                                ↓
                            BRAIN (decides)
                                ↓
                    ┌───────────┴───────────┐
                    ↓                       ↓
            Memory queries              Commands
                    ↓                       ↓
                MEMORY                  BEHAVIORS
                    ↓                       ↓
              Memories                  Motors
```

---

## Environment Detection

### Automatic via start.sh

The `start.sh` script automatically detects environment:

```bash
if [ -f "/.dockerenv" ]; then
    # Docker → launch simulation.launch.py
    ros2 launch cognitus_integration simulation.launch.py
else
    # LIMO Pro → launch limo_pro.launch.py
    ros2 launch cognitus_integration limo_pro.launch.py
fi
```

### Manual launch

**Docker:**
```bash
ros2 launch cognitus_integration simulation.launch.py
```

**LIMO Pro:**
```bash
ros2 launch cognitus_integration limo_pro.launch.py
```

**Testing:**
```bash
ros2 launch cognitus_integration demo.launch.py
```

---

## Dependencies

### ROS2 Packages

**Required (always):**
- `launch` - Launch system
- `launch_ros` - ROS launch actions
- `ament_index_python` - Package discovery

**Required (cognitus packages):**
- `cognitus_perception`
- `cognitus_memory`
- `cognitus_voice`
- `cognitus_cognition`
- `cognitus_behaviors`

**Optional (LIMO Pro only):**
- `limo_bringup` - LIMO drivers (pre-installed on robot)
- `nav2_bringup` - Navigation2 (pre-installed on robot)

**Optional (simulation only):**
- `gazebo_ros` - Gazebo integration

### Graceful Degradation

**If limo_bringup not found:**
- Logs warning
- Continues without LIMO drivers
- COGNITUS nodes still start
- Good for Docker development

**If Gazebo not available:**
- simulation.launch.py fails gracefully
- Use demo.launch.py instead

---

## Usage

### Quick Start

**Development (Docker):**
```bash
cd cognitus/
make dev
# Inside container:
ros2 launch cognitus_integration demo.launch.py
```

**Simulation:**
```bash
make sim
# Automatically runs simulation.launch.py
```

**Production (LIMO Pro):**
```bash
make start
# Automatically runs limo_pro.launch.py
```

### Monitor System

**Check all nodes running:**
```bash
ros2 node list
# Should see 9 nodes + LIMO drivers
```

**Check topics:**
```bash
ros2 topic list
# Should see ~30+ topics
```

**Monitor brain status:**
```bash
ros2 topic echo /brain/status
```

**Test voice command:**
```bash
ros2 topic pub /voice/command std_msgs/String "data: 'What do you see?'"
# Check response:
ros2 topic echo /brain/response
```

---

## Troubleshooting

### Nodes fail to start

**Check build:**
```bash
colcon build --symlink-install
source install/setup.bash
```

**Check dependencies:**
```bash
rosdep install --from-paths src --ignore-src -y
```

### LIMO drivers not found

**Expected in Docker** - Not an error

**On LIMO Pro:**
```bash
# Check if limo_bringup exists
ros2 pkg list | grep limo

# If missing, install LIMO packages
# (should be pre-installed)
```

### Gazebo won't start

**Check display:**
```bash
echo $DISPLAY
# Should show :0 or similar

# If empty:
export DISPLAY=:0
```

**Enable X11:**
```bash
xhost +local:docker
```

---

## Launch File Structure

### Common Pattern

All launch files follow:
```python
1. LogInfo header
2. Start external systems (Gazebo/LIMO)
3. Launch perception nodes
4. Launch memory node
5. Launch voice nodes
6. Launch brain node
7. Launch behaviors node
8. LogInfo completion message
```

### Parameters

**use_sim_time:**
- `true` - Simulation (uses Gazebo time)
- `false` - Real robot (uses system time)

**All other parameters:**
- Loaded from individual package config files
- Can override per-node if needed

---

## Package.xml

Dependencies on all COGNITUS packages:
```xml
<exec_depend>cognitus_perception</exec_depend>
<exec_depend>cognitus_memory</exec_depend>
<exec_depend>cognitus_voice</exec_depend>
<exec_depend>cognitus_cognition</exec_depend>
<exec_depend>cognitus_behaviors</exec_depend>
```

**This ensures all packages are built before integration.**

---

## Current Status

✅ **COMPLETE AND FUNCTIONAL**

- ✅ 3 launch files (demo, sim, limo_pro)
- ✅ All 9 nodes included
- ✅ Environment detection
- ✅ LIMO driver integration
- ✅ Gazebo support
- ✅ Parameter passing
- ✅ Error handling

**Ready for:** Full system testing

---

## Testing Checklist

### Demo Mode
```bash
ros2 launch cognitus_integration demo.launch.py

# Verify:
□ 9 nodes running
□ No error messages
□ Topics being published
```

### Simulation Mode
```bash
ros2 launch cognitus_integration simulation.launch.py

# Verify:
□ Gazebo opens
□ 9 nodes running
□ Simulated topics active
```

### LIMO Pro Mode
```bash
# On robot:
ros2 launch cognitus_integration limo_pro.launch.py

# Verify:
□ LIMO drivers start
□ 9 COGNITUS nodes running
□ Real sensor topics active
□ Camera images streaming
```

---

## Integration Points

### With LIMO Pro

**Pre-installed packages used:**
- `limo_base` - Hardware drivers
- `limo_bringup` - Launch files
- `Navigation2` - Path planning
- `SLAM Toolbox` - Mapping

**Topics from LIMO:**
- `/camera/color/image_raw`
- `/camera/depth/image_raw`
- `/scan` (LIDAR)
- `/imu`
- `/odom`

**Topics to LIMO:**
- `/cmd_vel` (motor control)
- `navigate_to_pose` (Navigation2 action)

### With Gazebo

**Simulated:**
- Robot model (URDF)
- Sensors (camera, LIDAR, IMU)
- Physics simulation
- Visualization

**Same topics as real robot** - code doesn't change!

---

## Notes

**Launch order doesn't matter:**
- ROS2 handles node dependencies
- Nodes wait for topics
- Automatic reconnection

**Shutdown:**
- Ctrl+C cleanly stops all nodes
- Behaviors stops robot first
- Safe shutdown

**Restart:**
- Can restart individual nodes
- Or restart entire system
- State preserved in memory (episodic DB)

---

**Module maintained by COGNITUS Team**
**Role:** System coordinator
**Status:** Production ready
