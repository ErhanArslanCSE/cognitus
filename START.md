# 🚀 COGNITUS - Quick Start

**Like NextJS - Super Simple!**

---

## ⚡ Development (Docker)

```bash
# Development environment (no GUI)
make dev

# OR with Gazebo simulation
make sim

# OR manually
docker-compose up                    # Development
docker-compose up simulation         # Simulation
```

**That's it!** Your development environment is ready.

### What You Can Do:
- 🔨 **make dev** - Interactive terminal, edit code, run commands
- 🌍 **make sim** - Full Gazebo simulation with LIMO robot
- 🧪 Test individual nodes before robot deployment

---

## 🤖 Production (LIMO Pro)

```bash
# Copy to robot
scp -r cognitus/ agilex@<robot-ip>:~/

# SSH to robot
ssh agilex@<robot-ip>
cd cognitus

# Start!
make start

# OR
./start.sh
```

**Done!** Robot is running.

---

## 🎯 What You Get

### On Docker (Development):
- ✅ ROS2 Foxy environment
- ✅ Gazebo simulation
- ✅ Live code editing (like hot reload)
- ✅ All development tools

### On LIMO Pro (Production):
- ✅ Same code, real hardware
- ✅ Auto-detects LIMO Pro
- ✅ Uses pre-installed drivers
- ✅ Ready to deploy

---

## 📦 What's Pre-installed on LIMO Pro?

LIMO Pro comes with:
- ✅ Ubuntu 20.04
- ✅ ROS2 Foxy + ROS1 Noetic
- ✅ `limo_base` - Robot driver
- ✅ `limo_bringup` - Launch files
- ✅ `limo_description` - URDF models
- ✅ Navigation2, SLAM packages
- ✅ Jetson Orin Nano setup

**COGNITUS uses these!** No need to reinstall.

---

## 🛠️ Available Commands

```bash
make dev      # Start Docker development
make sim      # Start Gazebo simulation
make start    # Start on LIMO Pro
make stop     # Stop Docker
make clean    # Clean everything
```

---

## 📁 Project Structure

```
cognitus/
├── start.sh              ← Main startup script
├── docker-compose.yml    ← Docker config
├── Makefile              ← Simple commands
└── src/                  ← Your ROS2 packages
    ├── cognitus_perception/
    ├── cognitus_voice/
    ├── cognitus_cognition/
    ├── cognitus_memory/
    ├── cognitus_behaviors/
    └── cognitus_integration/
```

---

## 🔄 Development Workflow

### 1. Develop in Docker:
```bash
make dev
# Edit code in src/
# Changes apply instantly (volume mounted)
```

### 2. Test on LIMO Pro:
```bash
# Copy entire cognitus folder
scp -r cognitus/ agilex@robot-ip:~/

# Run on robot
ssh agilex@robot-ip
cd cognitus && make start
```

### 3. Same Code, Different Environments:
- Docker: Uses simulation
- LIMO Pro: Uses real hardware
- **Auto-detected!** No config changes needed.

---

## 🌍 Simulation Details

### Gazebo Simulation

**What Gets Simulated:**
- LIMO robot model (4-wheel mobile robot)
- RGB-D Camera (640x480, 30 FPS)
- 2D LIDAR (360°, 12m range)
- IMU (orientation, acceleration)
- Odometry (position, velocity)

**Launch Commands:**
```bash
# Inside Docker container
ros2 launch cognitus_integration simulation.launch.py

# Check simulated topics
ros2 topic list
# Expected: /camera/*, /scan, /imu, /odom
```

**Visualization:**
- Gazebo opens automatically with LIMO robot
- Use RViz for sensor visualization:
  ```bash
  rviz2
  ```

### Testing Without Hardware

**Perfect for:**
- ✅ Algorithm development
- ✅ Navigation testing
- ✅ Sensor processing
- ✅ No risk to real robot

**Workflow:**
1. Write code in `src/`
2. Test in simulation (`make sim`)
3. If works → Deploy to LIMO Pro
4. Same code, zero changes needed!

---

## 💡 How It Works

`start.sh` automatically detects where it's running:

```bash
# Detects environment
if [ -f "/.dockerenv" ]; then
    # Docker → Launch simulation
    ros2 launch cognitus_integration simulation.launch.py
else
    # LIMO Pro → Launch with hardware
    ros2 launch cognitus_integration limo_pro.launch.py
fi
```

---

## 📖 Next Steps

1. **[README.md](README.md)** - Full documentation
2. **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** - System design
3. **[docs/IMPLEMENTATION.md](docs/IMPLEMENTATION.md)** - Start coding

---

**Just run `make dev` and start building!** 🎉
