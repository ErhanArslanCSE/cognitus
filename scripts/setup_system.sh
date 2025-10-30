#!/bin/bash

##############################################################################
# COGNITUS - System Setup Script
#
# This script performs complete installation and configuration of the
# COGNITUS system on the LIMO Pro robot.
#
# Usage: ./setup_system.sh
##############################################################################

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Logging
LOG_FILE="../logs/setup.log"
mkdir -p ../logs

log() {
    echo -e "${GREEN}[$(date '+%Y-%m-%d %H:%M:%S')]${NC} $1" | tee -a $LOG_FILE
}

error() {
    echo -e "${RED}[ERROR]${NC} $1" | tee -a $LOG_FILE
    exit 1
}

warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1" | tee -a $LOG_FILE
}

##############################################################################
# 1. System Requirements Check
##############################################################################

log "Step 1/10: Checking system requirements..."

# Check if running on Jetson
if ! grep -q "NVIDIA Jetson" /proc/device-tree/model 2>/dev/null && \
   ! command -v jetson_release &> /dev/null; then
    warn "Not running on NVIDIA Jetson. Continuing anyway..."
fi

# Check Ubuntu version
if [ -f /etc/os-release ]; then
    . /etc/os-release
    if [[ "$VERSION_ID" != "20.04" ]]; then
        warn "Expected Ubuntu 20.04, found $VERSION_ID"
    fi
else
    error "Cannot determine OS version"
fi

# Check ROS2 installation
if ! command -v ros2 &> /dev/null; then
    error "ROS2 not found. Please install ROS2 Foxy first."
fi

# Check CUDA
if ! command -v nvcc &> /dev/null; then
    error "CUDA not found. Please install NVIDIA JetPack."
fi

# Check Python version
PYTHON_VERSION=$(python3 --version | cut -d' ' -f2 | cut -d'.' -f1,2)
if [[ "$PYTHON_VERSION" != "3.8" ]]; then
    warn "Expected Python 3.8, found $PYTHON_VERSION"
fi

# Check available disk space (need at least 20GB)
AVAILABLE_SPACE=$(df -BG . | tail -1 | awk '{print $4}' | sed 's/G//')
if [ "$AVAILABLE_SPACE" -lt 20 ]; then
    error "Insufficient disk space. Need 20GB, have ${AVAILABLE_SPACE}GB"
fi

# Check available RAM (need 8GB)
TOTAL_RAM=$(free -g | awk '/^Mem:/{print $2}')
if [ "$TOTAL_RAM" -lt 7 ]; then
    error "Insufficient RAM. Need 8GB, have ${TOTAL_RAM}GB"
fi

log "✓ System requirements check passed"

##############################################################################
# 2. Install System Dependencies
##############################################################################

log "Step 2/10: Installing system dependencies..."

sudo apt-get update
sudo apt-get install -y \
    python3-pip \
    python3-dev \
    python3-opencv \
    libopencv-dev \
    libasound2-dev \
    portaudio19-dev \
    libportaudio2 \
    libportaudiocpp0 \
    ffmpeg \
    git \
    wget \
    curl \
    build-essential \
    cmake

log "✓ System dependencies installed"

##############################################################################
# 3. Install Python Dependencies
##############################################################################

log "Step 3/10: Installing Python dependencies..."

# Upgrade pip
python3 -m pip install --upgrade pip

# Install PyTorch (NVIDIA Jetson optimized version)
if ! python3 -c "import torch" 2>/dev/null; then
    log "Installing PyTorch for Jetson..."
    wget https://nvidia.box.com/shared/static/mp164asf3sceb570wvjsrezk1p4ftj8t.whl -O torch-2.0.0-cp38-cp38-linux_aarch64.whl
    pip3 install torch-2.0.0-cp38-cp38-linux_aarch64.whl
    rm torch-2.0.0-cp38-cp38-linux_aarch64.whl
fi

# Install other dependencies
pip3 install -r ../requirements.txt

log "✓ Python dependencies installed"

##############################################################################
# 4. Install ROS2 Dependencies
##############################################################################

log "Step 4/10: Installing ROS2 dependencies..."

sudo apt-get install -y \
    ros-foxy-vision-msgs \
    ros-foxy-audio-common \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-compressed-image-transport \
    ros-foxy-nav2-bringup \
    ros-foxy-navigation2 \
    ros-foxy-slam-toolbox

log "✓ ROS2 dependencies installed"

##############################################################################
# 5. Download and Prepare AI Models
##############################################################################

log "Step 5/10: Downloading AI models..."

./download_models.sh || error "Model download failed"

log "✓ AI models downloaded"

##############################################################################
# 6. Build ROS2 Workspace
##############################################################################

log "Step 6/10: Building ROS2 workspace..."

cd ..
source /opt/ros/foxy/setup.bash

# Build all packages
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

if [ $? -ne 0 ]; then
    error "ROS2 workspace build failed"
fi

source install/setup.bash

log "✓ ROS2 workspace built successfully"

##############################################################################
# 7. Configure Audio Devices
##############################################################################

log "Step 7/10: Configuring audio devices..."

# List audio devices
echo "Available audio input devices:"
arecord -l

echo "Available audio output devices:"
aplay -l

# Create ALSA config
cat > ~/.asoundrc << 'EOF'
pcm.!default {
    type asym
    playback.pcm "plughw:0,0"
    capture.pcm "plughw:1,0"
}
EOF

log "✓ Audio devices configured"

##############################################################################
# 8. Setup Memory Database
##############################################################################

log "Step 8/10: Setting up memory database..."

# Create database directory
mkdir -p ../memory_db

# Initialize ChromaDB
python3 << 'EOF'
import chromadb
from chromadb.config import Settings

client = chromadb.Client(Settings(
    chroma_db_impl="duckdb+parquet",
    persist_directory="../memory_db"
))

# Create collection
collection = client.create_collection(
    name="robot_experiences",
    metadata={"description": "LIMO Pro episodic memory"}
)

print("Memory database initialized")
EOF

log "✓ Memory database created"

##############################################################################
# 9. Configure Systemd Service (Optional)
##############################################################################

log "Step 9/10: Setting up systemd service..."

read -p "Do you want to enable autostart on boot? (y/n) " -n 1 -r
echo
if [[ $REPLY =~ ^[Yy]$ ]]; then
    sudo tee /etc/systemd/system/limo-ai-brain.service > /dev/null << EOF
[Unit]
Description=COGNITUS
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=$PWD
ExecStart=/bin/bash -c 'source /opt/ros/foxy/setup.bash && source $PWD/install/setup.bash && ros2 launch cognitus_integration full_system.launch.py'
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF

    sudo systemctl daemon-reload
    sudo systemctl enable limo-ai-brain.service
    log "✓ Systemd service enabled"
else
    log "Skipping systemd service setup"
fi

##############################################################################
# 10. Run Validation Tests
##############################################################################

log "Step 10/10: Running validation tests..."

./validate_installation.sh || warn "Some validation tests failed"

##############################################################################
# Installation Complete
##############################################################################

log "================================================"
log "Installation completed successfully!"
log "================================================"
log ""
log "Next steps:"
log "1. Source the workspace:"
log "   source ~/cognitus/install/setup.bash"
log ""
log "2. Launch the system:"
log "   ros2 launch cognitus_integration full_system.launch.py"
log ""
log "3. Test voice interaction:"
log "   Say: 'Hey LIMO, where are you?'"
log ""
log "For troubleshooting, see:"
log "   tail -f logs/system.log"
log "================================================"
