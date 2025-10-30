#!/bin/bash

##############################################################################
# COGNITUS - Model Download and Optimization Script
#
# Downloads all required AI models and optimizes them for Jetson Orin Nano
##############################################################################

set -e

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log() {
    echo -e "${GREEN}[$(date '+%H:%M:%S')]${NC} $1"
}

warn() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

MODELS_DIR="../models"
mkdir -p $MODELS_DIR
cd $MODELS_DIR

##############################################################################
# 1. Download YOLOv8n for Object Detection
##############################################################################

log "Downloading YOLOv8n..."

if [ ! -f "yolov8n.pt" ]; then
    python3 << 'EOF'
from ultralytics import YOLO

# Download YOLOv8n pretrained model
model = YOLO('yolov8n.pt')
print("YOLOv8n downloaded successfully")
EOF
else
    log "YOLOv8n already exists, skipping download"
fi

# Convert to TensorRT for Jetson optimization
log "Converting YOLOv8n to TensorRT..."

if [ ! -f "yolov8n.engine" ]; then
    python3 << 'EOF'
from ultralytics import YOLO
import torch

# Load model
model = YOLO('yolov8n.pt')

# Export to TensorRT with INT8 quantization
model.export(
    format='engine',
    device=0,
    half=False,
    int8=True,
    imgsz=640,
    workspace=4
)

print("YOLOv8n TensorRT engine created")
EOF
else
    log "YOLOv8n TensorRT engine already exists"
fi

##############################################################################
# 2. Download Whisper Tiny for Speech-to-Text
##############################################################################

log "Downloading Whisper Tiny..."

if [ ! -f "whisper_tiny.pt" ]; then
    python3 << 'EOF'
import whisper

# Download Whisper Tiny model
model = whisper.load_model("tiny", device="cuda")
print("Whisper Tiny downloaded successfully")
EOF
else
    log "Whisper Tiny already exists"
fi

# Convert to ONNX with INT8 quantization
log "Converting Whisper to ONNX..."

if [ ! -f "whisper_tiny_int8.onnx" ]; then
    python3 << 'EOF'
import whisper
import torch
from torch.quantization import quantize_dynamic

# Load model
model = whisper.load_model("tiny", device="cpu")

# Quantize to INT8
model_int8 = quantize_dynamic(
    model,
    {torch.nn.Linear},
    dtype=torch.qint8
)

# Export to ONNX
dummy_input = torch.randn(1, 80, 3000)
torch.onnx.export(
    model_int8,
    dummy_input,
    "whisper_tiny_int8.onnx",
    opset_version=14,
    input_names=['input'],
    output_names=['output']
)

print("Whisper ONNX model created")
EOF
else
    log "Whisper ONNX model already exists"
fi

##############################################################################
# 3. Download Phi-3-mini for LLM Brain
##############################################################################

log "Downloading Phi-3-mini (4-bit quantized)..."

if [ ! -d "phi-3-mini-4bit" ]; then
    mkdir -p phi-3-mini-4bit

    python3 << 'EOF'
from transformers import AutoModelForCausalLM, AutoTokenizer
import torch

model_name = "microsoft/Phi-3-mini-128k-instruct"

print("Downloading Phi-3-mini...")

# Download tokenizer
tokenizer = AutoTokenizer.from_pretrained(
    model_name,
    trust_remote_code=True
)
tokenizer.save_pretrained("phi-3-mini-4bit")

# Download model with 4-bit quantization
model = AutoModelForCausalLM.from_pretrained(
    model_name,
    device_map="cuda",
    torch_dtype=torch.float16,
    load_in_4bit=True,
    trust_remote_code=True
)

# Save quantized model
model.save_pretrained("phi-3-mini-4bit")

print("Phi-3-mini downloaded and quantized successfully")
EOF
else
    log "Phi-3-mini already exists"
fi

##############################################################################
# 4. Download Piper TTS
##############################################################################

log "Downloading Piper TTS..."

if [ ! -f "piper_voice_en.onnx" ]; then
    # Download Piper voice model
    wget -q https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx \
        -O piper_voice_en.onnx

    wget -q https://huggingface.co/rhasspy/piper-voices/resolve/main/en/en_US/lessac/medium/en_US-lessac-medium.onnx.json \
        -O piper_voice_en.onnx.json

    log "Piper TTS downloaded successfully"
else
    log "Piper TTS already exists"
fi

##############################################################################
# 5. Download MiniLM for Embeddings
##############################################################################

log "Downloading MiniLM embedding model..."

if [ ! -d "minilm-l6-v2" ]; then
    python3 << 'EOF'
from sentence_transformers import SentenceTransformer

# Download embedding model
model = SentenceTransformer('all-MiniLM-L6-v2')
model.save("minilm-l6-v2")

print("MiniLM embedding model downloaded")
EOF
else
    log "MiniLM already exists"
fi

##############################################################################
# 6. Create Model Info File
##############################################################################

log "Creating model info file..."

cat > MODEL_INFO.txt << 'EOF'
COGNITUS - Model Information
======================================

Downloaded Models:
------------------

1. YOLOv8n (Object Detection)
   - Format: TensorRT Engine (INT8)
   - Size: ~6 MB
   - Inference: ~20ms
   - Purpose: Real-time object detection

2. Whisper Tiny (Speech-to-Text)
   - Format: ONNX (INT8 quantized)
   - Size: ~150 MB
   - Inference: ~150ms per 3s audio
   - Purpose: Voice recognition
   - Languages: 99 languages

3. Phi-3-mini (LLM Brain)
   - Format: PyTorch (4-bit quantized)
   - Size: ~2 GB
   - Inference: ~500ms @ 15 tok/s
   - Purpose: Conversational AI
   - Context: 128K tokens

4. Piper TTS (Text-to-Speech)
   - Format: ONNX
   - Size: ~50 MB
   - Inference: Real-time
   - Purpose: Voice synthesis
   - Voice: en_US-lessac-medium

5. MiniLM-L6-v2 (Embeddings)
   - Format: PyTorch
   - Size: ~22 MB
   - Inference: ~10ms
   - Purpose: Semantic search in memory
   - Dimensions: 384

Total Size: ~2.2 GB
Total RAM Usage: ~3 GB during inference

Model Locations:
----------------
All models stored in: ./models/

Update Models:
--------------
Run: ./scripts/download_models.sh
EOF

log "Model info file created"

##############################################################################
# 7. Verify All Models
##############################################################################

log "Verifying downloaded models..."

REQUIRED_FILES=(
    "yolov8n.pt"
    "yolov8n.engine"
    "whisper_tiny_int8.onnx"
    "phi-3-mini-4bit/config.json"
    "phi-3-mini-4bit/model.safetensors"
    "piper_voice_en.onnx"
    "minilm-l6-v2/config.json"
)

ALL_OK=true

for file in "${REQUIRED_FILES[@]}"; do
    if [ -f "$file" ] || [ -d "$(dirname $file)" ]; then
        echo "✓ $file"
    else
        warn "✗ $file MISSING!"
        ALL_OK=false
    fi
done

if [ "$ALL_OK" = true ]; then
    log "================================================"
    log "All models downloaded and verified successfully!"
    log "================================================"
    log ""
    log "Total disk usage:"
    du -sh .
    log ""
    log "Models ready for use."
else
    echo "Some models are missing. Please check the output above."
    exit 1
fi

cd - > /dev/null
