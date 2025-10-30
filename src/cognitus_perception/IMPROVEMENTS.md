# cognitus_perception - Improvement Suggestions

This document outlines potential enhancements for the perception module.

---

## High Priority Improvements

### 1. Model Optimization for Jetson

**Current:** YOLOv8n runs in PyTorch (slower)

**Improvement:**
- Convert YOLOv8n to TensorRT INT8
- Expected speedup: 3-5x
- Lower latency: ~5ms per frame
- Lower power consumption

**Implementation:**
```python
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='engine', device=0, half=True, int8=True)
# Load: YOLO('yolov8n.engine')
```

**Benefits:**
- Real-time 30 FPS detection possible
- Lower GPU memory usage
- Better battery life

---

### 2. Object Tracking Across Frames

**Current:** Each frame is independent

**Improvement:**
- Add object ID tracking across frames
- Use simple centroid tracking or SORT algorithm
- Maintain object persistence

**Benefits:**
- Stable object IDs for memory
- Better spatial relation computation
- Track object movements over time
- Detect "disappeared" objects

**Implementation:**
```python
# Add to perception_node.py
from collections import deque

class ObjectTracker:
    def __init__(self):
        self.tracks = {}  # id -> object history
        self.next_id = 0

    def update(self, detections):
        # Match detections to existing tracks
        # Assign IDs to new detections
        # Return tracked objects with stable IDs
```

---

### 3. Semantic Segmentation

**Current:** Only bounding boxes

**Improvement:**
- Add instance segmentation (YOLOv8-seg)
- Get precise object boundaries
- Better spatial relation accuracy

**Benefits:**
- Accurate "on" relation (segmentation overlap)
- Better 3D position (use segment center of mass)
- Improved scene understanding

---

### 4. Enhanced Spatial Relations

**Current:** Basic relations (above, below, left, right, near)

**Improvement:**
- Add: `on`, `inside`, `behind`, `in_front_of`
- Use 3D geometry and object sizes
- Support grouping (e.g., "books on shelf")

**Implementation:**
```python
def detect_on_relation(obj1, obj2):
    # Check if obj1 centroid is within obj2 horizontal bounds
    # AND obj1 is slightly above obj2
    # Requires better 3D bounding boxes
```

---

### 5. Multi-Camera Support

**Current:** Single RGB-D camera

**Improvement:**
- Support multiple cameras for wider FOV
- Fuse detections from different viewpoints
- Better 3D accuracy

**Benefits:**
- 360° perception
- Reduced occlusions
- More robust detection

---

## Medium Priority Improvements

### 6. Advanced Anomaly Detection

**Current:** Simple frequency-based detection

**Improvements:**
- **Temporal anomalies:** Object appeared/disappeared suddenly
- **Spatial anomalies:** Object in unusual location
- **Contextual anomalies:** Unusual object combinations
- **Motion anomalies:** Unexpected movement patterns

**Implementation:**
- Add temporal window analysis
- Learn location-object associations
- Detect co-occurrence anomalies

---

### 7. Scene Classification

**Current:** No scene-level classification

**Improvement:**
- Classify room type (kitchen, bedroom, living room)
- Detect scene context (eating, working, sleeping)
- Provide context to brain

**Benefits:**
- Better anomaly detection (kettle in bedroom = unusual)
- Contextual behavior adaptation
- Room-specific navigation

---

### 8. Depth Processing Improvements

**Current:** Simple center-point depth lookup

**Improvements:**
- Use median depth within bounding box (more robust)
- Filter outliers (statistical filtering)
- Temporal smoothing for stable 3D positions
- Handle depth holes (interpolation)

**Implementation:**
```python
def robust_depth_estimation(bbox, depth_image):
    # Extract depth patch
    patch = depth_image[y1:y2, x1:x2]

    # Remove zeros and outliers
    valid_depths = patch[patch > 0]

    # Use median instead of center point
    return np.median(valid_depths)
```

---

### 9. Calibration Support

**Current:** Hardcoded camera intrinsics

**Improvement:**
- Load calibration from file or camera_info topic
- Subscribe to `/camera/color/camera_info`
- Auto-detect camera parameters

**Benefits:**
- Accurate 3D positions
- Support different cameras
- Better portability

---

### 10. Performance Profiling

**Current:** Basic frame count logging

**Improvement:**
- Detailed timing for each stage:
  - Image conversion
  - YOLOv8 inference
  - 3D computation
  - Relation extraction
- Publish performance metrics
- Adaptive FPS based on CPU/GPU load

**Benefits:**
- Identify bottlenecks
- Optimize critical paths
- Dynamic resource management

---

## Low Priority / Future Enhancements

### 11. Point Cloud Generation

**Current:** Per-object 3D positions only

**Improvement:**
- Generate full point cloud from depth
- Publish to `/perception/point_cloud`
- Enable advanced 3D processing

**Use cases:**
- Navigation obstacle detection
- 3D mapping
- Manipulation planning

---

### 12. Visual Features for Memory

**Current:** Only semantic labels

**Improvement:**
- Extract visual features (CNN embeddings)
- Enable visual similarity search
- Re-identify previously seen objects

**Benefits:**
- "Have I seen this cup before?"
- Visual memory retrieval
- Object re-identification across time

---

### 13. Active Perception

**Current:** Passive observation

**Improvement:**
- Request camera movement for better view
- Focus attention on regions of interest
- Zoom/pan control for details

**Benefits:**
- Better object recognition
- Resolve ambiguities
- Proactive information gathering

---

### 14. Occlusion Handling

**Current:** Only visible objects detected

**Improvement:**
- Infer occluded objects from partial visibility
- Track objects when temporarily occluded
- Predict hidden objects

**Benefits:**
- More complete scene understanding
- Better object persistence
- Reduced false "object disappeared" events

---

### 15. Lighting Adaptation

**Current:** No lighting compensation

**Improvement:**
- Detect low-light conditions
- Auto-adjust detection thresholds
- Use different models for different lighting

**Benefits:**
- Robust performance in various conditions
- Better night-time operation
- Fewer false detections

---

## Research Opportunities

### Integration with Academic Papers

This module directly supports several research contributions:

**Paper 4: Spatial Language Grounding**
- Enhance natural language generation from spatial relations
- Add landmark-based descriptions
- Improve uncertainty expressions

**Paper 3: Abductive Reasoning**
- Use observed objects as evidence
- Infer hidden states (e.g., "raining" from wet umbrella)
- Build evidence-state correlations

**Paper 9: Curiosity-Driven Exploration**
- Track unexplored areas
- Quantify visual uncertainty
- Generate exploration goals

---

## Implementation Priority

**Immediate (Week 1-2):**
1. Model optimization (TensorRT)
2. Object tracking
3. Calibration support

**Short-term (Month 1):**
4. Enhanced spatial relations
5. Advanced anomaly detection
6. Performance profiling

**Medium-term (Month 2-3):**
7. Semantic segmentation
8. Scene classification
9. Visual features

**Long-term (Month 3+):**
10. Multi-camera support
11. Active perception
12. Point cloud generation

---

## Testing Recommendations

### Unit Tests
- Camera data conversion
- 3D position computation
- Spatial relation logic
- Anomaly detection algorithm

### Integration Tests
- Full perception pipeline
- Message publishing/subscribing
- Configuration loading
- Error recovery

### Performance Tests
- FPS benchmarks
- Latency measurements
- Memory usage profiling
- GPU utilization

---

## Known Limitations

1. **Single camera view** - Limited field of view
2. **Depth accuracy** - Depends on camera quality and calibration
3. **YOLOv8 limitations** - Pre-trained on COCO dataset (80 classes)
4. **No temporal consistency** - Each frame independent (until tracking added)
5. **Simple anomaly detection** - Frequency-based only
6. **No occlusion reasoning** - Can't infer hidden objects

---

## Maintenance Notes

### Model Updates
- YOLOv8 models updated regularly by Ultralytics
- Consider upgrading to newer versions
- Test thoroughly before deployment

### Configuration Tuning
- Adjust `detection_fps` based on compute availability
- Tune `confidence_threshold` to reduce false positives
- Calibrate camera intrinsics for your specific hardware

### Monitoring
- Watch `/perception/scene_description` for quality
- Monitor detection count for performance
- Check anomaly alerts for false positives

---

**Last updated:** 2024-10-30
**Status:** Complete and functional
**Next milestone:** TensorRT optimization
