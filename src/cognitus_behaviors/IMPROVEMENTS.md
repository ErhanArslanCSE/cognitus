# cognitus_behaviors - Improvement Suggestions

Enhancements for the behavior execution module while maintaining simplicity.

---

## High Priority Improvements

### 1. Precise Rotation Control

**Current:** Open-loop rotation (no feedback)

**Improvement:**
- Use IMU feedback for closed-loop control
- Rotate to exact angle
- Stop when target reached

**Implementation:**
```python
def rotate_to_angle_precise(target, current_imu):
    while abs(target - current_angle) > tolerance:
        angular_vel = P * error  # PID control
        publish(angular_vel)
```

**Benefit:** Accurate orientation, repeatable movements

---

### 2. Trajectory Execution

**Current:** Single-point navigation

**Improvement:**
- Execute waypoint trajectories
- Multi-point paths
- Smooth motion

**Format:**
```
navigate_trajectory [(x1,y1), (x2,y2), (x3,y3)]
```

**Benefit:** Complex paths, patrol routes

---

### 3. Battery-aware Behavior

**Current:** Just warns on low battery

**Improvement:**
- Estimate battery for task
- Reject task if insufficient
- Auto-return to dock if critical

**Implementation:**
```python
estimated_energy = calculate_energy_for_path(goal)
if battery < estimated_energy + safety_margin:
    report("insufficient battery")
```

---

### 4. Speed Adaptation

**Current:** Fixed speeds

**Improvement:**
- Adjust speed based on obstacle distance
- Slow down when approaching
- Dynamic speed control

**Implementation:**
```python
if obstacle_dist < 1.0:
    speed = base_speed * (obstacle_dist / 1.0)
else:
    speed = base_speed
```

---

### 5. Multi-mode Support

**Current:** Assumes differential drive

**Improvement:**
- Detect LIMO drive mode
- Adapt commands per mode
- Utilize mecanum fully

**LIMO Pro modes:**
- Differential: Standard
- Mecanum: Omnidirectional (strafe)
- Ackermann: Car-like
- Track: High traction

**Benefit:** Full hardware utilization

---

## Medium Priority

### 6. Path Smoothing

**Current:** Direct Navigation2 usage

**Improvement:**
- Pre-process paths
- Smooth sharp turns
- Optimize for LIMO dynamics

---

### 7. Recovery Behaviors

**Current:** Stops on obstacle

**Improvement:**
- Try backing up
- Try rotating to find clear path
- Report to brain if stuck

**Navigation2 has recovery behaviors:**
- Integrate with them
- Custom recovery strategies

---

### 8. Collision Prediction

**Current:** Reactive (stops when obstacle close)

**Improvement:**
- Predict collision trajectory
- Preemptive slowdown
- Smoother obstacle avoidance

---

### 9. Visual Servoing

**Current:** Map-based navigation only

**Improvement:**
- Navigate to visual target
- "Move toward the table"
- No map coordinates needed

**Use perception:**
```
Brain: "Move toward table"
Behaviors: Query perception for table position
          Navigate using visual feedback
```

---

### 10. Action Queuing

**Current:** One command at a time

**Improvement:**
- Queue multiple commands
- Execute sequence
- Interrupt-able

**Benefit:** Complex behaviors, smoother execution

---

## Low Priority / Future

### 11. Manipulation Support

**Current:** Navigation only (LIMO Pro has no arm)

**Future:** If arm added
- Primitive manipulation actions
- Pick, place, grasp
- Arm control

---

### 12. Formation Control

**Current:** Single robot

**Future:** Multi-robot
- Follow leader
- Maintain formation
- Coordinated movement

---

### 13. Terrain Adaptation

**Current:** Flat surface assumption

**Improvement:**
- Detect terrain type
- Adjust speed/mode
- Track mode for rough terrain

---

### 14. Energy Optimization

**Current:** Direct paths

**Improvement:**
- Energy-optimal paths
- Coast when possible
- Regenerative braking

---

### 15. Behavior Learning

**Current:** Static primitives

**Improvement:**
- Learn new primitives from demonstration
- Optimize parameters from experience
- Adaptive behavior

---

## Research Opportunities

### Paper 7: Task Decomposition
- Brain decomposes high-level tasks
- Behaviors execute primitives
- Feedback loop for replanning

### Paper 8: Multi-Persona
- Different movement styles per persona
- Guardian: Cautious, slow
- Efficient: Fast, direct
- Companion: Following, nearby

### Paper 6: Routine Learning
- Behaviors learns common routes
- Optimizes frequently-used paths
- Suggests shortcuts to brain

---

## Implementation Priority

**Immediate (Week 1):**
1. Precise rotation control (IMU feedback)
2. Battery estimation

**Short-term (Month 1):**
3. Speed adaptation
4. Recovery behaviors

**Medium-term (Month 2-3):**
5. Trajectory execution
6. Path smoothing
7. Multi-mode support

**Long-term (Month 3+):**
8. Visual servoing
9. Collision prediction
10. Action queuing

---

## Testing Recommendations

### Unit Tests
- Command parsing
- Safety checks
- Primitive actions (mock cmd_vel)

### Integration Tests
- Navigation2 action calls
- Safety blocking
- Status reporting

### Real-world Tests
- Navigate to multiple points
- Obstacle avoidance
- Emergency stop response
- Battery low behavior

---

## Known Limitations

1. **No path verification** - Trusts brain's coordinates
2. **Open-loop rotation** - No precise angle control (yet)
3. **No stuck detection** - Can't tell if wheel spinning
4. **No arm control** - LIMO Pro has no manipulator
5. **Simple safety** - LIDAR-based only, no camera-based obstacles
6. **No localization** - Relies on Navigation2's AMCL

---

## Maintenance Notes

### Navigation2 Configuration

LIMO Pro comes with Navigation2 pre-configured:
- AMCL for localization
- DWA or TEB for local planning
- Costmap for obstacle avoidance

**This module doesn't configure Navigation2, just uses it.**

### Safety Tuning

**Adjust thresholds based on environment:**
- Open space: Lower obstacle_threshold (0.2m)
- Crowded space: Higher obstacle_threshold (0.5m)
- Fast operation: Higher battery_threshold (30%)

### Speed Limits

**LIMO Pro max speeds:**
- Linear: 1.0 m/s (indoor safe: 0.3 m/s)
- Angular: 2.0 rad/s (indoor safe: 0.5 rad/s)

**Config defaults are conservative for safety.**

---

**Last updated:** 2024-10-30
**Status:** Complete and production-ready
**Next milestone:** IMU-based rotation control
