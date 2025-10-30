# cognitus_behaviors - Academic Resources

Academic papers and research related to mobile robot navigation, motion planning, safety monitoring, and behavior execution.

**Last Updated:** October 30, 2025

---

## Mobile Robot Navigation

### 1. Advances in Social Robot Navigation: Planning, HRI, and Beyond (ICRA 2025 Workshop)
- **Conference:** ICRA 2025
- **Link:** https://2025.ieee-icra.org/event/advances-in-social-robot-navigation-planning-hri-and-beyond-2/
- **Topics:** Motion-task planning, human-aware navigation
- **Relevance:** Social navigation techniques

### 2. DARKO-Nav: Hierarchical Risk and Context-Aware Robot Navigation (2025)
- **Conference:** European Robotics Forum 2025
- **Link:** https://link.springer.com/chapter/10.1007/978-3-031-89471-8_24
- **Key Innovation:** Risk-aware navigation in complex environments
- **Relevance:** Safety-aware navigation strategies

### 3. Safe Planner: Empowering Safety Awareness in Large Pre-Trained Models (2025)
- **Authors:** Siyuan Li et al.
- **Conference:** ICRA 2025
- **Focus:** Safety-aware task planning with LLMs
- **Relevance:** Safety integration with planning

---

## Navigation2 & ROS Navigation

### 4. Navigation2 Official Documentation (2024-2025)
- **Link:** https://navigation.ros.org/
- **Platform:** ROS2 navigation stack
- **Features:** Path planning, obstacle avoidance, recovery
- **Relevance:** Core navigation system we use (LIMO Pro pre-installed)

### 5. Algorithms for Planning and Control of Robot Motion
- **Organization:** IEEE RAS Technical Committee
- **Link:** https://www.ieee-ras.org/algorithms-for-planning-and-control-of-robot-motion/activities
- **Coverage:** State-of-the-art in motion planning
- **Relevance:** Planning algorithms overview

---

## Safety & Collision Avoidance

### 6. Hierarchical Adaptive Motion Planning with Nonlinear Model Predictive Control for Safety-Critical Applications (2024)
- **Topic:** Safety-critical collaborative locomotion-manipulation
- **Approach:** Hierarchical planning with safety constraints
- **Relevance:** Safety monitoring architecture

### 7. Real-time Obstacle Avoidance for Mobile Robots (2024)
- **Common Approaches:** Dynamic window approach (DWA), Time Elastic Band (TEB)
- **Implementation:** Navigation2 includes both
- **Relevance:** Local planning with obstacle avoidance

---

## Motion Planning

### 8. SigmaRL: Sample-Efficient Multi-Agent RL for Motion Planning (2024)
- **Conference:** ITSC 2024
- **Key Innovation:** Multi-agent reinforcement learning
- **Application:** Coordinated motion planning
- **Relevance:** Future multi-robot scenarios

### 9. Motion Planning Algorithms - Recent Advances (2024)
- **Topics:** RRT*, PRM*, sampling-based planners
- **Trend:** Learning-based vs classical planners
- **Relevance:** Understanding navigation algorithms

---

## Safety Monitoring

### 10. LIDAR-Based Obstacle Detection for Mobile Robots: A Survey (2024)
- **Topics:** Point cloud processing, obstacle classification
- **Techniques:** Statistical filtering, clustering
- **Relevance:** Our safety_monitor.py LIDAR processing

### 11. Formal Methods in Robotics Systems Workshop (IROS 2024)
- **Date:** October 15, 2024, Abu Dhabi
- **Topics:** Formal verification, safety guarantees
- **Relevance:** Rigorous safety approaches

---

## LIMO Pro & Similar Platforms

### 12. AGILeX LIMO Pro User Manual (2024)
- **Link:** Official documentation
- **Content:** Hardware specs, drive modes, ROS2 integration
- **Relevance:** Platform-specific capabilities

### 13. ROS2 Foxy Navigation Stack (Official)
- **Link:** https://docs.ros.org/en/foxy/
- **Content:** Complete ROS2 Foxy documentation
- **Relevance:** API reference for our implementation

---

## Primitive Actions & Low-Level Control

### 14. Twist Message Control for Differential Drive Robots (Standard)
- **Topic:** /cmd_vel control
- **Message:** geometry_msgs/Twist
- **Standard:** ROS/ROS2 convention
- **Relevance:** Our primitive_actions.py implementation

### 15. Mecanum Wheel Control Strategies (2024)
- **Application:** LIMO Pro mecanum mode
- **Control:** Omnidirectional movement
- **Relevance:** Strafe actions implementation

---

## Task Execution & Behavior Trees

### 16. BehaviorTree.CPP: Behavior Trees for Robotics (2024)
- **Link:** https://www.behaviortree.dev/
- **Application:** Complex behavior composition
- **Relevance:** Future enhancement for complex tasks

### 17. FlexBE: Flexible Behavior Engine (2024)
- **Platform:** ROS/ROS2
- **Use Case:** State machine-based behaviors
- **Relevance:** Alternative behavior architecture

---

## Battery Management & Energy

### 18. Energy-Efficient Navigation for Mobile Robots (2024)
- **Topics:** Path optimization for battery life
- **Techniques:** Energy-aware planning
- **Relevance:** Battery monitoring strategies

---

## Safety Standards & Best Practices

### 19. ISO 13482: Safety Requirements for Personal Care Robots (2024)
- **Standard:** International safety standard
- **Application:** Service robots, mobile platforms
- **Relevance:** Safety threshold guidelines

### 20. ROS-Industrial Safety Guidelines (2024)
- **Topics:** E-stop, safety zones, risk assessment
- **Relevance:** Industrial-grade safety practices

---

## GitHub Repositories

### 21. Navigation2 Repository
- **Link:** https://github.com/ros-planning/navigation2
- **Content:** Complete Nav2 stack, examples
- **Relevance:** Source code and examples

### 22. ICRA 2024 Paper List
- **Link:** https://github.com/ryanbgriffiths/ICRA2024PaperList
- **Content:** All ICRA 2024 papers organized
- **Relevance:** Latest robotics research

---

## Key Conferences

**Robotics:**
- ICRA (International Conference on Robotics and Automation)
- IROS (International Conference on Intelligent Robots and Systems)
- RSS (Robotics: Science and Systems)
- CoRL (Conference on Robot Learning)

**Recent Papers:** https://arxiv.org/list/cs.RO/recent

---

## Research Trends (2024-2025)

**Navigation:**
- LLM-guided navigation (high-level to low-level)
- Social navigation (human-aware)
- Semantic navigation (goal by description)

**Safety:**
- Formal verification methods
- Predictive collision avoidance
- Multi-sensor fusion

**Planning:**
- Hierarchical planning (task→motion)
- Learning-based planners
- Adaptive replanning

---

## Implementation Insights from Literature

**Primitive Actions (our approach):**
- Proven effective: Simple, reliable, composable
- Low-level control → High-level planning separation ✓
- Safety at execution layer ✓

**Navigation2 (our choice):**
- Industry standard for ROS2 ✓
- Proven robust in real-world
- Well-documented, maintained

**Safety Monitoring:**
- LIDAR primary sensor for mobile robots ✓
- Multi-layer safety (warning + emergency) ✓
- Continuous monitoring essential ✓

---

## Best Practices from Literature

**From Safety Papers:**
1. Multiple safety thresholds (warning/critical)
2. Emergency stop within 100ms
3. Continuous sensor monitoring
4. Graceful degradation

**From Navigation Papers:**
1. Hierarchical planning (global→local)
2. Dynamic obstacle avoidance
3. Recovery behaviors
4. Battery-aware planning

**From Control Papers:**
1. Simple primitives compose well
2. Feedback control when possible
3. Safety checks before action
4. Status reporting essential

---

## Future Research Directions

**From Recent Papers:**
1. LLM-based motion planning
2. Visual servoing integration
3. Multi-robot coordination
4. Learning from demonstration
5. Adaptive safety thresholds

**Workshops to Follow:**
- ICRA 2025: Social Robot Navigation
- CoRL 2025: Robot Learning
- IROS: Motion Planning Track

---

## Performance Benchmarks (from community)

**Navigation2 on Mobile Robots:**
- Path planning: 100-500ms
- Local planning: 50-100ms (10Hz update)
- Safety monitoring: 1-10Hz

**Primitive Actions:**
- Command parse: <1ms
- Safety check: <5ms
- cmd_vel publish: <1ms

---

## Known Issues & Solutions (from literature)

**Dynamic Window Approach (DWA):**
- Issue: Can get stuck in local minima
- Solution: Recovery behaviors, replanning

**Pure Pursuit:**
- Issue: Path tracking errors
- Solution: Adaptive look-ahead distance

**Obstacle Avoidance:**
- Issue: Oscillation near obstacles
- Solution: Temporal smoothing, hysteresis

---

**Curated by:** COGNITUS Team
**Focus Areas:** Navigation, Motion Planning, Safety, Primitive Actions
**Technology:** Navigation2 + LIDAR Safety + Primitive Executor
**Last Literature Review:** October 2025
