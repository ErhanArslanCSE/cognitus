# Research Papers for LIMO Pro Embodied AI System

## Overview

This document presents nine interconnected research papers that form the scientific foundation of the LIMO Pro autonomous home robot system. Each paper addresses a specific challenge in embodied AI, contributes novel solutions, and has clear paths to publication in top-tier venues. The papers are designed to be implementable on actual hardware within realistic timeframes.

---

## Paper 1: Hierarchical Event Summarization for Long-term Robot Memory

### Problem Statement
Home robots generate overwhelming amounts of data (>50GB/day) but must operate with limited storage. Current systems either forget important events or overflow their memory. How can a robot maintain meaningful memories of a week's observations in limited storage?

### Research Question
Can hierarchical temporal summarization preserve important information while achieving 100x compression ratios?

### Proposed Solution
**THESIS**: Transform continuous observations into a four-level hierarchy: raw events → grouped activities → narrative stories → daily digests

### Technical Approach

```
Level 1 (Raw Events) - 10 minute retention:
- "person_detected_kitchen_14:32:05"
- "door_opened_14:32:08"
- "sound_doorbell_14:32:10"

Level 2 (Activity Groups) - 1 hour retention:
- "delivery_attempt: person→door→doorbell→wait→leave"

Level 3 (Story Formation) - 24 hour retention:
- "A delivery person arrived at 2:30 PM, rang the doorbell 
   three times over 2 minutes, waited, then left a package"

Level 4 (Daily Digest) - 7 day retention:
- "Tuesday: 2 deliveries (morning package, afternoon food), 
   unusual noise at 3 AM (investigated: cat), guest visit 4-6 PM"
```

### Novel Contributions
1. **First robotic system using narrative generation for memory compression**
2. **Information-theoretic scoring for selective retention**
3. **Human-interpretable summaries at multiple temporal scales**

### Experimental Validation

**Dataset:**
- 30-day continuous operation in real home
- ~1.5TB raw data → 8GB compressed storage

**Metrics:**
- Compression ratio: target 100x
- Information retention: human-rated 1-5 scale
- Query success rate: % of questions answerable
- Retrieval latency: time to answer queries

**Expected Results:**
- 100-150x compression achieved
- 4.2/5 human rating for summary quality
- 85% query success rate
- <2 second retrieval time

### Why This Will Succeed
- Builds on proven video summarization techniques
- Natural language summaries are inherently compact
- Human evaluation is straightforward
- Clear improvement over FIFO baselines

### Publication Strategy
**Target Venue:** HRI 2025 (Human-Robot Interaction)
**Submission Timeline:** Month 3
**Differentiator:** Focus on human-interpretable summaries

---

## Paper 2: Learning When to Look - Adaptive Duty Cycling for Mobile Robots

### Problem Statement
Continuous perception drains robot batteries in 3 hours, but important events are rare. Most of the time "nothing happens" yet robots waste energy continuously monitoring.

### Research Question
Can robots learn temporal patterns to predict quiet periods and reduce sensing without missing important events?

### Proposed Solution
**THESIS**: Mine temporal patterns from activity history to identify predictable quiet periods, then adaptively reduce sampling rates while maintaining safety triggers.

### Technical Approach

```
Phase 1 - Pattern Learning (Week 1):
- Record all activity at maximum rate (30 FPS, 15 Hz LIDAR)
- Extract temporal patterns:
  - Daily: "2 AM - 6 AM typically quiet"
  - Weekly: "Sunday mornings minimal activity"
  - Contextual: "After everyone leaves, quiet until 3 PM"

Phase 2 - Adaptive Sampling:
IF time in quiet_period AND no_motion_detected:
  vision_fps = 0.5  (was 30)
  lidar_hz = 0      (was 15)
  audio_hz = 1      (was 100)
  power = 3W        (was 18W)

Safety Overrides:
- PIR motion sensor always active (0.1W)
- Audio threshold trigger for immediate wake
- Periodic full scan every 5 minutes
```

### Novel Contributions
1. **First learned duty-cycling system for mobile robots**
2. **Temporal pattern mining for energy optimization**
3. **Probabilistic quiet period prediction**

### Experimental Validation

**Setup:**
- 2-week deployment in home environment
- Week 1: Learning patterns
- Week 2: Adaptive sampling

**Metrics:**
- Battery life extension: hours of operation
- Missed event rate: important events not detected
- False wake rate: unnecessary activations
- Energy per detected event: efficiency measure

**Expected Results:**
- 3-5x battery life (3 hours → 10-15 hours)
- <2% missed important events
- <10% false wake rate
- 70% reduction in average power consumption

### Why This Will Succeed
- Clear, measurable improvement in battery life
- Temporal patterns in homes are highly predictable
- Safety mechanisms prevent missing critical events
- Direct practical impact for deployment

### Publication Strategy
**Target Venue:** IROS 2025
**Submission Timeline:** Month 4
**Differentiator:** Learning-based vs fixed schedules

---

## Paper 3: Elementary My Dear Robot - Probabilistic Abductive Reasoning

### Problem Statement
Robots observe symptoms but cannot infer causes. They see wet floors, open umbrellas, and wet shoes but don't conclude "it's raining." This limits their ability to understand and communicate about their environment.

### Research Question
Can robots learn to make probabilistic inferences about hidden states from accumulated observable evidence?

### Proposed Solution
**THESIS**: Build probabilistic models linking observable evidence to hidden states, enabling Bayesian inference about unobserved conditions.

### Technical Approach

```
Evidence-State Learning:
For each observed sequence ending in revealed state:
  P(wet_floor | raining) = 0.8
  P(umbrella_open | raining) = 0.7
  P(wet_shoes | raining) = 0.9
  P(wet_floor | spill) = 0.6
  P(umbrella_open | spill) = 0.1

Inference Process:
Given evidence E = {wet_floor, umbrella_open, wet_shoes}
  P(raining|E) = P(E|raining) × P(raining) / P(E)
              = 0.8 × 0.7 × 0.9 × prior / normalization
              = 0.85

Output: "It's probably raining (85% confidence)"
```

### Novel Contributions
1. **First systematic abductive reasoning for embodied AI**
2. **Learning evidence-state correlations from observation**
3. **Calibrated probability outputs for uncertainty**

### Experimental Validation

**Scenarios:**
- Weather inference (rain, snow, sunny)
- Occupancy status (away, sleeping, working)
- Activity recognition (cooking, cleaning, relaxing)
- Emergency detection (fire, break-in, medical)

**Metrics:**
- Inference accuracy: precision/recall
- Calibration error: predicted vs actual probability
- Evidence efficiency: minimum evidence for 90% confidence
- Generalization: novel state-evidence combinations

**Expected Results:**
- 75-80% inference accuracy
- <0.1 expected calibration error
- 3-4 pieces of evidence sufficient
- 60% accuracy on novel combinations

### Why This Will Succeed
- Probabilistic framework is well-understood
- Observable evidence is readily available
- Clear evaluation metrics
- Practical value for situation awareness

### Publication Strategy
**Target Venue:** CoRL 2025
**Submission Timeline:** Month 5
**Differentiator:** Embodied abduction vs abstract reasoning

---

## Paper 4: Real-time Spatial Language Grounding with Single Depth Camera

### Problem Statement
Robots respond to "Where's my keys?" with coordinates "x:1.2, y:0.8, z:0.5" instead of natural descriptions like "on the table, next to your phone."

### Research Question
Can robots generate natural spatial descriptions in real-time using only a single depth camera?

### Proposed Solution
**THESIS**: Combine depth-based 3D understanding with learned spatial relation templates to generate natural language descriptions.

### Technical Approach

```
Pipeline:
1. Object Detection: YOLOv8n → objects + 2D boxes
2. Depth Projection: 2D + depth → 3D positions
3. Relation Extraction:
   For each pair (target, landmark):
     - Compute relative position
     - Classify spatial relation
     - Score salience for description

4. Description Generation:
   Template: "[target] is [relation] [landmark]"
   Example: "keys are on the table, next to the phone"
   
5. Uncertainty Expression:
   If confidence < 0.7:
     "I think the keys might be..."
```

### Novel Contributions
1. **Real-time (<500ms) spatial description generation**
2. **Single depth camera instead of full SLAM**
3. **Salience-based landmark selection**
4. **Natural uncertainty expression**

### Experimental Validation

**Setup:**
- 100 objects in home environment
- 500 spatial queries from 10 users
- Compare with multi-camera and SLAM baselines

**Metrics:**
- Localization success: user finds object from description
- Description quality: human rating 1-5
- Processing latency: query to response time
- Disambiguation: success with multiple similar objects

**Expected Results:**
- 85% localization success
- 4.0/5 description quality
- <500ms response time
- 75% disambiguation success

### Why This Will Succeed
- Depth cameras are now standard in robotics
- Spatial relations are finite and learnable
- Template-based generation is reliable
- Clear practical value for interaction

### Publication Strategy
**Target Venue:** ICRA 2025
**Submission Timeline:** Month 3
**Differentiator:** Single camera, real-time performance

---

## Paper 5: Graph-Based Scene Memory for Week-Long Robot Operation

### Problem Statement
Robots generate 50GB of visual data daily but have 8GB storage budget for a week. Current compression methods lose semantic information needed for queries.

### Research Question
Can differential scene graphs achieve 100x compression while maintaining queryable semantic memory?

### Proposed Solution
**THESIS**: Represent scenes as graphs, store only differentials between timestamps, reconstruct past states on demand.

### Technical Approach

```
Scene Graph Representation:
- Nodes: Objects with attributes
- Edges: Spatial relations
- Metadata: Timestamp, confidence

Differential Storage:
Time T0 (base): Full graph (100 nodes, 200 edges)
Time T1: +node(book), +edge(book, on, table), -node(cup)
Time T2: Δposition(book, 0.1m), +node(person)
...
Storage: Base + compressed deltas

Reconstruction:
Query: "Where was the book yesterday at 3 PM?"
1. Load nearest base graph
2. Apply deltas up to requested time
3. Extract requested information
```

### Novel Contributions
1. **First differential scene graph compression for robotics**
2. **Semantic-preserving compression at 100x ratios**
3. **Temporal queries on compressed representation**

### Experimental Validation

**Setup:**
- 7-day continuous operation
- Compare with video compression, keyframe sampling
- 1000 temporal-spatial queries

**Metrics:**
- Compression ratio: original/compressed size
- Reconstruction accuracy: IoU with ground truth
- Query success rate: correct answers
- Query latency: time to answer

**Expected Results:**
- 80-120x compression ratio
- 90% reconstruction accuracy
- 85% query success rate
- <1 second query latency

### Why This Will Succeed
- Graph differentials are inherently efficient
- Semantic information is preserved
- Builds on proven version control concepts
- Scales to long deployments

### Publication Strategy
**Target Venue:** IJRR or T-RO (journal)
**Submission Timeline:** Month 6-9
**Differentiator:** Semantic preservation at extreme compression

---

## Paper 6: Learning Human Routines for Proactive Assistance

### Problem Statement
Robots react to commands but don't anticipate needs. They don't learn that coffee is needed every morning at 8 AM or that lights should be checked before bed.

### Research Question
Can robots discover human routines through observation and provide proactive assistance without explicit programming?

### Proposed Solution
**THESIS**: Mine temporal patterns from human activities, identify routines with high confidence, formulate proactive assistance behaviors.

### Technical Approach

```
Pattern Mining:
- Input: Activity stream over time
- Algorithm: PrefixSpan with temporal constraints
- Output: Recurring sequences with timing

Example Patterns:
Pattern: wake → bathroom → kitchen
Timing: 7:00-7:30 AM daily
Confidence: 0.85
Action: Prepare coffee at 7:25 AM

Routine Types Discovered:
1. Morning: Coffee, news display, weather check
2. Leaving: Keys/wallet reminder, lights off
3. Evening: Temperature adjustment, device charging
4. Bedtime: Door locks, alarm set

Validation Loop:
- Execute proactive action
- Observe human response
- Positive: Reinforce pattern
- Negative: Adjust timing/action
```

### Novel Contributions
1. **Unsupervised routine discovery from observation**
2. **Temporal pattern mining with action generation**
3. **No reward function or programming required**

### Experimental Validation

**Setup:**
- 2-week deployment in 5 homes
- No initial programming of routines
- Measure discovered vs actual routines

**Metrics:**
- Routine discovery rate: found/actual
- Timing accuracy: predicted vs actual time
- User acceptance: appreciated/total actions
- Learning speed: days to stable routine

**Expected Results:**
- 70% of major routines discovered
- ±15 minute timing accuracy
- 60% user acceptance rate
- 5-7 days to learn routine

### Why This Will Succeed
- Human routines are highly regular
- Pattern mining is well-established
- Clear value proposition for users
- Measurable improvement in assistance

### Publication Strategy
**Target Venue:** RSS 2025
**Submission Timeline:** Month 5
**Differentiator:** No programming or reward engineering

---

## Paper 7: Self-Delegating Task Decomposition for Autonomous Robots

### Problem Statement
Robots fail at complex tasks because they can't break them down into manageable steps. "Check all windows" requires understanding of subtasks: list rooms, navigate, identify windows, check status, report.

### Research Question
Can robots autonomously decompose high-level tasks into executable subtasks without human-provided hierarchies?

### Proposed Solution
**THESIS**: Use language models for recursive task decomposition, validate subtasks against capabilities, execute with monitoring and re-planning.

### Technical Approach

```
Decomposition Algorithm:
Input: "Check all windows in the house"

Step 1 - Initial breakdown:
- Identify all rooms
- For each room: go to room
- For each room: find windows
- For each window: check if open/closed
- Report findings

Step 2 - Recursive refinement:
"go to room" →
- Plan path to room
- Navigate avoiding obstacles
- Confirm arrival

Step 3 - Capability checking:
For each atomic subtask:
  Can_execute(subtask)?
  If no: decompose further

Step 4 - Execution with monitoring:
- Execute subtasks sequentially
- If failure: re-decompose failed task
- Update decomposition strategy
```

### Novel Contributions
1. **Recursive decomposition without task hierarchies**
2. **Capability-aware task breakdown**
3. **Learning from decomposition failures**

### Experimental Validation

**Tasks:**
- "Check all windows" (navigation + perception)
- "Find three red objects" (search + counting)
- "Report unusual things" (patrol + anomaly detection)
- "Ensure home is secure" (multi-aspect check)

**Metrics:**
- Decomposition success: executable plans generated
- Task completion: full task achieved
- Efficiency: steps vs optimal
- Adaptation: improvement over attempts

**Expected Results:**
- 75% successful decomposition
- 60% end-to-end completion
- 1.5x optimal step count
- 30% improvement after 5 attempts

### Why This Will Succeed
- LLMs are good at breaking down tasks
- Capability checking ensures feasibility
- Learning improves over time
- Clear practical value

### Publication Strategy
**Target Venue:** AAMAS 2025 or ICRA 2025
**Submission Timeline:** Month 6
**Differentiator:** No pre-programmed hierarchies

---

## Paper 8: Multi-Persona Behavioral Adaptation for Context-Aware Robots

### Problem Statement
Robots use the same behavior regardless of context. They're equally chatty at night (when people want to sleep) and during emergencies (when quick action is needed).

### Research Question
Can a single robot develop and switch between distinct behavioral personas based on environmental and social context?

### Proposed Solution
**THESIS**: Define behavioral personas with different parameters, learn context-persona mappings, smoothly transition between personas.

### Technical Approach

```
Persona Definitions:

Guardian (Night Security):
- High motion sensitivity
- Silent operation
- Patrol behavior
- Alert on anomalies

Assistant (Daily Tasks):
- Normal interaction
- Task-focused
- Efficient routing
- Proactive help

Companion (Social):
- More conversational
- Stays near humans
- Emotional support
- Patience mode

Emergency (Crisis):
- Maximum sensor rates
- Direct communication
- Safety priorities
- Ignore battery limits

Context → Persona Mapping:
Features: time, motion, sound, human_state, battery
Classifier: Random Forest
Output: Persona + confidence
```

### Novel Contributions
1. **Dynamic persona switching in single robot**
2. **Context-behavior mapping learned from feedback**
3. **Smooth behavioral transitions**

### Experimental Validation

**Setup:**
- 2-week deployment
- 4 personas defined
- Multiple context types daily
- User feedback on appropriateness

**Metrics:**
- Context recognition: correct persona selection
- Transition smoothness: user rating 1-5
- Task performance: success per persona
- User satisfaction: behavior appropriateness

**Expected Results:**
- 85% correct context recognition
- 4.2/5 transition smoothness
- 25% better task performance
- 4.0/5 appropriateness rating

### Why This Will Succeed
- Contexts are distinguishable
- Behavior parameters are adjustable
- User feedback is clear signal
- Improves user experience significantly

### Publication Strategy
**Target Venue:** HRI 2025
**Submission Timeline:** Month 7
**Differentiator:** Single robot, learned switching

---

## Paper 9: Curiosity-Driven Exploration for Autonomous Discovery

### Problem Statement
Robots only go where told, missing opportunities to discover useful information. They don't explore behind furniture, check closed doors when open, or investigate unusual sounds.

### Research Question
Can robots autonomously generate and pursue exploration goals based on information-theoretic curiosity?

### Proposed Solution
**THESIS**: Quantify uncertainty in world model, generate exploration goals for high-information areas, balance curiosity with safety and energy.

### Technical Approach

```
Curiosity Formulation:
For each location L:
  Uncertainty U(L) = entropy of predictions
  Novelty N(L) = time since last visit
  Reachability R(L) = path cost
  
  Curiosity C(L) = α·U(L) + β·N(L) - γ·R(L)

Exploration Planning:
1. Rank locations by C(L)
2. Filter by safety constraints
3. Check energy budget
4. Generate exploration goal
5. Execute and update model

Example Discoveries:
- "Never seen behind couch" → Find lost items
- "Door usually closed, now open" → Map new room
- "Unusual sound from attic" → Detect problem
- "Corner never visited" → Complete map
```

### Novel Contributions
1. **Information-theoretic curiosity for physical robots**
2. **Balancing exploration with energy constraints**
3. **Safety-bounded autonomous exploration**

### Experimental Validation

**Setup:**
- Unknown apartment environment
- 1 week autonomous exploration
- Compare with random walk and frontier exploration

**Metrics:**
- Coverage: % environment explored
- Discoveries: meaningful findings per day
- Efficiency: information per energy unit
- Safety: incidents during exploration

**Expected Results:**
- 40% better coverage than random
- 3-5 meaningful discoveries daily
- 2x information per battery charge
- Zero safety incidents

### Why This Will Succeed
- Information theory provides principled approach
- Safety bounds prevent problems
- Clear improvement over baselines
- Discoveries have real value

### Publication Strategy
**Target Venue:** IROS 2025 or Autonomous Robots journal
**Submission Timeline:** Month 8
**Differentiator:** Physical robot, energy-aware

---

## Integration and Timeline

### Development Phases

**Phase 1 (Months 1-3): Core Capabilities**
- Paper 4: Spatial Language (ICRA submission)
- Paper 1: Hierarchical Summarization (HRI submission)
- Basic system integration

**Phase 2 (Months 4-6): Learning Systems**
- Paper 2: Adaptive Sampling (IROS submission)
- Paper 6: Routine Learning (RSS submission)
- Paper 3: Abductive Reasoning (CoRL submission)

**Phase 3 (Months 7-9): Advanced Behaviors**
- Paper 7: Task Decomposition (AAMAS submission)
- Paper 8: Multi-Persona (HRI submission)
- Paper 9: Curiosity (IROS submission)

**Phase 4 (Months 10-12): System Paper**
- Paper 5: Graph Memory (Journal submission)
- Integrated system validation
- 30-day deployment study

### Success Metrics

**Individual Papers:**
- Each addresses clear problem
- Novel solution with theoretical backing
- Implementable on LIMO Pro
- Clear evaluation metrics
- Strong publication venue

**System Integration:**
- Papers complement each other
- Shared infrastructure and data
- Combined system greater than parts
- Real-world validation
- Open source release planned

### Risk Mitigation

**Technical Risks:**
- Each paper has fallback approaches
- Modular design allows independent development
- Conservative baselines ensure some results

**Publication Risks:**
- Multiple venue options per paper
- Staggered submissions reduce coupling
- Strong empirical results focus

---

## Conclusion

These nine papers form a comprehensive research program in embodied AI that:

1. **Addresses real problems** - Each paper solves a genuine challenge in home robotics
2. **Provides novel solutions** - New approaches not existing in literature
3. **Validates on real hardware** - LIMO Pro deployment, not just simulation
4. **Builds toward complete system** - Papers integrate into working robot
5. **Targets top venues** - Clear path to publication in premier conferences

The research program is ambitious yet achievable, with each paper contributing both individual value and system-level capabilities. The modular approach allows parallel development while maintaining integration potential.

Expected impact: 6-9 publications in top venues, 500+ citations within 3 years, and a working home robot system that advances the field of embodied AI.