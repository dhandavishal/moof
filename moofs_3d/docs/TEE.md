---

## 1. System Overview:

```
┌────────────────────────────────────────────────────────────────┐
│                    USER/OPERATOR LAYER                         │
│     Mission Plans, Commands, Monitoring Dashboard              │
└──────────────────────────┬───────────────────────────────────┘
                           │
                           ▼
┌────────────────────────────────────────────────────────────────┐
│           TASK EXECUTION ENGINE (TEE)  ⚙️                       │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │ Mission Manager                                          │ │
│  │ - Receives missions from user                           │ │
│  │ - Breaks down into atomic tasks                         │ │
│  │ - Manages task dependencies                             │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │ Task Queue (Priority-Based)                             │ │
│  │ - Prioritizes tasks (0-10 scale)                        │ │
│  │ - Manages dependencies                                  │ │
│  │ - Handles retries                                       │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │ State Machine                                            │ │
│  │ - Manages mission states (IDLE, ARMED, IN_TRANSIT, etc)│ │
│  │ - Ensures valid transitions                             │ │
│  │ - Coordinates with drone state                          │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │ Task Executor (Main Loop)                               │ │
│  │ - "What should happen?" (Strategy Layer)                │ │
│  │ - Sends task goals to FAL via ROS2 Actions            │ │
│  │ - Monitors progress and receives feedback               │ │
│  │ - Handles timeouts and errors                           │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                │
└────────────────────────────┬────────────────────────────────┘
                             │
                    ROS2 Action Interface
                    (/drone_0/takeoff, etc)
                             │
                             ▼
┌────────────────────────────────────────────────────────────────┐
│        FLIGHT ABSTRACTION LAYER (FAL)  🚁                       │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │ Action Servers (ROS2)                                   │ │
│  │ - Takeoff Action Server                                │ │
│  │ - Land Action Server                                   │ │
│  │ - GoToWaypoint Action Server                           │ │
│  │ - ExecutePrimitive Action Server                       │ │
│  │ - ArmDisarm Service Server                             │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │ Flight Primitives                                        │ │
│  │ - "How to actually do it?" (Implementation Layer)       │ │
│  │ - Arm/Disarm                                             │ │
│  │ - Takeoff                                                │ │
│  │ - GoToWaypoint (Navigation)                              │ │
│  │ - Land                                                   │ │
│  │ - Hover                                                  │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                │
│  ┌──────────────────────────────────────────────────────────┐ │
│  │ MAVROS Bridge                                            │ │
│  │ - Communicates with MAVROS drivers                      │ │
│  │ - Reads sensor data (GPS, IMU, battery, etc)           │ │
│  │ - Publishes commands to drone                           │ │
│  └──────────────────────────────────────────────────────────┘ │
│                                                                │
└────────────────────────────┬────────────────────────────────┘
                             │
                    MAVROS Topics/Services
                             │
                             ▼
┌────────────────────────────────────────────────────────────────┐
│        ArduPilot / Autopilot (SITL or Real Hardware)           │
│        - Flight Control                                        │
│        - Sensor Fusion                                         │
│        - Stability Control                                     │
└────────────────────────────────────────────────────────────────┘
```


## 3. Interaction Pattern: The Complete Flow

### **Step-by-Step: Takeoff Task Example**

```
┌─────────────────────────────────────────────────────────────────┐
│ TEE: MISSION PHASE (Strategy)                                   │
└─────────────────────────────────────────────────────────────────┘

1. USER SUBMITS MISSION
   ┌──────────────────────────────────────────┐
   │ Mission: "Fly to 50m altitude, patrol"   │
   │ Drones: 3 (A, B, C)                      │
   │ Duration: 30 minutes                     │
   └──────────────────────────────────────────┘
                    │
                    ▼
   TEE: MISSION → TASK BREAKDOWN
   ┌──────────────────────────────────────────┐
   │ Task 1: Arm all drones                   │
   │ Task 2: Takeoff all drones to 50m        │
   │ Task 3: Patrol route (Drone A)           │
   │ Task 4: Patrol route (Drone B)           │
   │ Task 5: Patrol route (Drone C)           │
   │ Task 6: RTH and land all drones          │
   │ Task 7: Disarm all drones                │
   └──────────────────────────────────────────┘
                    │
                    ▼
   TEE: TASK QUEUE CREATION
   ┌──────────────────────────────────────────┐
   │ Priority │ Task                    │ Status│
   ├──────────┼────────────────────────┼───────┤
   │    9     │ Arm_A                  │ Queue │
   │    9     │ Arm_B                  │ Queue │
   │    9     │ Arm_C                  │ Queue │
   │    8     │ Takeoff_A (50m)        │ Queue │
   │    8     │ Takeoff_B (50m)        │ Queue │
   │    8     │ Takeoff_C (50m)        │ Queue │
   │    6     │ Patrol_A               │ Queue │
   │    6     │ Patrol_B               │ Queue │
   │    6     │ Patrol_C               │ Queue │
   └──────────────────────────────────────────┘


┌─────────────────────────────────────────────────────────────────┐
│ TEE: EXECUTION PHASE (Sending to FAL)                           │
└─────────────────────────────────────────────────────────────────┘

2. TEE GETS NEXT TASK FROM QUEUE
   Current task: Takeoff_A (Priority: 8)
   Target drone: Drone A
   Parameters: target_altitude=50m, climb_rate=2m/s
                    │
                    ▼

3. TEE VALIDATES PRECONDITIONS
   ┌────────────────────────────────┐
   │ ✓ Drone A is armed             │
   │ ✓ Drone A is in GUIDED mode    │
   │ ✓ GPS has lock                 │
   │ ✓ Battery > 50%                │
   └────────────────────────────────┘
                    │
                    ▼

4. TEE SENDS TASK TO FAL (ROS2 Action)
   ┌─────────────────────────────────────────┐
   │ Action Client sends goal to:            │
   │ /drone_0/takeoff                        │
   │                                         │
   │ Goal Message:                           │
   │ {                                       │
   │   target_altitude: 50.0,                │
   │   climb_rate: 2.0,                      │
   │   timeout: 30.0                         │
   │ }                                       │
   └─────────────────────────────────────────┘
                    │
                    ▼
        (Crosses ROS2 Action Interface)
                    │
                    ▼

┌─────────────────────────────────────────────────────────────────┐
│ FAL: EXECUTION PHASE (How to do it)                             │
└─────────────────────────────────────────────────────────────────┘

5. FAL RECEIVES TAKEOFF GOAL
   ┌──────────────────────────────────┐
   │ Takeoff Action Server:           │
   │ execute_callback() called        │
   │                                  │
   │ Goal received:                   │
   │ {                                │
   │   target_altitude: 50.0,         │
   │   climb_rate: 2.0                │
   │ }                                │
   └──────────────────────────────────┘
                    │
                    ▼

6. FAL INVOKES PRIMITIVE
   ┌──────────────────────────────────┐
   │ TakeoffPrimitive.execute()       │
   │                                  │
   │ Actions:                         │
   │ ✓ Verify armed state             │
   │ ✓ Switch to GUIDED mode          │
   │ ✓ Start publishing commands      │
   │ ✓ Set climb rate                 │
   │ ✓ Initialize monitoring          │
   └──────────────────────────────────┘
                    │
                    ▼

7. FAL PUBLISHES SETPOINT COMMANDS
   ┌─────────────────────────────────────┐
   │ MAVROS Topic:                       │
   │ /drone_0/mavros/setpoint_position   │
   │                                     │
   │ Published at 20 Hz:                 │
   │ {                                   │
   │   x: 0, y: 0,                       │
   │   z: 50.0,                          │
   │   frame: FRAME_LOCAL_NED            │
   │ }                                   │
   └─────────────────────────────────────┘
                    │
                    ▼
        (Communicates with ArduPilot)
                    │
                    ▼

8. AUTOPILOT EXECUTES COMMAND
   ┌──────────────────────────────────┐
   │ ArduPilot (SITL/Real Hardware)   │
   │ - Reads setpoint (z=50m)         │
   │ - Applies climb_rate (2 m/s)     │
   │ - Stabilizes attitude            │
   │ - Maintains altitude target      │
   │ - Publishes status updates       │
   └──────────────────────────────────┘
                    │
                    ▼

9. FAL MONITORS EXECUTION (Real-Time Loop)
   ┌─────────────────────────────────────┐
   │ Every 0.1 seconds (10 Hz):          │
   │                                     │
   │ ✓ Read current altitude             │
   │   /drone_0/mavros/local_position    │
   │                                     │
   │ ✓ Calculate progress                │
   │   progress = current_alt / target_alt │
   │                                     │
   │ ✓ Check completion condition        │
   │   if altitude >= target - tolerance │
   │                                     │
   │ ✓ Publish feedback to TEE           │
   │   Feedback:                         │
   │   {                                 │
   │     current_altitude: 32.5,         │
   │     progress_percentage: 65,        │
   │     status: "climbing"              │
   │   }                                 │
   │                                     │
   │ ✓ Check timeout                     │
   │   if elapsed_time > timeout         │
   │                                     │
   │ ✓ Check cancellation                │
   │   if goal_cancelled, abort          │
   └─────────────────────────────────────┘
                    │
         (Publishes feedback back to TEE)
                    │
                    ▼

┌─────────────────────────────────────────────────────────────────┐
│ TEE: MONITORING PHASE (Tracking Progress)                       │
└─────────────────────────────────────────────────────────────────┘

10. TEE MONITORS FAL FEEDBACK
    ┌────────────────────────────────────┐
    │ Action Client callback receives:   │
    │                                    │
    │ Feedback:                          │
    │ {                                  │
    │   current_altitude: 32.5,          │
    │   progress_percentage: 65,         │
    │   status: "climbing"               │
    │ }                                  │
    │                                    │
    │ TEE Actions:                       │
    │ ✓ Log progress                     │
    │ ✓ Update mission status            │
    │ ✓ Check for issues                 │
    │ ✓ Publish to monitoring dashboard  │
    └────────────────────────────────────┘
                    │
         (Every 100ms, multiple updates)
                    │
                    ▼

11. FAL COMPLETES TAKEOFF
    ┌────────────────────────────────────┐
    │ Current altitude: 50.1m            │
    │ Target altitude: 50.0m             │
    │ Tolerance: ±0.5m                   │
    │                                    │
    │ ✓ Condition met! Takeoff complete  │
    │                                    │
    │ Return result to TEE:              │
    │ {                                  │
    │   success: true,                   │
    │   final_altitude: 50.1,            │
    │   message: "Takeoff successful",   │
    │   duration: 24.5                   │
    │ }                                  │
    └────────────────────────────────────┘
                    │
        (Crosses ROS2 Action Interface)
                    │
                    ▼

12. TEE RECEIVES COMPLETION
    ┌────────────────────────────────────┐
    │ Action Server result:              │
    │ {                                  │
    │   success: true,                   │
    │   final_altitude: 50.1,            │
    │   message: "Takeoff successful",   │
    │   duration: 24.5                   │
    │ }                                  │
    │                                    │
    │ TEE Actions:                       │
    │ ✓ Mark task complete               │
    │ ✓ Update state machine             │
    │ ✓ Remove from queue                │
    │ ✓ Get next task (Takeoff_B)        │
    │ ✓ Repeat process...                │
    └────────────────────────────────────┘
```

---

## 4. Communication Flow Diagram

```
TIMELINE VIEW: How Messages Flow

Time: t=0
┌─────────────┐
│ TEE sends   │
│ Takeoff     │ ──────────────────────────────────┐
│ goal        │                                    │
└─────────────┘                                    ▼
                                        ┌──────────────────┐
                                        │ FAL receives     │
                                        │ goal             │
                                        │ t = 0 ms         │
                                        └──────────────────┘

Time: t=100ms (continuous 10 Hz updates)
                                        ┌──────────────────┐
                                        │ FAL publishes    │
                                        │ feedback         │
                                        │ (altitude: 5m)   │
                                        └──────────────────┘
                                                    │
┌─────────────┐                                     │
│ TEE receives│◄────────────────────────────────────┘
│ feedback    │
│ (5m)        │
└─────────────┘

Time: t=200ms
                                        ┌──────────────────┐
                                        │ FAL publishes    │
                                        │ feedback         │
                                        │ (altitude: 10m)  │
                                        └──────────────────┘
                                                    │
┌─────────────┐                                     │
│ TEE receives│◄────────────────────────────────────┘
│ feedback    │
│ (10m)       │
└─────────────┘

... (continues every 100ms) ...

Time: t=24.5s (Completion)
                                        ┌──────────────────┐
                                        │ FAL publishes    │
                                        │ RESULT           │
                                        │ (success: true)  │
                                        └──────────────────┘
                                                    │
┌─────────────┐                                     │
│ TEE receives│◄────────────────────────────────────┘
│ result      │
│ (complete)  │
└─────────────┘
```

---

## 5. Key Design Principles

### **Principle 1: Asynchronous Communication**
```
TEE doesn't wait for completion:
✓ Sends goal to FAL
✓ Continues managing other tasks
✓ Receives feedback in callbacks
✓ Handles results when ready

This enables parallel execution of multiple tasks!
```

### **Principle 2: Feedback Loop (10 Hz)**
```
FAL continuously publishes state:

Every 100ms:
- Current altitude
- Progress percentage
- Status message
- Diagnostic info

TEE uses this to:
- Monitor progress
- Detect anomalies
- Update user dashboard
- Make dynamic decisions
```

### **Principle 3: Abstraction Layers**
```
TEE doesn't care HOW takeoff works:
- Doesn't know about PID loops
- Doesn't know about MAVROS topics
- Doesn't know about ArduPilot commands
- Just sends high-level goal

FAL doesn't care WHAT mission is running:
- Doesn't know about task queue
- Doesn't know about priority
- Doesn't know about multi-drone coordination
- Just executes primitive, returns result
```

### **Principle 4: Error Isolation**
```
Problems at FAL level (primitive-level):
- Low-level retries
- Sensor reads and validation
- Timeout handling
- State recovery

Problems at TEE level (task-level):
- Task retry
- Task cancellation
- Reassignment to another drone
- Mission replanning
```

### **Principle 5: ROS2 Action Paradigm**
```
Why use ROS2 Actions (not Services or Topics)?

Services:  Request-Reply → Too simple (no feedback)
Topics:    Publish-Subscribe → No goal semantics
Actions:   Goal-Feedback-Result → Perfect fit!

Actions provide:
✓ Send goal to FAL
✓ Receive continuous feedback (10 Hz)
✓ Get final result
✓ Cancel mid-execution
✓ Handle timeouts
✓ Multi-threaded execution
```

---

## 6. Multi-Drone Scaling

```
With 3 drones, the architecture scales:

TEE (Global Level):
├─ Task Queue (all tasks, all drones)
├─ State Machine (all drone states)
└─ Scheduler (allocate tasks to drones)

FAL Nodes (Per-Drone):
├─ Drone A FAL Node
│  ├─ Action Servers for Drone A
│  └─ Primitives for Drone A
├─ Drone B FAL Node
│  ├─ Action Servers for Drone B
│  └─ Primitives for Drone B
└─ Drone C FAL Node
   ├─ Action Servers for Drone C
   └─ Primitives for Drone C

Parallel Execution:
- TEE sends Takeoff_A to FAL_A
- TEE sends Takeoff_B to FAL_B
- TEE sends Takeoff_C to FAL_C
- All 3 drones takeoff simultaneously!
- FAL updates via namespaced topics
  /drone_0/takeoff
  /drone_1/takeoff
  /drone_2/takeoff
```

---

## 7. Error Handling & Recovery

```
FAILURE SCENARIO: FAL Takes Too Long

TEE sends: Takeoff goal (timeout: 30 sec)
           ↓
FAL executes: Takeoff starts, climbing
           ↓
30 seconds pass: No completion yet!
           ↓
FAL detects: TIMEOUT
           ↓
FAL actions:
  - Stop climb commands
  - Return Result (success: false)
  - Include error message
           ↓
TEE receives: FAILED result
           ↓
TEE actions:
  - Mark task as FAILED
  - Decision:
    a) Retry takeoff? (retry_count++)
    b) Cancel mission?
    c) Switch to different drone?
    d) User intervention?
           ↓
TEE decides: Retry (retry_count < max_retries)
           ↓
Send new Takeoff goal to FAL
```

---

## 8. Real-Time Monitoring Dashboard

```
What the user sees (powered by TEE-FAL data):

┌─────────────────────────────────────────┐
│     Multi-Drone Surveillance Dashboard   │
├─────────────────────────────────────────┤
│                                         │
│  DRONE A (Takeoff)                      │
│  ├─ Altitude: 32.5m / 50m  [65%] ▓▓▓   │
│  ├─ Battery: 75%                        │
│  ├─ Status: Climbing                    │
│  ├─ ETA: 15 seconds                     │
│  └─ Next Task: Survey_Area1             │
│                                         │
│  DRONE B (Patrol)                       │
│  ├─ Altitude: 50.0m / 50m  [100%] ▓▓▓▓▓│
│  ├─ Battery: 65%                        │
│  ├─ Status: In Flight (Patrol Route)    │
│  ├─ Waypoint 3/5 reached                │
│  └─ Next Task: Land                     │
│                                         │
│  DRONE C (Charging)                     │
│  ├─ Altitude: 0m                        │
│  ├─ Battery: 45% → 85%                  │
│  ├─ Status: Charging (11 min remaining) │
│  ├─ Charge Rate: +4% per minute         │
│  └─ Next Task: Takeoff (queue)          │
│                                         │
│  MISSION STATUS                         │
│  ├─ Progress: 45% (127 min of 280)      │
│  ├─ Tasks Completed: 24/52              │
│  ├─ Tasks Running: 3                    │
│  ├─ Tasks Queued: 25                    │
│  └─ Overall Health: ✓ Good              │
│                                         │
└─────────────────────────────────────────┘

Data flow:
FAL → TEE → Dashboard (Published via topics)
```

---

## 9. Sequence Diagram: Complete Mission

```
User    TEE         FAL_A       FAL_B       FAL_C    Drone
 │       │            │           │           │       │
 │ Mission│            │           │           │       │
 ├───────>│ Create     │           │           │       │
 │        │ Tasks      │           │           │       │
 │        │ Queue      │           │           │       │
 │        │            │           │           │       │
 │        │─Arm goal──>│           │           │       │
 │        │            │─Command──────────────────────>│
 │        │            │           │           │       │
 │        │<─Feedback─┤           │           │       │
 │        │ (armed)    │           │           │       │
 │        │            │           │           │       │
 │        │<─Result───┤           │           │       │
 │        │ (success)  │           │           │       │
 │        │            │           │           │       │
 │        │─Arm goal──────────────>│           │       │
 │        │                        │─Command──────────>│
 │        │            │<─Feedback────────────────────┤
 │        │<─Result──────────────┤           │       │
 │        │            │           │           │       │
 │        │─Takeoff goal>          │           │       │
 │        │            │           │           │       │
 │        │<─Feedback─┤ (10 Hz)   │           │       │
 │        │<─Feedback─┤           │           │       │
 │        │<─Feedback─┤           │           │       │
 │        │<─Feedback─┤           │           │       │
 │        │            │           │           │       │
 │        │<─Result───┤           │           │       │
 │        │ (success)  │           │           │       │
 │        │            │           │           │       │
 │        │─Takeoff goal───────────────────────>      │
 │        │<─Feedback─────────────────────────┤      │
 │        │<─Feedback─────────────────────────┤      │
 │        │            │           │           │       │
 │        │ (Send Navigate to A, B, C in parallel)    │
 │        │            │           │           │       │
 │        │ (Monitor all in parallel)         │       │
 │        │            │           │           │       │
 │        │ (Multi-drone coordination!)       │       │
 │        │            │           │           │       │
 │        │ ...continue mission...            │       │
```

---

## 10. High-Level Summary Table

| Aspect | TEE | FAL |
|--------|-----|-----|
| **Responsibility** | WHAT & WHEN | HOW & WHY |
| **Scope** | Mission strategy | Primitive execution |
| **Time Scale** | Seconds to hours | 10-30 seconds |
| **Feedback Rate** | Periodic monitoring | 10 Hz continuous |
| **Error Handling** | Task-level retry/cancel | Low-level sensor validation |
| **State** | Mission state machine | Primitive state machine |
| **Interface** | ROS2 Action (sends goal) | ROS2 Action (receives goal) |
| **Abstraction Level** | High-level tasks | Low-level commands |
| **Dependency** | Depends on FAL | Independent of mission |
| **Scalability** | Manages N drones | One per drone (namespaced) |
| **Typical Runtime** | Full mission (8+ hours) | Single primitive (seconds) |

---

## 11. Information Flow Summary

```
┌─────────────────────────────────────────────────────────────────┐
│ FORWARD FLOW (Command/Goal Direction)                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│ User Mission → TEE breaks down → Task Queue                    │
│                                    │                            │
│                                    ▼                            │
│                         Pick next highest priority task          │
│                                    │                            │
│                                    ▼                            │
│                      Send goal to FAL via ROS2 Action           │
│                                    │                            │
│                                    ▼                            │
│                     FAL executes primitive on drone              │
│                                    │                            │
│                                    ▼                            │
│              ArduPilot controls drone (actual flight)            │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

┌─────────────────────────────────────────────────────────────────┐
│ FEEDBACK FLOW (Status/Progress Direction)                       │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│     ArduPilot publishes sensor data (10+ Hz)                    │
│                                    │                            │
│                                    ▼                            │
│     MAVROS reads sensors and publishes on topics                │
│                                    │                            │
│                                    ▼                            │
│     FAL reads topics, calculates progress (feedback)            │
│                                    │                            │
│                                    ▼                            │
│     FAL publishes feedback to TEE via ROS2 Action              │
│                                    │                            │
│                                    ▼                            │
│     TEE receives feedback callback (10 Hz)                      │
│                                    │                            │
│                                    ▼                            │
│     TEE updates UI/Dashboard with current status                │
│                                    │                            │
│                                    ▼                            │
│         User sees real-time drone status on screen              │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

---

## 12. Key Takeaways

### **TEE is the Strategic Brain**
- Thinks about the mission (long-term)
- Decides WHAT tasks to do and WHEN
- Manages priorities and dependencies
- Handles high-level errors
- Orchestrates multi-drone operations

### **FAL is the Tactical Executor**
- Thinks about primitives (short-term)
- Focuses on HOW to do each task
- Handles real-time flight control
- Recovers from low-level errors
- Provides continuous feedback

### **They Communicate via ROS2 Actions**
- Asynchronous, non-blocking
- Supports goal, feedback, result model
- Perfect for long-running operations
- Scalable to multiple drones
- Clean abstraction boundary

### **This Design Enables**
✅ Parallel multi-drone execution
✅ Mission-level error recovery
✅ Real-time monitoring & feedback
✅ Dynamic task prioritization
✅ Complex multi-step missions
✅ Graceful failure handling
✅ Easy testing and debugging
✅ Future enhancements (AI, planning, etc)

---

## Summary: One-Line Explanation

**TEE decides WHAT to do and WHEN to do it, while FAL figures out HOW to actually do it, using ROS2 Actions to communicate asynchronously with continuous 10 Hz feedback.**
