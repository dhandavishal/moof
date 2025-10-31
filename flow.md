┌─────────────────────────────────────────────────────────────┐
│ Squadron Manager                                             │
│   sends JSON mission via /squadron/mission_command           │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│ TEE: _mission_callback()                                     │
│   • Parse JSON                                               │
│   • Create PrioritizedTask                                   │
│   • Enqueue in TaskQueue                                     │
│   • Transition: IDLE → VALIDATING                            │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│ TEE: _handle_validating_state()                              │
│   • Get next task from queue                                 │
│   • Check battery (BatteryMonitor)                           │
│   • Check GPS (GPSMonitor)                                   │
│   • Run TaskValidator.validate_task()                        │
│      - Syntax, battery, GPS, connection checks               │
└────────────────────┬────────────────────────────────────────┘
                     ↓
              ┌──────┴──────┐
              │ Valid?      │
              └──┬──────┬───┘
          No ←───┘      └───→ Yes
          ↓                  ↓
    ┌─────────────┐   ┌──────────────────────────────────────┐
    │ Mark failed │   │ Select executor based on task_type:  │
    │ → ABORTED   │   │   • waypoint → WaypointExecutor      │
    └─────────────┘   │   • survey   → SurveyExecutor        │
                      │   • search   → SearchExecutor        │
                      └────────────┬─────────────────────────┘
                                   ↓
                      ┌──────────────────────────────────────┐
                      │ Executor.execute(parameters)         │
                      │   Returns: List[PrimitiveCommand]    │
                      │                                      │
                      │ WaypointExecutor example:            │
                      │   Input: waypoints=[{pos:[x,y,z]}]  │
                      │   Output: [Goto(x1,y1,z1),          │
                      │            Goto(x2,y2,z2)]          │
                      └────────────┬─────────────────────────┘
                                   ↓
                      ┌──────────────────────────────────────┐
                      │ Build full sequence:                 │
                      │   1. ARM                             │
                      │   2. TAKEOFF (altitude from config)  │
                      │   3. [mission primitives...]         │
                      │   4. LAND                            │
                      │                                      │
                      │ Store in: self.current_primitives    │
                      │ Set: self.primitive_index = 0        │
                      │                                      │
                      │ Transition: VALIDATING → EXECUTING   │
                      └────────────┬─────────────────────────┘
                                   ↓
┌─────────────────────────────────────────────────────────────┐
│ TEE: _handle_executing_state() [20Hz loop]                  │
│                                                              │
│   For each primitive in sequence:                           │
│   ┌────────────────────────────────────────────────┐       │
│   │ 1. Get primitive[primitive_index]              │       │
│   │ 2. Log: "Executing primitive X/Y: TYPE"        │       │
│   │ 3. Call: _execute_primitive_blocking()         │       │
│   │    ├─ arm  → _execute_arm()                    │       │
│   │    ├─ takeoff → _execute_takeoff()             │       │
│   │    ├─ goto → _execute_goto()                   │       │
│   │    └─ land → _execute_land()                   │       │
│   │                                                  │       │
│   │ Each execution method:                          │       │
│   │   • Creates action goal/service request         │       │
│   │   • Sends to FAL action server                  │       │
│   │   • BLOCKS until FAL completes                  │       │
│   │   • Returns success/failure                     │       │
│   │                                                  │       │
│   │ 4. Update ProgressMonitor                       │       │
│   │ 5. primitive_index++                            │       │
│   └────────────────────────────────────────────────┘       │
│                                                              │
│   When all primitives complete:                             │
│     Transition: EXECUTING → COMPLETED                       │
└────────────────────┬────────────────────────────────────────┘
                     ↓
┌─────────────────────────────────────────────────────────────┐
│ TEE: _handle_completed_state()                               │
│   • Mark task as completed                                   │
│   • Complete progress tracking                               │
│   • Check for more tasks:                                    │
│      - Yes: → VALIDATING                                     │
│      - No:  → IDLE (ready for next mission)                  │
└─────────────────────────────────────────────────────────────┘