#!/bin/bash

# Critical Fix Verification Script
# Checks if the mission completion fix is actually loaded

cat << 'EOF'

╔════════════════════════════════════════════════════════════════════════╗
║              🔍 CRITICAL: Fix Not Yet Active!                          ║
╚════════════════════════════════════════════════════════════════════════╝

❌ PROBLEM IDENTIFIED:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

You're still running the OLD code!

Evidence from your logs:
  Mission 1: ✅ Allocated to drone_0
  Mission 2: ✅ Executed (but Squadron Manager not notified)
  Mission 3: ❌ "No available drones" - drone stuck with task=mission1
  Mission 4: ❌ "No available drones" - drone STILL stuck with task=mission1

This is EXACTLY the bug we fixed, but the fix isn't loaded yet because:
  ❌ You restarted Squadron Manager (good!)
  ❌ You did NOT restart complete_system.launch (contains TEE!)

🔧 THE FIX EXISTS BUT ISN'T RUNNING:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

We modified: src/task_execution/task_execution/tee_node.py
We rebuilt: ✅ task_execution package
Problem: TEE is still running from the OLD install directory!

🚨 YOU MUST RESTART COMPLETE_SYSTEM.LAUNCH:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

STEP 1: Stop complete_system.launch
   → Terminal 2: Press Ctrl+C

STEP 2: Source the rebuilt workspace
   → cd ~/multi_drone_ws
   → source install/setup.bash

STEP 3: Restart complete_system.launch
   → Terminal 2: ros2 launch as2_ardu_msn complete_system.launch.py

STEP 4: Stop and restart Squadron Manager (to ensure clean state)
   → Terminal 3: Press Ctrl+C
   → source ~/multi_drone_ws/install/setup.bash
   → ros2 launch squadron_manager squadron_manager.launch.py num_drones:=1

STEP 5: Wait 10 seconds for initialization

STEP 6: Test again with multiple missions:
   → ./scripts/send_squadron_mission.sh 10 10 15
   → (wait 70 seconds)
   → ./scripts/send_squadron_mission.sh 20 20 20

📊 HOW TO VERIFY THE FIX IS LOADED:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

After mission 1 completes, look for this in TEE logs (Terminal 2):
  ✅ [INFO] Entering COMPLETED state
  ✅ [INFO] Published completion status for mission: squadron_test_XXX
  ✅ [INFO] Task squadron_test_XXX marked as completed

And this in Squadron Manager logs (Terminal 3):
  ✅ [INFO] ✓ Mission squadron_test_XXX completed by drone_0 - 
           clearing task assignment
  ✅ [INFO] Drone drone_0 updated: state=BUSY->AVAILABLE

If you DON'T see "Published completion status", the old code is still running!

🎯 EXPECTED BEHAVIOR AFTER RESTART:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Mission 1:
  [INFO] Allocated task squadron_test_AAA to drone drone_0
  [INFO] Sent mission squadron_test_AAA to drone_0
  (60 seconds later)
  [INFO] ✓ Mission squadron_test_AAA completed by drone_0
  [INFO] Drone drone_0 updated: state=BUSY->AVAILABLE

Mission 2:
  [INFO] Received mission: squadron_test_BBB
  [INFO] Allocated task squadron_test_BBB to drone drone_0  ← SUCCESS!
  [INFO] Sent mission squadron_test_BBB to drone_0

Mission 3:
  [INFO] Received mission: squadron_test_CCC
  [INFO] Allocated task squadron_test_CCC to drone drone_0  ← SUCCESS!
  [INFO] Sent mission squadron_test_CCC to drone_0

⚠️  CRITICAL: BOTH SYSTEMS MUST BE RESTARTED!
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

The fix is in TWO packages:
  1. task_execution (TEE) - Publishes completion with mission_id
  2. squadron_manager - Receives and processes completion

You restarted #2 but not #1. Must restart BOTH!

📝 WHY THIS HAPPENS:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

When you run:
  ros2 launch as2_ardu_msn complete_system.launch.py

It loads TEE from: ~/multi_drone_ws/install/task_execution/

When we rebuild with colcon build, the NEW code goes to install/, but
the RUNNING process still has the OLD code in memory.

You must STOP and RESTART to load the new code!

🚀 DO THIS NOW:
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

Terminal 2 (complete_system.launch):
  1. Ctrl+C
  2. source ~/multi_drone_ws/install/setup.bash
  3. ros2 launch as2_ardu_msn complete_system.launch.py

Terminal 3 (squadron_manager):
  1. Ctrl+C
  2. source ~/multi_drone_ws/install/setup.bash
  3. ros2 launch squadron_manager squadron_manager.launch.py num_drones:=1

Then test again! The fix WILL work once both are restarted! 🎉

EOF
