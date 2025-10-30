#!/usr/bin/env python3
"""
Verification script to check TEE installation and imports.
"""

import sys


def check_imports():
    """Check all TEE imports"""
    print("=" * 60)
    print("TEE COMPONENT VERIFICATION")
    print("=" * 60)
    print()
    
    errors = []
    
    # Core components
    print("Checking core components...")
    try:
        from task_execution.core.state_machine import StateMachine, MissionState
        print("  ✓ State Machine")
    except ImportError as e:
        print(f"  ✗ State Machine: {e}")
        errors.append(str(e))
    
    try:
        from task_execution.core.task_queue import TaskQueue, PrioritizedTask
        print("  ✓ Task Queue")
    except ImportError as e:
        print(f"  ✗ Task Queue: {e}")
        errors.append(str(e))
    
    try:
        from task_execution.core.task_validator import TaskValidator
        print("  ✓ Task Validator")
    except ImportError as e:
        print(f"  ✗ Task Validator: {e}")
        errors.append(str(e))
    
    print()
    
    # Executors
    print("Checking executors...")
    try:
        from task_execution.executors.waypoint_executor import WaypointExecutor
        print("  ✓ Waypoint Executor")
    except ImportError as e:
        print(f"  ✗ Waypoint Executor: {e}")
        errors.append(str(e))
    
    try:
        from task_execution.executors.survey_executor import SurveyExecutor
        print("  ✓ Survey Executor")
    except ImportError as e:
        print(f"  ✗ Survey Executor: {e}")
        errors.append(str(e))
    
    try:
        from task_execution.executors.search_executor import SearchExecutor
        print("  ✓ Search Executor")
    except ImportError as e:
        print(f"  ✗ Search Executor: {e}")
        errors.append(str(e))
    
    print()
    
    # Monitors
    print("Checking monitors...")
    try:
        from task_execution.monitors.battery_monitor import BatteryMonitor
        print("  ✓ Battery Monitor")
    except ImportError as e:
        print(f"  ✗ Battery Monitor: {e}")
        errors.append(str(e))
    
    try:
        from task_execution.monitors.gps_monitor import GPSMonitor
        print("  ✓ GPS Monitor")
    except ImportError as e:
        print(f"  ✗ GPS Monitor: {e}")
        errors.append(str(e))
    
    try:
        from task_execution.monitors.health_monitor import HealthMonitor
        print("  ✓ Health Monitor")
    except ImportError as e:
        print(f"  ✗ Health Monitor: {e}")
        errors.append(str(e))
    
    try:
        from task_execution.monitors.progress_monitor import ProgressMonitor
        print("  ✓ Progress Monitor")
    except ImportError as e:
        print(f"  ✗ Progress Monitor: {e}")
        errors.append(str(e))
    
    print()
    
    # Main node
    print("Checking main TEE node...")
    try:
        from task_execution.tee_node import TaskExecutionEngineNode
        print("  ✓ TEE Node")
    except ImportError as e:
        print(f"  ✗ TEE Node: {e}")
        errors.append(str(e))
    
    print()
    
    # Dependencies
    print("Checking Python dependencies...")
    try:
        import networkx
        print(f"  ✓ networkx (version {networkx.__version__})")
    except ImportError:
        print("  ✗ networkx (not installed)")
        errors.append("networkx not installed")
    
    try:
        import shapely
        print(f"  ✓ shapely (version {shapely.__version__})")
    except ImportError:
        print("  ✗ shapely (not installed)")
        errors.append("shapely not installed")
    
    try:
        import numpy
        print(f"  ✓ numpy (version {numpy.__version__})")
    except ImportError:
        print("  ✗ numpy (not installed)")
        errors.append("numpy not installed")
    
    try:
        import yaml
        print("  ✓ yaml")
    except ImportError:
        print("  ✗ yaml (not installed)")
        errors.append("yaml not installed")
    
    print()
    print("=" * 60)
    
    if errors:
        print(f"FAILED: {len(errors)} errors found")
        print()
        print("Errors:")
        for error in errors:
            print(f"  - {error}")
        return False
    else:
        print("SUCCESS: All components verified!")
        return True


def main():
    """Main entry point"""
    success = check_imports()
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
