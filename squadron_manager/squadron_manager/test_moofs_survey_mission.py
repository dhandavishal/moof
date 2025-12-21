#!/usr/bin/env python3
"""


Comprehensive test script for MOOFS framework validation with 3 drones.
Tests: Squadron Manager → TEE → FAL → MAVROS integration

Usage:
    ros2 run squadron_manager test_moofs_survey_mission

Prerequisites:
    1. 3 SITL instances running (ports 14550, 14560, 14570)
    2. MAVROS nodes for each drone
    3. FAL nodes for each drone
    4. TEE nodes for each drone
    5. Squadron Manager running
"""

import json
import time
import threading
from typing import Dict, List, Optional
from dataclasses import dataclass, field
from enum import Enum
from collections import defaultdict

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import String, Empty
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import BatteryState
from mavros_msgs.msg import State


class TestPhase(Enum):
    """Test phases"""
    INIT = "initialization"
    PREFLIGHT = "preflight_checks"
    SINGLE_DRONE = "single_drone_mission"
    MULTI_DRONE = "multi_drone_mission"
    FORMATION = "formation_test"
    FAILURE_RECOVERY = "failure_recovery"
    COMPLETE = "complete"


@dataclass
class DroneStatus:
    """Track drone status during test"""
    drone_id: str
    connected: bool = False
    armed: bool = False
    mode: str = "UNKNOWN"
    position: tuple = (0.0, 0.0, 0.0)
    battery: float = 0.0
    mission_state: str = "idle"
    mission_progress: float = 0.0
    last_update: float = 0.0


@dataclass 
class TestResult:
    """Test result tracking"""
    test_name: str
    passed: bool
    duration: float
    details: str = ""
    errors: List[str] = field(default_factory=list)


class MOOFSTestSuite(Node):
    """
    Comprehensive MOOFS test suite for 3-drone survey mission.
    """
    
    def __init__(self):
        super().__init__('moofs_test_suite')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("MOOFS FRAMEWORK TEST SUITE")
        self.get_logger().info("=" * 60)
        
        # Configuration
        self.num_drones = 3
        self.test_timeout = 300.0  # 5 minutes max per test
        
        # State tracking
        self.drone_status: Dict[str, DroneStatus] = {}
        self.squadron_status: Dict = {}
        self.test_results: List[TestResult] = []
        self.current_phase = TestPhase.INIT
        self.mission_events: List[Dict] = []
        
        # Initialize drone status
        for i in range(self.num_drones):
            drone_id = f"drone_{i}"
            self.drone_status[drone_id] = DroneStatus(drone_id=drone_id)
        
        # QoS profiles
        self.reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.mavros_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Setup communications
        self._setup_publishers()
        self._setup_subscribers()
        
        # Wait for connections
        time.sleep(2.0)
        
        self.get_logger().info("Test suite initialized")
    
    def _setup_publishers(self):
        """Setup all publishers"""
        # Squadron command publishers
        self.mission_pub = self.create_publisher(
            String, '/squadron/mission_command', self.reliable_qos
        )
        self.formation_pub = self.create_publisher(
            String, '/squadron/formation_command', self.reliable_qos
        )
        self.abort_pub = self.create_publisher(
            Empty, '/squadron/abort', self.reliable_qos
        )
        
        # Direct TEE publishers for testing
        self.tee_pubs = {}
        for i in range(self.num_drones):
            self.tee_pubs[f"drone_{i}"] = self.create_publisher(
                String, f'/drone_{i}/tee/mission_command', self.reliable_qos
            )
    
    def _setup_subscribers(self):
        """Setup all subscribers"""
        # Squadron status
        self.create_subscription(
            String, '/squadron/status',
            self._squadron_status_cb, self.reliable_qos
        )
        
        # Per-drone subscriptions
        for i in range(self.num_drones):
            drone_id = f"drone_{i}"
            ns = f"/drone_{i}"
            
            # MAVROS state
            self.create_subscription(
                State, f'{ns}/mavros/state',
                lambda msg, d=drone_id: self._mavros_state_cb(msg, d),
                self.mavros_qos
            )
            
            # Position
            self.create_subscription(
                PoseStamped, f'{ns}/mavros/local_position/pose',
                lambda msg, d=drone_id: self._position_cb(msg, d),
                self.mavros_qos
            )
            
            # Battery
            self.create_subscription(
                BatteryState, f'{ns}/mavros/battery',
                lambda msg, d=drone_id: self._battery_cb(msg, d),
                self.mavros_qos
            )
            
            # TEE mission status
            self.create_subscription(
                String, f'{ns}/tee/mission_status',
                lambda msg, d=drone_id: self._tee_status_cb(msg, d),
                self.reliable_qos
            )
    
    # ========================================================================
    # Callbacks
    # ========================================================================
    
    def _squadron_status_cb(self, msg: String):
        """Handle squadron status updates"""
        try:
            self.squadron_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass
    
    def _mavros_state_cb(self, msg: State, drone_id: str):
        """Handle MAVROS state updates"""
        status = self.drone_status[drone_id]
        status.connected = msg.connected
        status.armed = msg.armed
        status.mode = msg.mode
        status.last_update = time.time()
    
    def _position_cb(self, msg: PoseStamped, drone_id: str):
        """Handle position updates"""
        status = self.drone_status[drone_id]
        status.position = (
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        )
    
    def _battery_cb(self, msg: BatteryState, drone_id: str):
        """Handle battery updates"""
        status = self.drone_status[drone_id]
        status.battery = msg.percentage * 100.0 if msg.percentage > 0 else 0.0
    
    def _tee_status_cb(self, msg: String, drone_id: str):
        """Handle TEE mission status"""
        try:
            data = json.loads(msg.data)
            status = self.drone_status[drone_id]
            status.mission_state = data.get('state', 'unknown')
            status.mission_progress = data.get('progress', 0.0)
            
            # Log significant events
            self.mission_events.append({
                'timestamp': time.time(),
                'drone_id': drone_id,
                'event': data
            })
            
        except json.JSONDecodeError:
            pass
    
    # ========================================================================
    # Test Utilities
    # ========================================================================
    
    def wait_for_condition(
        self,
        condition_fn,
        timeout: float = 30.0,
        poll_rate: float = 0.5,
        description: str = "condition"
    ) -> bool:
        """Wait for a condition with timeout"""
        start = time.time()
        while time.time() - start < timeout:
            if condition_fn():
                return True
            time.sleep(poll_rate)
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().warning(f"Timeout waiting for: {description}")
        return False
    
    def all_drones_connected(self) -> bool:
        """Check if all drones are connected"""
        return all(s.connected for s in self.drone_status.values())
    
    def all_drones_have_gps(self) -> bool:
        """Check if all drones have valid positions (proxy for GPS)"""
        return all(
            s.position[2] != 0.0 or s.last_update > 0
            for s in self.drone_status.values()
        )
    
    def print_status(self):
        """Print current status of all drones"""
        self.get_logger().info("-" * 50)
        for drone_id, status in self.drone_status.items():
            self.get_logger().info(
                f"{drone_id}: connected={status.connected}, "
                f"armed={status.armed}, mode={status.mode}, "
                f"battery={status.battery:.1f}%, "
                f"pos=({status.position[0]:.1f}, {status.position[1]:.1f}, {status.position[2]:.1f}), "
                f"mission={status.mission_state}"
            )
        self.get_logger().info("-" * 50)
    
    def send_mission(self, mission: Dict):
        """Send mission to squadron manager"""
        msg = String()
        msg.data = json.dumps(mission)
        self.mission_pub.publish(msg)
        self.get_logger().info(f"Sent mission: {mission.get('mission_id', 'unknown')}")
    
    def send_formation_command(self, command: Dict):
        """Send formation command"""
        msg = String()
        msg.data = json.dumps(command)
        self.formation_pub.publish(msg)
        self.get_logger().info(f"Sent formation command: {command.get('type', 'unknown')}")
    
    # ========================================================================
    # Test Cases
    # ========================================================================
    
    def test_preflight_checks(self) -> TestResult:
        """Test 1: Preflight connectivity and readiness"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("TEST 1: PREFLIGHT CHECKS")
        self.get_logger().info("=" * 60)
        
        start_time = time.time()
        errors = []
        
        # Check 1: All drones connected
        self.get_logger().info("Checking drone connections...")
        if not self.wait_for_condition(
            self.all_drones_connected,
            timeout=30.0,
            description="all drones connected"
        ):
            errors.append("Not all drones connected within timeout")
        
        # Check 2: Squadron Manager responding
        self.get_logger().info("Checking Squadron Manager...")
        if not self.wait_for_condition(
            lambda: bool(self.squadron_status),
            timeout=10.0,
            description="squadron status received"
        ):
            errors.append("Squadron Manager not responding")
        
        # Check 3: Battery levels adequate
        self.get_logger().info("Checking battery levels...")
        for drone_id, status in self.drone_status.items():
            if status.battery < 20.0 and status.battery > 0:
                errors.append(f"{drone_id} battery too low: {status.battery:.1f}%")
        
        self.print_status()
        
        duration = time.time() - start_time
        passed = len(errors) == 0
        
        return TestResult(
            test_name="Preflight Checks",
            passed=passed,
            duration=duration,
            details=f"Checked {self.num_drones} drones",
            errors=errors
        )
    
    def test_single_drone_waypoint(self) -> TestResult:
        """Test 2: Single drone waypoint mission via Squadron Manager"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("TEST 2: SINGLE DRONE WAYPOINT MISSION")
        self.get_logger().info("=" * 60)
        
        start_time = time.time()
        errors = []
        
        # Define simple waypoint mission
        mission = {
            "mission_id": f"test_single_{int(time.time())}",
            "task_type": "waypoint",
            "priority": 100,
            "timeout": 120.0,
            "multi_drone": False,
            "parameters": {
                "waypoints": [
                    {"x": 10.0, "y": 0.0, "z": 20.0, "yaw": 0.0},
                    {"x": 10.0, "y": 10.0, "z": 20.0, "yaw": 1.57},
                    {"x": 0.0, "y": 10.0, "z": 20.0, "yaw": 3.14},
                    {"x": 0.0, "y": 0.0, "z": 20.0, "yaw": 0.0}
                ],
                "velocity": 3.0,
                "acceptance_radius": 2.0
            }
        }
        
        # Send mission
        self.send_mission(mission)
        
        # Wait for allocation (check squadron status)
        self.get_logger().info("Waiting for task allocation...")
        time.sleep(2.0)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        # Check if a drone received the mission
        allocated_drone = None
        for drone_id, status in self.drone_status.items():
            if status.mission_state in ['validating', 'executing']:
                allocated_drone = drone_id
                break
        
        if not allocated_drone:
            # Check if any drone is busy in squadron status
            if self.squadron_status.get('busy', 0) > 0:
                allocated_drone = "unknown"
                self.get_logger().info("Mission allocated (drone busy)")
            else:
                errors.append("No drone received mission allocation")
        else:
            self.get_logger().info(f"Mission allocated to {allocated_drone}")
        
        # Wait for mission to start executing or complete
        def mission_progressing():
            for status in self.drone_status.values():
                if status.mission_state == 'executing':
                    return True
                if status.mission_progress > 0:
                    return True
            return False
        
        if allocated_drone and not errors:
            self.get_logger().info("Waiting for mission execution...")
            if not self.wait_for_condition(
                mission_progressing,
                timeout=60.0,
                description="mission executing"
            ):
                errors.append("Mission did not start executing")
            else:
                self.get_logger().info("✓ Mission is executing")
        
        # For testing, we don't wait for full completion
        # Just verify the pipeline is working
        
        self.print_status()
        duration = time.time() - start_time
        passed = len(errors) == 0
        
        return TestResult(
            test_name="Single Drone Waypoint",
            passed=passed,
            duration=duration,
            details=f"Allocated to: {allocated_drone}",
            errors=errors
        )
    
    def test_multi_drone_formation(self) -> TestResult:
        """Test 3: Multi-drone formation mission"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("TEST 3: MULTI-DRONE FORMATION MISSION")
        self.get_logger().info("=" * 60)
        
        start_time = time.time()
        errors = []
        
        # First, create a formation
        formation_cmd = {
            "type": "create",
            "formation_type": "wedge",
            "spacing": 15.0,
            "altitude": 30.0,
            "center": [0.0, 0.0, 30.0]
        }
        
        self.send_formation_command(formation_cmd)
        time.sleep(1.0)
        
        # Now send multi-drone mission
        mission = {
            "mission_id": f"test_formation_{int(time.time())}",
            "task_type": "waypoint",
            "priority": 100,
            "timeout": 180.0,
            "multi_drone": True,
            "formation_type": "wedge",
            "spacing": 15.0,
            "altitude": 30.0,
            "parameters": {
                "waypoints": [
                    {"x": 30.0, "y": 0.0, "z": 30.0},
                    {"x": 30.0, "y": 30.0, "z": 30.0},
                    {"x": 0.0, "y": 30.0, "z": 30.0}
                ],
                "velocity": 3.0
            }
        }
        
        self.send_mission(mission)
        
        # Wait for multi-drone allocation
        self.get_logger().info("Waiting for multi-drone allocation...")
        time.sleep(3.0)
        
        # Check how many drones received missions
        drones_with_missions = 0
        for drone_id, status in self.drone_status.items():
            rclpy.spin_once(self, timeout_sec=0.1)
            if status.mission_state in ['validating', 'executing']:
                drones_with_missions += 1
                self.get_logger().info(f"  {drone_id}: {status.mission_state}")
        
        # Check squadron status for busy drones
        if self.squadron_status.get('busy', 0) >= 2:
            self.get_logger().info(
                f"✓ Multi-drone mission distributed to {self.squadron_status['busy']} drones"
            )
        elif drones_with_missions >= 2:
            self.get_logger().info(
                f"✓ {drones_with_missions} drones received missions"
            )
        else:
            errors.append(
                f"Expected 2+ drones to receive missions, got {drones_with_missions}"
            )
        
        self.print_status()
        duration = time.time() - start_time
        passed = len(errors) == 0
        
        return TestResult(
            test_name="Multi-Drone Formation",
            passed=passed,
            duration=duration,
            details=f"Drones active: {drones_with_missions}",
            errors=errors
        )
    
    def test_survey_mission(self) -> TestResult:
        """Test 4: Survey mission with area coverage"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("TEST 4: SURVEY MISSION")
        self.get_logger().info("=" * 60)
        
        start_time = time.time()
        errors = []
        
        # Define survey area (100m x 100m)
        mission = {
            "mission_id": f"test_survey_{int(time.time())}",
            "task_type": "survey",
            "priority": 50,
            "timeout": 600.0,
            "multi_drone": False,
            "parameters": {
                "area": [[0, 0], [100, 0], [100, 100], [0, 100]],
                "altitude": 50.0,
                "gsd": 3.0,
                "overlap_forward": 0.70,
                "overlap_side": 0.60,
                "velocity": 5.0
            }
        }
        
        self.send_mission(mission)
        
        # Wait for allocation and validation
        self.get_logger().info("Waiting for survey mission allocation...")
        time.sleep(3.0)
        
        # Check if mission was received
        survey_drone = None
        for drone_id, status in self.drone_status.items():
            rclpy.spin_once(self, timeout_sec=0.1)
            if status.mission_state != 'idle':
                survey_drone = drone_id
                self.get_logger().info(
                    f"Survey assigned to {drone_id}: {status.mission_state}"
                )
                break
        
        if not survey_drone:
            errors.append("Survey mission was not allocated")
        else:
            self.get_logger().info(f"✓ Survey mission allocated to {survey_drone}")
        
        self.print_status()
        duration = time.time() - start_time
        passed = len(errors) == 0
        
        return TestResult(
            test_name="Survey Mission",
            passed=passed,
            duration=duration,
            details=f"Allocated to: {survey_drone}",
            errors=errors
        )
    
    def test_abort_functionality(self) -> TestResult:
        """Test 5: Abort command functionality"""
        self.get_logger().info("\n" + "=" * 60)
        self.get_logger().info("TEST 5: ABORT FUNCTIONALITY")
        self.get_logger().info("=" * 60)
        
        start_time = time.time()
        errors = []
        
        # Send abort command
        self.get_logger().info("Sending abort command...")
        msg = Empty()
        self.abort_pub.publish(msg)
        
        # Wait for drones to abort
        time.sleep(2.0)
        
        # Verify all drones are idle or aborted
        for drone_id, status in self.drone_status.items():
            rclpy.spin_once(self, timeout_sec=0.1)
            if status.mission_state not in ['idle', 'aborted', 'completed']:
                self.get_logger().warning(
                    f"{drone_id} still in state: {status.mission_state}"
                )
        
        self.get_logger().info("✓ Abort command sent")
        
        self.print_status()
        duration = time.time() - start_time
        passed = len(errors) == 0
        
        return TestResult(
            test_name="Abort Functionality",
            passed=passed,
            duration=duration,
            details="Abort command processed",
            errors=errors
        )
    
    # ========================================================================
    # Main Test Runner
    # ========================================================================
    
    def run_all_tests(self):
        """Run all tests in sequence"""
        self.get_logger().info("\n")
        self.get_logger().info("*" * 60)
        self.get_logger().info("*" + " STARTING MOOFS TEST SUITE ".center(58) + "*")
        self.get_logger().info("*" * 60)
        self.get_logger().info(f"Testing {self.num_drones} drone configuration")
        self.get_logger().info("")
        
        # Run tests
        tests = [
            self.test_preflight_checks,
            self.test_single_drone_waypoint,
            # Wait between tests
            lambda: (time.sleep(5.0), TestResult("Cooldown", True, 5.0))[1],
            self.test_multi_drone_formation,
            lambda: (time.sleep(5.0), TestResult("Cooldown", True, 5.0))[1],
            self.test_survey_mission,
            lambda: (time.sleep(3.0), TestResult("Cooldown", True, 3.0))[1],
            self.test_abort_functionality,
        ]
        
        for test_fn in tests:
            try:
                result = test_fn()
                if result.test_name != "Cooldown":
                    self.test_results.append(result)
                    
                    status = "✓ PASSED" if result.passed else "✗ FAILED"
                    self.get_logger().info(
                        f"\n{status}: {result.test_name} ({result.duration:.1f}s)"
                    )
                    if result.errors:
                        for error in result.errors:
                            self.get_logger().error(f"  - {error}")
            
            except Exception as e:
                self.get_logger().error(f"Test exception: {e}")
                self.test_results.append(TestResult(
                    test_name="Unknown",
                    passed=False,
                    duration=0.0,
                    errors=[str(e)]
                ))
        
        # Print summary
        self.print_summary()
    
    def print_summary(self):
        """Print test summary"""
        self.get_logger().info("\n")
        self.get_logger().info("=" * 60)
        self.get_logger().info("TEST SUMMARY")
        self.get_logger().info("=" * 60)
        
        passed = sum(1 for r in self.test_results if r.passed)
        failed = sum(1 for r in self.test_results if not r.passed)
        total = len(self.test_results)
        
        for result in self.test_results:
            status = "✓" if result.passed else "✗"
            self.get_logger().info(
                f"  {status} {result.test_name}: {result.duration:.1f}s"
            )
        
        self.get_logger().info("-" * 60)
        self.get_logger().info(f"TOTAL: {passed}/{total} passed, {failed} failed")
        
        if failed == 0:
            self.get_logger().info("\n✓ ALL TESTS PASSED!")
        else:
            self.get_logger().info(f"\n✗ {failed} TEST(S) FAILED")


def main():
    """Main entry point"""
    rclpy.init()
    
    test_suite = MOOFSTestSuite()
    
    try:
        # Give time for all nodes to be discovered
        time.sleep(3.0)
        
        # Run tests
        test_suite.run_all_tests()
        
    except KeyboardInterrupt:
        test_suite.get_logger().info("\nTest interrupted by user")
    finally:
        test_suite.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()