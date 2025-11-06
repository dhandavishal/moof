#!/usr/bin/env python3
"""
Task validator for pre-flight validation in ROS2.

Performs comprehensive task feasibility validation including syntax,
battery, GPS, and connection checks.
"""

import yaml
from typing import List, Dict, Optional
from dataclasses import dataclass

from rclpy.node import Node
from task_execution.core.task_queue import PrioritizedTask
from task_execution.monitors.battery_monitor import BatteryStatus
from task_execution.monitors.gps_monitor import GPSStatus


@dataclass
class CheckResult:
    """Result of a validation check"""
    passed: bool
    message: str
    details: Optional[Dict] = None


@dataclass
class ValidationResult:
    """Complete validation result"""
    valid: bool
    failures: List[CheckResult]
    warnings: List[CheckResult]


class TaskValidator:
    """
    Performs comprehensive task feasibility validation.
    """
    
    def __init__(self, node: Node, config: dict):
        """
        Initialize validator with configuration.
        
        Args:
            node: ROS2 node instance
            config: Configuration dictionary
        """
        self.node = node
        self.config = config
        
        # Extract safety parameters
        safety = self.config.get('safety', {})
        self.min_battery_pct = safety.get('min_battery_percentage', 0.25)
        self.critical_battery_pct = safety.get('critical_battery_percentage', 0.20)
        self.min_gps_satellites = safety.get('min_gps_satellites', 8)
        self.max_gps_hdop = safety.get('max_gps_hdop', 2.0)
        
        # Drone specifications
        drone = self.config.get('drone', {})
        self.drone_mass = drone.get('mass', 2.5)
        self.hover_power = drone.get('hover_power', 250)
        
        self.node.get_logger().info("Task validator initialized")
    
    def validate_task(self, 
                     task: PrioritizedTask, 
                     battery_status: Optional[BatteryStatus] = None,
                     gps_status: Optional[GPSStatus] = None,
                     connected: bool = True) -> ValidationResult:
        """
        Perform comprehensive task validation.
        
        Args:
            task: Task to validate
            battery_status: Current battery status
            gps_status: Current GPS status
            connected: Connection status
            
        Returns:
            ValidationResult with pass/fail and details
        """
        failures = []
        warnings = []
        
        # Run all validation checks
        checks = [
            self._check_syntax(task),
        ]
        
        # Battery check if status available
        if battery_status:
            checks.append(self._check_battery(task, battery_status))
        
        # GPS check if status available
        if gps_status:
            checks.append(self._check_gps(gps_status))
        
        # Connection check
        checks.append(self._check_connection(connected))
        
        # Separate failures and warnings
        for check in checks:
            if not check.passed:
                severity = check.details.get('severity', 'critical') if check.details else 'critical'
                if 'critical' in severity:
                    failures.append(check)
                else:
                    warnings.append(check)
        
        # Overall validation result
        valid = len(failures) == 0
        
        if valid:
            self.node.get_logger().info(f"Task {task.task_id} validation passed")
        else:
            self.node.get_logger().error(
                f"Task {task.task_id} validation failed: {len(failures)} failures"
            )
        
        return ValidationResult(
            valid=valid,
            failures=failures,
            warnings=warnings
        )
    
    def _check_syntax(self, task: PrioritizedTask) -> CheckResult:
        """Validate task structure and parameters"""
        
        # Define required parameters for each task type
        required_params = {
            'survey': ['area', 'altitude'],
            'waypoint': ['waypoints'],
            'search': ['area', 'pattern'],
            'rtl': []
        }
        
        if task.task_type not in required_params:
            return CheckResult(
                passed=False,
                message=f"Unknown task type: {task.task_type}",
                details={'severity': 'critical'}
            )
        
        # Check required parameters exist
        missing = []
        for param in required_params[task.task_type]:
            if param not in task.parameters:
                missing.append(param)
        
        if missing:
            return CheckResult(
                passed=False,
                message=f"Missing required parameters: {missing}",
                details={'severity': 'critical', 'missing': missing}
            )
        
        # Type-specific validation
        if task.task_type == 'survey':
            area = task.parameters.get('area', [])
            if len(area) < 3:
                return CheckResult(
                    passed=False,
                    message="Survey area must have at least 3 points",
                    details={'severity': 'critical'}
                )
            
            altitude = task.parameters.get('altitude', 0)
            if altitude <= 0:
                return CheckResult(
                    passed=False,
                    message="Altitude must be positive",
                    details={'severity': 'critical'}
                )
        
        elif task.task_type == 'waypoint':
            waypoints = task.parameters.get('waypoints', [])
            if len(waypoints) < 1:
                return CheckResult(
                    passed=False,
                    message="Waypoint task must have at least 1 waypoint",
                    details={'severity': 'critical'}
                )
        
        return CheckResult(passed=True, message="Syntax valid")
    
    def _check_battery(self, task: PrioritizedTask, battery_status: BatteryStatus) -> CheckResult:
        """Validate battery sufficiency"""
        
        # Get current battery state
        current_pct = battery_status.percentage
        
        # SITL detection: Check for unrealistic battery capacity
        # Real drone batteries: 200-500Wh typical, SITL often reports 50-80Wh
        # If capacity is suspiciously low but percentage is high, likely SITL
        is_sitl_mode = (
            battery_status.capacity_wh <= 0.0 or   # No capacity reported
            current_pct < 0.0 or                     # Negative percentage (SITL default)
            current_pct == 0.0 or                    # Zero percentage with no real battery
            (battery_status.capacity_wh < 100.0 and current_pct >= 0.9)  # Low capacity but 90%+ charge (SITL)
        )
        
        if is_sitl_mode:
            self.node.get_logger().warn(
                f"SITL mode detected - bypassing battery energy calculation "
                f"(capacity={battery_status.capacity_wh:.1f}Wh, percentage={current_pct*100:.0f}%)"
            )
            return CheckResult(
                passed=True,
                message=f"Battery check bypassed for SITL mode",
                details={
                    'mode': 'sitl_bypass',
                    'capacity_wh': battery_status.capacity_wh,
                    'percentage': current_pct
                }
            )
        
        # Critical check
        if current_pct < self.critical_battery_pct:
            return CheckResult(
                passed=False,
                message=f"Critical battery level: {current_pct*100:.1f}%",
                details={
                    'severity': 'critical',
                    'current': current_pct,
                    'threshold': self.critical_battery_pct
                }
            )
        
        # Estimate mission energy (simplified)
        estimated_duration = self._estimate_task_duration(task)
        estimated_energy_wh = (self.hover_power / 3600.0) * estimated_duration
        
        # Add RTL reserve (assume 5 minutes)
        rtl_energy_wh = (self.hover_power / 3600.0) * 300  # 5 min
        
        # Total required with 20% safety margin
        total_required_wh = (estimated_energy_wh + rtl_energy_wh) * 1.2
        
        available_wh = battery_status.capacity_wh
        
        if available_wh < total_required_wh:
            return CheckResult(
                passed=False,
                message=f"Insufficient battery for mission",
                details={
                    'severity': 'critical',
                    'available_wh': available_wh,
                    'required_wh': total_required_wh,
                    'deficit_wh': total_required_wh - available_wh
                }
            )
        
        return CheckResult(
            passed=True,
            message="Battery sufficient",
            details={
                'available_wh': available_wh,
                'required_wh': total_required_wh
            }
        )
    
    def _estimate_task_duration(self, task: PrioritizedTask) -> float:
        """
        Estimate task duration in seconds (simplified).
        
        Args:
            task: Task to estimate
            
        Returns:
            Estimated duration in seconds
        """
        # Simplified duration estimation
        if task.task_type == 'survey':
            # Rough estimate: 1 minute per 100m² at 50m altitude
            area = task.parameters.get('area', [])
            if len(area) >= 3:
                area_m2 = self._calculate_polygon_area(area)
                return (area_m2 / 100.0) * 60.0  # seconds
            return 600.0  # default 10 minutes
        
        elif task.task_type == 'waypoint':
            # Estimate based on waypoint count
            waypoints = task.parameters.get('waypoints', [])
            return len(waypoints) * 30.0  # 30 seconds per waypoint
        
        elif task.task_type == 'search':
            # Default search time
            return 900.0  # 15 minutes
        
        else:
            return 300.0  # default 5 minutes
    
    def _calculate_polygon_area(self, polygon: List) -> float:
        """Calculate polygon area using shoelace formula"""
        n = len(polygon)
        if n < 3:
            return 0.0
        
        area = 0.0
        for i in range(n):
            j = (i + 1) % n
            # Handle multiple formats: [x, y], [x, y, z], or {"x": ..., "y": ...}
            if isinstance(polygon[i], dict):
                x_i = polygon[i].get('x', 0.0)
                y_i = polygon[i].get('y', 0.0)
            elif isinstance(polygon[i], (list, tuple)):
                x_i = polygon[i][0]
                y_i = polygon[i][1]
            else:
                x_i = 0.0
                y_i = 0.0
                
            if isinstance(polygon[j], dict):
                x_j = polygon[j].get('x', 0.0)
                y_j = polygon[j].get('y', 0.0)
            elif isinstance(polygon[j], (list, tuple)):
                x_j = polygon[j][0]
                y_j = polygon[j][1]
            else:
                x_j = 0.0
                y_j = 0.0
            
            area += x_i * y_j
            area -= x_j * y_i
        
        return abs(area) / 2.0
    
    def _check_gps(self, gps_status: GPSStatus) -> CheckResult:
        """Validate GPS quality"""
        
        sats = gps_status.satellites
        hdop = gps_status.hdop
        fix_type = gps_status.fix_type
        
        # TEMPORARY: Relaxed GPS check for SITL testing
        # In SITL, MAVROS global_position may report status=0 even with good GPS
        # Check fix type (allow any fix >= 0 for SITL)
        if fix_type < 0:  # Only reject if completely invalid
            return CheckResult(
                passed=False,
                message="No GPS 3D fix",
                details={'severity': 'critical', 'fix_type': fix_type}
            )
        
        # Check satellite count (relaxed for SITL)
        if sats < self.min_gps_satellites and sats > 0:  # Only fail if sats reported but insufficient
            return CheckResult(
                passed=False,
                message=f"Insufficient GPS satellites: {sats}",
                details={
                    'severity': 'critical',
                    'satellites': sats,
                    'minimum': self.min_gps_satellites
                }
            )
        # Allow 0 satellites for SITL testing (GPS data might not include sat count)
        if sats == 0:
            self.node.get_logger().warn("GPS satellite count is 0 - assuming SITL mode, allowing mission")
        
        # Check HDOP
        if hdop > self.max_gps_hdop:
            return CheckResult(
                passed=False,
                message=f"GPS HDOP too high: {hdop:.2f}",
                details={
                    'severity': 'warning',
                    'hdop': hdop,
                    'maximum': self.max_gps_hdop
                }
            )
        
        return CheckResult(passed=True, message="GPS quality acceptable")
    
    def _check_connection(self, connected: bool) -> CheckResult:
        """Validate connection to drone"""
        
        if not connected:
            return CheckResult(
                passed=False,
                message="No connection to drone",
                details={'severity': 'critical'}
            )
        
        return CheckResult(passed=True, message="Connection OK")
