#!/usr/bin/env python3
"""
Advanced Precision Position Controller (Headless)
Navigate to target position using odometry feedback.
This script is designed for command-line/subprocess execution.
"""

import sys
import math
import time
import numpy as np
from typing import Tuple, Optional, List, Dict
from dataclasses import dataclass, field
from enum import Enum
from collections import deque
import threading
import os

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from scipy.spatial.transform import Rotation as SciR


class ControllerState(Enum):
    """Controller state machine states"""
    IDLE = 0
    ROTATING_TO_TARGET = 1
    MOVING_TO_TARGET = 2
    FINAL_ADJUSTMENT = 3
    PULSE_CONTROL = 4
    REACHED = 5
    ERROR = 6
    PAUSED = 7


@dataclass
class PIDGains:
    """PID controller gains with enhanced monitoring"""
    kp: float = 0.0
    ki: float = 0.0
    kd: float = 0.0
    integral_limit: float = 1.0
    output_limit: float = 1.0
    
    # Performance tracking
    p_term: float = field(default=0.0, init=False)
    i_term: float = field(default=0.0, init=False)
    d_term: float = field(default=0.0, init=False)
    total_output: float = field(default=0.0, init=False)


@dataclass
class RobotPose:
    """Robot pose in 2D space with enhanced tracking"""
    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0
    
    # Velocity tracking
    vx: float = field(default=0.0, init=False)
    vy: float = field(default=0.0, init=False)
    omega: float = field(default=0.0, init=False)
    
    def distance_to(self, target_x: float, target_y: float) -> float:
        """Calculate Euclidean distance to target position"""
        return math.sqrt((target_x - self.x)**2 + (target_y - self.y)**2)
    
    def angle_to(self, target_x: float, target_y: float) -> float:
        """Calculate angle to target position in robot's world frame"""
        return math.atan2(target_y - self.y, target_x - self.x)


@dataclass
class PerformanceMetrics:
    """Comprehensive performance metrics tracking"""
    # Timing metrics
    start_time: float = 0.0
    total_time: float = 0.0
    settling_time: float = 0.0
    
    # Error metrics
    max_position_error: float = 0.0
    max_angular_error: float = 0.0
    rms_position_error: float = 0.0
    rms_angular_error: float = 0.0
    
    # Performance indicators
    overshoot_count: int = 0
    oscillation_count: int = 0
    average_speed: float = 0.0
    distance_traveled: float = 0.0
    
    # Control effort
    total_control_effort: float = 0.0
    peak_linear_velocity: float = 0.0
    peak_angular_velocity: float = 0.0


class EnhancedPIDController:
    """Enhanced PID controller with comprehensive monitoring and analytics"""
    
    def __init__(self, gains: PIDGains, name: str = "PID"):
        self.gains = gains
        self.name = name
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        
        # Enhanced tracking
        self.error_history = deque(maxlen=1000)
        self.output_history = deque(maxlen=1000)
        self.derivative_history = deque(maxlen=100)
        
        # Performance metrics
        self.peak_error = 0.0
        self.settling_criteria = 0.02  # 2% settling band
        self.settled_time = None
        self.oscillation_count = 0
        self.last_error_sign = 0
    
    def reset(self):
        """Reset controller state and clear history"""
        self.integral = 0.0
        self.prev_error = 0.0
        self.prev_time = None
        self.error_history.clear()
        self.output_history.clear()
        self.derivative_history.clear()
        self.peak_error = 0.0
        self.settled_time = None
        self.oscillation_count = 0
        self.last_error_sign = 0
    
    def compute(self, error: float, current_time: float) -> float:
        """Compute PID control output with enhanced analytics"""
        # Track error history
        self.error_history.append((current_time, error))
        
        # Update peak error
        self.peak_error = max(self.peak_error, abs(error))
        
        # Check for oscillations
        current_error_sign = np.sign(error)
        if self.last_error_sign != 0 and current_error_sign != self.last_error_sign:
            self.oscillation_count += 1
        self.last_error_sign = current_error_sign
        
        # Check settling
        if abs(error) < self.settling_criteria and self.settled_time is None:
            self.settled_time = current_time
        elif abs(error) >= self.settling_criteria:
            self.settled_time = None
        
        if self.prev_time is None:
            self.prev_time = current_time
            self.prev_error = error
            output = self.gains.kp * error
            self.gains.p_term = output
            self.gains.i_term = 0.0
            self.gains.d_term = 0.0
            self.gains.total_output = output
            return output
        
        dt = current_time - self.prev_time
        if dt <= 0:
            return self.gains.kp * error
        
        # Proportional term
        self.gains.p_term = self.gains.kp * error
        
        # Integral term with anti-windup
        self.integral += error * dt
        self.integral = np.clip(self.integral, 
                               -self.gains.integral_limit, 
                               self.gains.integral_limit)
        self.gains.i_term = self.gains.ki * self.integral
        
        # Derivative term with filtering
        derivative = (error - self.prev_error) / dt
        self.derivative_history.append(derivative)
        
        # Apply simple moving average filter to derivative
        if len(self.derivative_history) > 5:
            filtered_derivative = np.mean(list(self.derivative_history)[-5:])
        else:
            filtered_derivative = derivative
            
        self.gains.d_term = self.gains.kd * filtered_derivative
        
        # Compute total output with saturation
        output = self.gains.p_term + self.gains.i_term + self.gains.d_term
        output = np.clip(output, -self.gains.output_limit, self.gains.output_limit)
        self.gains.total_output = output
        
        # Track output history
        self.output_history.append((current_time, output))
        
        # Update state
        self.prev_error = error
        self.prev_time = current_time
        
        return output


class DataCollector:
    """Comprehensive data collection system for performance analysis"""
    
    def __init__(self, max_samples: int = 5000):
        self.max_samples = max_samples
        
        # Time series data
        self.timestamps = deque(maxlen=max_samples)
        self.position_x = deque(maxlen=max_samples)
        self.position_y = deque(maxlen=max_samples)
        self.position_error = deque(maxlen=max_samples)
        self.angular_error = deque(maxlen=max_samples)
        self.distance_to_target = deque(maxlen=max_samples)
        self.linear_velocity_cmd = deque(maxlen=max_samples)
        self.angular_velocity_cmd = deque(maxlen=max_samples)
        self.controller_state = deque(maxlen=max_samples)
        
        # PID data for both controllers
        self.linear_pid_p = deque(maxlen=max_samples)
        self.linear_pid_i = deque(maxlen=max_samples)
        self.linear_pid_d = deque(maxlen=max_samples)
        self.angular_pid_p = deque(maxlen=max_samples)
        self.angular_pid_i = deque(maxlen=max_samples)
        self.angular_pid_d = deque(maxlen=max_samples)
        
        # Advanced metrics
        self.control_effort = deque(maxlen=max_samples)
        self.trajectory_x = deque(maxlen=max_samples)
        self.trajectory_y = deque(maxlen=max_samples)
    
    def add_sample(self, timestamp: float, pose: RobotPose, target_x: float, target_y: float,
                   angular_error: float, cmd_vel: Twist, state: ControllerState,
                   linear_pid: EnhancedPIDController, angular_pid: EnhancedPIDController):
        """Add a comprehensive data sample"""
        
        self.timestamps.append(timestamp)
        self.position_x.append(pose.x)
        self.position_y.append(pose.y)
        
        distance = pose.distance_to(target_x, target_y)
        self.position_error.append(distance)
        self.angular_error.append(angular_error)
        self.distance_to_target.append(distance)
        
        self.linear_velocity_cmd.append(cmd_vel.linear.x)
        self.angular_velocity_cmd.append(cmd_vel.angular.z)
        self.controller_state.append(state.value)
        
        # PID components
        self.linear_pid_p.append(linear_pid.gains.p_term)
        self.linear_pid_i.append(linear_pid.gains.i_term)
        self.linear_pid_d.append(linear_pid.gains.d_term)
        self.angular_pid_p.append(angular_pid.gains.p_term)
        self.angular_pid_i.append(angular_pid.gains.i_term)
        self.angular_pid_d.append(angular_pid.gains.d_term)
        
        # Advanced metrics
        control_effort = abs(cmd_vel.linear.x) + abs(cmd_vel.angular.z)
        self.control_effort.append(control_effort)
        self.trajectory_x.append(pose.x)
        self.trajectory_y.append(pose.y)
    
    def get_arrays(self) -> Dict[str, np.ndarray]:
        """Get all data as numpy arrays for plotting"""
        return {
            'timestamps': np.array(self.timestamps),
            'position_x': np.array(self.position_x),
            'position_y': np.array(self.position_y),
            'position_error': np.array(self.position_error),
            'angular_error': np.array(self.angular_error),
            'distance_to_target': np.array(self.distance_to_target),
            'linear_velocity_cmd': np.array(self.linear_velocity_cmd),
            'angular_velocity_cmd': np.array(self.angular_velocity_cmd),
            'controller_state': np.array(self.controller_state),
            'linear_pid_p': np.array(self.linear_pid_p),
            'linear_pid_i': np.array(self.linear_pid_i),
            'linear_pid_d': np.array(self.linear_pid_d),
            'angular_pid_p': np.array(self.angular_pid_p),
            'angular_pid_i': np.array(self.angular_pid_i),
            'angular_pid_d': np.array(self.angular_pid_d),
            'control_effort': np.array(self.control_effort),
            'trajectory_x': np.array(self.trajectory_x),
            'trajectory_y': np.array(self.trajectory_y)
        }


class AdvancedPositionController(Node):
    """
    Advanced position controller for headless operation.
    """
    
    def __init__(self):
        super().__init__('advanced_position_controller')
        
        # Initialize data collection system
        self.data_collector = DataCollector()
        
        # Target position and orientation
# PICKUP_ST1_NAV     = [0.241, -0.227, 0.707]  # Arm pickup station (Nav2 target)

        self.declare_parameter('target_x', 0.24)
        self.declare_parameter('target_y', -0.23)
        self.declare_parameter('target_yaw', 1.57)
        self.target_x = self.get_parameter('target_x').value
        self.target_y = self.get_parameter('target_y').value
        self.target_yaw = self.get_parameter('target_yaw').value
        
        # Control thresholds
        self.declare_parameter('position_tolerance', 0.015)
        self.declare_parameter('angle_tolerance', 0.05)
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.angle_tolerance = self.get_parameter('angle_tolerance').value
        
        # Pulse control parameters
        self.declare_parameter('pulse_activation_distance', 0.05)
        self.declare_parameter('pulse_duration', 0.2) # 0.05
        self.declare_parameter('pulse_interval', 0.3) # 0.2
        self.declare_parameter('pulse_linear_speed', 0.02)
        self.declare_parameter('pulse_angular_speed', 0.2)
        
        self.pulse_activation_distance = self.get_parameter('pulse_activation_distance').value
        self.pulse_duration = self.get_parameter('pulse_duration').value
        self.pulse_interval = self.get_parameter('pulse_interval').value
        self.pulse_linear_speed = self.get_parameter('pulse_linear_speed').value
        self.pulse_angular_speed = self.get_parameter('pulse_angular_speed').value
        
        # Velocity limits
        self.declare_parameter('max_linear_vel', 0.2)
        self.declare_parameter('max_angular_vel', 0.5)
        self.max_linear_vel = self.get_parameter('max_linear_vel').value
        self.max_angular_vel = self.get_parameter('max_angular_vel').value
        
        # Enhanced PID controllers
        self.angular_pid = EnhancedPIDController(PIDGains(
            kp=2.0,
            ki=0.1,
            kd=0.5,
            integral_limit=0.5,
            output_limit=self.max_angular_vel
        ), "Angular")
        
        self.linear_pid = EnhancedPIDController(PIDGains(
            kp=1.0,
            ki=0.05,
            kd=0.2,
            integral_limit=0.3,
            output_limit=self.max_linear_vel
        ), "Linear")
        
        # State variables
        self.current_pose = RobotPose()
        self.state = ControllerState.IDLE
        self.odom_received = False
        
        # Performance tracking
        self.performance_metrics = PerformanceMetrics()
        self.mission_start_time = None
        
        # Pulse control state
        self.pulse_start_time = None
        self.pulse_active = False
        self.pulse_cycle_start_time = None
        
        # ROS2 setup
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.odom_sub = self.create_subscription(Odometry, '/odom_aruco', self.odom_callback, qos)
        self.odom_aruco_sub = self.create_subscription(Odometry, '/DP_08/odom_aruco', self.odom_callback, qos)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Control loop timer (50Hz)
        self.control_timer = self.create_timer(0.02, self.control_loop)
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        # Timeout protection
        self.start_time = self.get_clock().now()
        self.timeout_duration = 100000.0
        
        self.get_logger().info('🚀 Advanced Position Controller (Headless) initialized')
        self.get_logger().info(f'Default Target: ({self.target_x:.3f}, {self.target_y:.3f})')
    
    def odom_callback(self, msg: Odometry):
        """Enhanced odometry callback with velocity tracking"""
        if not self.odom_received:
            # Start control on first odom message if idle
            if self.state == ControllerState.IDLE:
                self.state = ControllerState.ROTATING_TO_TARGET
                self.mission_start_time = self.get_clock().now().nanoseconds / 1e9
                self.performance_metrics.start_time = self.mission_start_time
                self.get_logger().info('🎯 Starting navigation to target...')
        
        self.odom_received = True
        
        # Extract position
        self.current_pose.x = msg.pose.pose.position.x
        self.current_pose.y = msg.pose.pose.position.y
        
        # Extract orientation
        q = msg.pose.pose.orientation
        rotation = SciR.from_quat([q.x, q.y, q.z, q.w])
        euler = rotation.as_euler('xyz')
        self.current_pose.theta = euler[2]
        
        # Extract velocities
        self.current_pose.vx = msg.twist.twist.linear.x
        self.current_pose.vy = msg.twist.twist.linear.y
        self.current_pose.omega = msg.twist.twist.angular.z
        
    def wrap_angle(self, angle: float) -> float:
        """Wrap angle to [-pi, pi]"""
        return (angle + math.pi) % (2 * math.pi) - math.pi
    
    def get_shortest_angular_distance(self, from_angle: float, to_angle: float) -> float:
        """Calculate shortest angular distance from current to target angle
        Returns value in range [-pi, pi] where positive = CCW, negative = CW"""
        # Normalize both angles to [-pi, pi]
        from_norm = self.wrap_angle(from_angle)
        to_norm = self.wrap_angle(to_angle)
        
        # Calculate raw difference
        diff = to_norm - from_norm
        
        # Wrap to shortest path [-pi, pi]
        if diff > math.pi:
            diff -= 2 * math.pi
        elif diff < -math.pi:
            diff += 2 * math.pi
            
        return diff
    
    def update_pulse_state(self, current_time: float) -> bool:
        """Update pulse control state"""
        if self.pulse_cycle_start_time is None:
            self.pulse_cycle_start_time = current_time
            self.pulse_active = True
            return True
        
        cycle_elapsed = current_time - self.pulse_cycle_start_time
        
        if cycle_elapsed >= self.pulse_interval:
            self.pulse_cycle_start_time = current_time
            cycle_elapsed = 0.0
        
        self.pulse_active = cycle_elapsed < self.pulse_duration
        return self.pulse_active
    
    def control_loop(self):
        """Enhanced control loop with comprehensive data collection"""
        if not self.odom_received:
            return
        
        # Check timeout
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed > self.timeout_duration and self.state != ControllerState.REACHED:
            self.get_logger().error(f'⏰ Timeout! Failed to reach target in {self.timeout_duration}s')
            self.state = ControllerState.ERROR
            self.stop_robot()
            return
        
        cmd_vel = Twist()
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Calculate errors
        distance = self.current_pose.distance_to(self.target_x, self.target_y)
        target_angle = self.current_pose.angle_to(self.target_x, self.target_y)
        angle_error = self.wrap_angle(target_angle - self.current_pose.theta)
        
        # Update performance metrics
        self.performance_metrics.max_position_error = max(
            self.performance_metrics.max_position_error, distance
        )
        self.performance_metrics.max_angular_error = max(
            self.performance_metrics.max_angular_error, abs(angle_error)
        )
        
        # State machine logic (same as original, but with enhanced monitoring)
        if self.state == ControllerState.ROTATING_TO_TARGET:
            if abs(angle_error) > self.angle_tolerance*2:
                cmd_vel.angular.z = self.angular_pid.compute(angle_error, current_time)
            else:
                self.get_logger().info('✅ Rotation complete, moving to target')
                self.state = ControllerState.MOVING_TO_TARGET
                self.angular_pid.reset()
                self.linear_pid.reset()
        
        elif self.state == ControllerState.MOVING_TO_TARGET:
            if distance > self.position_tolerance:
                if distance <= self.pulse_activation_distance:
                    self.get_logger().info(f'🎯 Entering pulse control (distance: {distance:.4f}m)')
                    self.state = ControllerState.PULSE_CONTROL
                    self.pulse_cycle_start_time = None
                    self.linear_pid.reset()
                    self.angular_pid.reset()
                else:
                    cmd_vel.linear.x = self.linear_pid.compute(distance, current_time)
                    if distance < 0.1:
                        cmd_vel.linear.x *= 0.5
                    if abs(angle_error) > self.angle_tolerance * 2:
                        cmd_vel.angular.z = self.angular_pid.compute(angle_error, current_time) * 0.5
            else:
                self.get_logger().info('🎯 Position reached, final adjustment')
                self.state = ControllerState.FINAL_ADJUSTMENT
                self.linear_pid.reset()
                self.angular_pid.reset()
        
        elif self.state == ControllerState.PULSE_CONTROL:
            if distance > self.position_tolerance:
                pulse_active = self.update_pulse_state(current_time)
                if pulse_active:
                    dx = self.target_x - self.current_pose.x
                    dy = self.target_y - self.current_pose.y
                    distance_norm = math.sqrt(dx*dx + dy*dy)
                    
                    if distance_norm > 0:
                        target_vx = (dx / distance_norm) * self.pulse_linear_speed
                        target_vy = (dy / distance_norm) * self.pulse_linear_speed
                        
                        cos_theta = math.cos(self.current_pose.theta)
                        sin_theta = math.sin(self.current_pose.theta)
                        
                        cmd_vel.linear.x = target_vx * cos_theta + target_vy * sin_theta
                        
                        if abs(angle_error) > self.angle_tolerance:
                            cmd_vel.angular.z = np.sign(angle_error) * min(
                                self.pulse_angular_speed, abs(angle_error)
                            )
                else:
                    cmd_vel = Twist()
            else:
                self.get_logger().info('✅ Pulse control complete, final adjustment')
                self.state = ControllerState.FINAL_ADJUSTMENT
                self.pulse_cycle_start_time = None
                self.linear_pid.reset()
                self.angular_pid.reset()
        
        elif self.state == ControllerState.FINAL_ADJUSTMENT:
            # DECOUPLED CONTROL: Position first, then orientation
            if distance > self.position_tolerance:
                # POSITION ONLY - No rotation to prevent diagonal drift
                dx = self.target_x - self.current_pose.x
                dy = self.target_y - self.current_pose.y
                
                # Calculate forward/backward component in robot frame
                forward_dist = dx * math.cos(self.current_pose.theta) + dy * math.sin(self.current_pose.theta)
                
                # Move straight forward/backward only
                if abs(forward_dist) > 0.005:  # 5mm dead zone
                    cmd_vel.linear.x = np.sign(forward_dist) * min(0.03, abs(forward_dist) * 2)
                    cmd_vel.angular.z = 0.0  # ✅ NO rotation during position adjustment
                else:
                    cmd_vel = Twist()  # Stop completely
                    
            else:
                # ORIENTATION ONLY - Position reached, now rotate
                # Use shortest path calculation to prevent 180° flips
                final_yaw_error = self.get_shortest_angular_distance(
                    self.current_pose.theta, self.target_yaw
                )
                
                # Dead zone to prevent micro-oscillations
                if abs(final_yaw_error) > self.angle_tolerance:
                    # ADAPTIVE SPEED based on error magnitude
                    error_deg = abs(math.degrees(final_yaw_error))
                    
                    # Progressive speed reduction
                    if error_deg > 45:
                        angular_speed = 0.20  # Fast for large errors
                        pulse_duration_ratio = 1.0  # 70% on time
                    elif error_deg > 20:
                        angular_speed = 0.12  # Medium speed
                        pulse_duration_ratio = 1.0  # 50% on time
                    elif error_deg > 10:
                        angular_speed = 0.08  # Slow
                        pulse_duration_ratio = 1.0  # 30% on time
                    else:
                        angular_speed = 0.04  # Very slow for final approach
                        pulse_duration_ratio = 0.7  # 20% on time
                    
                    # Apply pulse control for smooth convergence
                    pulse_active = self.update_pulse_state(current_time)
                    
                    if pulse_active:
                        # Check if we should pulse based on duration ratio
                        cycle_elapsed = current_time - self.pulse_cycle_start_time
                        if (cycle_elapsed % self.pulse_interval) < (self.pulse_interval * pulse_duration_ratio):
                            # Active phase - rotate
                            rotation_direction = np.sign(final_yaw_error)
                            
                            # Velocity damping to prevent overshooting
                            if abs(self.current_pose.omega) > 0.15:  # Already rotating fast
                                damping = 0.5  # Reduce command by half
                            else:
                                damping = 1.0
                            
                            cmd_vel.angular.z = rotation_direction * angular_speed * damping
                            cmd_vel.linear.x = 0.0  # ✅ NO linear motion during rotation
                            
                            # Log occasionally
                            if cycle_elapsed < 0.02:
                                self.get_logger().info(
                                    f'🔄 Rotation: {math.degrees(final_yaw_error):.1f}° | '
                                    f'Speed: {angular_speed:.2f} rad/s | '
                                    f'Pulse: {int(pulse_duration_ratio*100)}%'
                                )
                        else:
                            # Rest phase - complete stop
                            cmd_vel = Twist()
                    else:
                        cmd_vel = Twist()
                        
                else:
                    # Target reached!
                    self.get_logger().info('🎉 TARGET REACHED SUCCESSFULLY!')
                    self.get_logger().info(f'📍 Final position: ({self.current_pose.x:.4f}, {self.current_pose.y:.4f})')
                    self.get_logger().info(f'📐 Final orientation: {math.degrees(self.current_pose.theta):.1f}°')
                    self.get_logger().info(f'📏 Position error: {distance:.4f}m')
                    self.get_logger().info(f'📐 Yaw error: {math.degrees(final_yaw_error):.2f}°')
                    
                    total_time = current_time - self.performance_metrics.start_time
                    self.performance_metrics.total_time = total_time
                    self.performance_metrics.settling_time = total_time
                    
                    self.state = ControllerState.REACHED
                    self.pulse_cycle_start_time = None
                    self.stop_robot()
                    return
        
        elif self.state == ControllerState.REACHED:
            if distance > self.position_tolerance * 2:
                self.get_logger().warn('⚠️ Position drift detected, correcting...')
                if distance <= self.pulse_activation_distance:
                    self.state = ControllerState.PULSE_CONTROL
                    self.pulse_cycle_start_time = None
                else:
                    self.state = ControllerState.ROTATING_TO_TARGET
                self.angular_pid.reset()
                self.linear_pid.reset()
            else:
                cmd_vel = Twist()
        
        elif self.state == ControllerState.PAUSED:
            cmd_vel = Twist()
        
        elif self.state == ControllerState.ERROR:
            self.stop_robot()
            return
        
        # Update performance tracking
        control_effort = abs(cmd_vel.linear.x) + abs(cmd_vel.angular.z)
        self.performance_metrics.total_control_effort += control_effort
        self.performance_metrics.peak_linear_velocity = max(
            self.performance_metrics.peak_linear_velocity, abs(cmd_vel.linear.x)
        )
        self.performance_metrics.peak_angular_velocity = max(
            self.performance_metrics.peak_angular_velocity, abs(cmd_vel.angular.z)
        )
        
        # Collect data for GUI
        self.data_collector.add_sample(
            current_time, self.current_pose, self.target_x, self.target_y,
            angle_error, cmd_vel, self.state, self.linear_pid, self.angular_pid
        )
        
        # Publish command
        self.cmd_vel_pub.publish(cmd_vel)
    
    def stop_robot(self):
        """Send stop command"""
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
    
    def print_status(self):
        """Enhanced status printing with performance metrics"""
        if not self.odom_received:
            self.get_logger().warn('⏳ Waiting for odometry data...')
            return
        
        distance = self.current_pose.distance_to(self.target_x, self.target_y)
        target_angle = self.current_pose.angle_to(self.target_x, self.target_y)
        
        # Calculate angle error based on current control state (same as control loop logic)
        if self.state == ControllerState.FINAL_ADJUSTMENT and distance <= self.position_tolerance * 0.5:
            # During final orientation adjustment, show error to final target orientation
            angle_error = self.wrap_angle(self.target_yaw - self.current_pose.theta)
        else:
            # During navigation, show error to target direction
            angle_error = self.wrap_angle(target_angle - self.current_pose.theta)
        
        angle_error_deg = math.degrees(angle_error)
        
        # Enhanced status with performance indicators
        status_msg = (
            f'🤖 State: {self.state.name} | '
            f'📍 Pos: ({self.current_pose.x:.3f}, {self.current_pose.y:.3f}) | '
            f'📏 Dist: {distance:.3f}m | '
            f'📐 Angle: {angle_error_deg:.1f}° | '
            f'⚡ Max Error: {self.performance_metrics.max_position_error:.3f}m'
        )
        
        self.get_logger().info(status_msg)
    
    def reset_control(self):
        """Reset control system"""
        self.state = ControllerState.IDLE
        self.angular_pid.reset()
        self.linear_pid.reset()
        self.performance_metrics = PerformanceMetrics()
        self.data_collector = DataCollector()
        self.mission_start_time = None
        self.pulse_cycle_start_time = None
        self.stop_robot()
    
    def emergency_stop(self):
        """Emergency stop procedure"""
        self.state = ControllerState.ERROR
        self.stop_robot()
        self.get_logger().error('🚨 EMERGENCY STOP ACTIVATED!')
    
    def set_target(self, x: float, y: float, target_yaw: float = 0.0):
        """Set new target position and orientation"""
        self.target_x = x
        self.target_y = y
        self.target_yaw = target_yaw
        self.get_logger().info(f'🎯 New target set: ({x:.3f}, {y:.3f}), yaw: {math.degrees(target_yaw):.1f}°')


def main(args=None):
    """Headless main function for standalone execution"""
    rclpy.init(args=args)
    
    controller = AdvancedPositionController()
    
    # This main function is for testing. 
    # The actual target will be set by the FSM when used as a class.
    
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(controller)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        controller.get_logger().info("Parking script shutting down...")
        controller.stop_robot()
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()