from typing import List
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Int32
import json
import sys
import os
import time

# Database access removed - coordinates now provided by Fleet Manager

from .arm import arm_pick, arm_place_on_robot
from .nav2_waypoint_class import WaypointNavigator
try:
    import robot_config as cfg
except ImportError:
    # Fallback to main config for backward compatibility
    import config as cfg

from enum import IntEnum, auto

class Step(IntEnum):
    IDLE          = auto()
    GO_TO_ARM     = auto()
    PICK          = auto()  # Start picking process
    WAIT_ARM      = auto()  # Wait for robot arm to complete
    GO_TO_USER    = auto()
    WAIT_CONFIRM  = auto()
    GO_DOCK       = auto()

# ───────── ROS FSM 정의 ─────────
class DeliveryFSM(Node):
    def __init__(self, robot_name: str, wp_nav: WaypointNavigator):
        super().__init__(f"{robot_name}_delivery_fsm", namespace=robot_name)
        self.step = Step.IDLE
        self.waypoint_nav = wp_nav
        self.create_subscription(String, cfg.ROS_CMD_TOPIC, self.on_cmd, 10)
        self.cmd_pub   = self.create_publisher(String, cfg.ROS_CMD_TOPIC, 10)
        self.stat_pub  = self.create_publisher(String, cfg.ROS_STAT_TOPIC, 10)
        self.create_timer(0.5, self.loop)
        
        # Robot arm communication
        self.arm_cmd_pub = self.create_publisher(Int32, '/robot_arm/user_cmd', 10)
        self.create_subscription(String, '/robot_arm/status', self.on_arm_status, 10)
        self.arm_complete = False
        self.arm_timeout_start = None
        self.current_item_id = None
        
        # Default service station coordinates (will be updated per delivery)
        self.service_station_coords = cfg.SERVICE_ST1
        self.get_logger().info(f"Initialized with default service station coordinates: {self.service_station_coords}")
        
        # Send initial status after a delay
        self.initial_status_timer = self.create_timer(1.0, self.send_initial_status)
        
        # Periodic status heartbeat (every 2 seconds)
        self.create_timer(2.0, self.send_heartbeat)
    
    def _parse_coordinates(self, coords):
        """Parse PostgreSQL array format {x,y,z} to Python list [x,y,z]"""
        if isinstance(coords, list):
            return coords
        if isinstance(coords, str):
            # Remove braces and split by comma
            clean_coords = coords.strip('{}')
            return [float(x.strip()) for x in clean_coords.split(',')]
        return coords

    # ---------- 토픽 콜백 / 퍼블리시 ----------
    def on_cmd(self, msg: String):
        try:
            # Try to parse as JSON first (new format with resident_id and coordinates)
            cmd_data = json.loads(msg.data)
            command = cmd_data.get("command")
            resident_id = cmd_data.get("resident_id")
            target_coordinates = cmd_data.get("target_coordinates")
            item_id = cmd_data.get("item_id")  # Get item_id for robot arm
            
            if command == "order":
                if target_coordinates:
                    # Use coordinates provided by Fleet Manager
                    self.service_station_coords = target_coordinates
                    self.get_logger().info(f"Updated target coordinates for resident {resident_id}: {target_coordinates}")
                elif resident_id:
                    # Fallback: try to get coordinates from database (for backward compatibility)
                    new_coords = self._get_service_station_coords(resident_id)
                    if new_coords:
                        self.service_station_coords = new_coords
                        self.get_logger().info(f"Fallback: Updated target for resident {resident_id}: {new_coords}")
                
                # Store item_id for robot arm communication
                self.current_item_id = item_id
                if item_id:
                    self.get_logger().info(f"Received item_id: {item_id} for robot arm")
                
                # Determine task type (delivery vs call)
                task_type = cmd_data.get("task_type", "배달")  # Default to delivery
                
                if self.step == Step.IDLE:
                    if task_type == "호출":
                        # Call task: go directly to user (skip arm/pick)
                        self.set_step(Step.GO_TO_USER)
                    else:
                        # Delivery task: go to arm first
                        self.set_step(Step.GO_TO_ARM)
                elif self.step == Step.WAIT_CONFIRM:
                    # New task assigned immediately after confirm
                    if task_type == "호출":
                        self.set_step(Step.GO_TO_USER)
                    else:
                        self.set_step(Step.GO_TO_ARM)
                elif self.step == Step.GO_DOCK:
                    # Interrupt dock return and start new task immediately
                    self.get_logger().info(f"Interrupting dock return for new {task_type} task")
                    if task_type == "호출":
                        self.set_step(Step.GO_TO_USER)
                    else:
                        self.set_step(Step.GO_TO_ARM)
                    
        except json.JSONDecodeError:
            # Fallback to old string format
            if msg.data == "order":
                if self.step == Step.IDLE:
                    self.set_step(Step.GO_TO_ARM)
                elif self.step == Step.WAIT_CONFIRM:
                    # New task assigned immediately after confirm - go directly to arm
                    self.set_step(Step.GO_TO_ARM)
                elif self.step == Step.GO_DOCK:
                    # Interrupt dock return and start new task immediately
                    self.get_logger().info("Interrupting dock return for new task (fallback)")
                    self.set_step(Step.GO_TO_ARM)
        
        # Handle confirm command (always string format)
        if msg.data == "confirm" and self.step == Step.WAIT_CONFIRM:
            self.set_step(Step.GO_DOCK)
    
    def _get_service_station_coords(self, resident_id):
        """Fallback method - coordinates should be provided by Fleet Manager"""
        self.get_logger().warn(f"Fallback: No coordinates provided for resident {resident_id}. Using default.")
        return cfg.SERVICE_ST1  # Return default coordinates
    
    def _handle_navigation_failure(self):
        """Handle navigation failure - reset to IDLE and report failure"""
        self.get_logger().error(f"Navigation failed in step {self.step.name}. Resetting to IDLE.")
        
        # Reset navigation state
        self.waypoint_nav.pinky_nav2_state = "None"
        
        # Report failure status to fleet manager
        self.pub_status("navigation_failed")
        
        # Reset to IDLE state
        self.set_step(Step.IDLE)
    
    def on_arm_status(self, msg: String):
        """Handle status updates from robot arm"""
        if msg.data == "complete":
            self.arm_complete = True
            self.get_logger().info("Robot arm reported task complete")

    def pub_status(self, text: str):
        self.stat_pub.publish(String(data=text))
        self.get_logger().info(f"[STATUS] {text}")

    # ---------- 상태 전이 ----------
    def set_step(self, nxt: int):
        self.step = nxt
        self.waypoint_nav.pinky_nav2_state = "None"

        match self.step:
            case Step.IDLE:
                self.pub_status("idle")
            case Step.GO_TO_ARM:
                self.pub_status("moving_to_arm")
                self.waypoint_nav.send_goal(cfg.PICKUP_ST1)
            case Step.PICK:
                self.pub_status("picking")
                # Send item_id to robot arm if available
                if self.current_item_id:
                    item_msg = Int32()
                    item_msg.data = int(self.current_item_id)
                    self.arm_cmd_pub.publish(item_msg)
                    self.get_logger().info(f"Sent item_id {self.current_item_id} to robot arm")
                    # Reset arm completion flag and start timeout
                    self.arm_complete = False
                    self.arm_timeout_start = time.time()
                    self.set_step(Step.WAIT_ARM)
                else:
                    # Fallback to old behavior if no item_id
                    self.get_logger().warn("No item_id available, using legacy arm control")
                    arm_pick("vitamin")
                    arm_place_on_robot()
                    self.set_step(Step.GO_TO_USER)
            case Step.WAIT_ARM:
                self.pub_status("picking")  # Keep same status while waiting
            case Step.GO_TO_USER:
                self.pub_status("moving_to_user")
                self.waypoint_nav.send_goal(self.service_station_coords)
            case Step.WAIT_CONFIRM:
                self.pub_status("waiting_confirm")
            case Step.GO_DOCK:
                self.pub_status("returning_to_dock")
                self.waypoint_nav.send_goal(cfg.CHARGING_ST)

    def loop(self):
        if self.step == Step.GO_TO_ARM  and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.PICK)
        elif self.step == Step.WAIT_ARM:
            # Check if robot arm completed or timeout
            if self.arm_complete:
                self.get_logger().info("Robot arm task complete, proceeding to user")
                self.arm_complete = False
                self.arm_timeout_start = None
                self.set_step(Step.GO_TO_USER)
            elif self.arm_timeout_start and (time.time() - self.arm_timeout_start > 30):
                # Timeout after 30 seconds
                self.get_logger().warn("Robot arm timeout, proceeding anyway")
                self.arm_timeout_start = None
                self.set_step(Step.GO_TO_USER)
        elif self.step == Step.GO_TO_USER and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.WAIT_CONFIRM)
        elif self.step == Step.GO_DOCK   and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.IDLE)
        
        # Handle navigation failures
        elif self.waypoint_nav.pinky_nav2_state == "Failed":
            self._handle_navigation_failure()
    
    def send_initial_status(self):
        """Send initial status to fleet manager"""
        self.pub_status("idle")
        # Cancel the timer after first call
        self.initial_status_timer.cancel()
    
    def send_heartbeat(self):
        """Send periodic status update to keep connection alive"""
        # Send current status based on step
        status_map = {
            Step.IDLE: "idle",
            Step.GO_TO_ARM: "moving_to_arm", 
            Step.PICK: "picking",
            Step.WAIT_ARM: "picking",  # Keep showing picking status while waiting for arm
            Step.GO_TO_USER: "moving_to_user",
            Step.WAIT_CONFIRM: "waiting_confirm",
            Step.GO_DOCK: "returning_to_dock"
        }
        
        current_status = status_map.get(self.step, "idle")
        self.stat_pub.publish(String(data=current_status))
        self.get_logger().debug(f"Heartbeat: {current_status}")
