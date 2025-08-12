from typing import List
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from std_msgs.msg import String, Int32
import json
import sys
import os
import time
import threading

# Database access removed - coordinates now provided by Fleet Manager

from .nav2_waypoint_class import WaypointNavigator
from .parking import AdvancedPositionController, ControllerState

try:
    import robot_config as cfg
except ImportError:
    # Fallback to main config for backward compatibility
    import config as cfg

from enum import IntEnum, auto

class Step(IntEnum):
    IDLE          = auto()
    GO_TO_ARM     = auto()
    PARKING       = auto()
    PICK          = auto()  # Start picking process
    WAIT_ARM      = auto()  # Wait for robot arm to complete
    GO_TO_USER    = auto()
    WAIT_CONFIRM  = auto()
    GO_DOCK       = auto()
    DOCK_PARKING  = auto()

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
        
        # Parking controller attributes
        self.parking_controller = None
        self.parking_executor = None
        self.parking_thread = None

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

    def _cleanup_parking(self):
        """Safely shut down and clean up the parking controller and its thread."""
        if self.parking_executor:
            self.parking_executor.shutdown()
        
        if self.parking_controller:
            # Wait briefly for the node to stop publishing
            time.sleep(0.1)
            self.parking_controller.destroy_node()
        
        self.parking_controller = None
        self.parking_executor = None
        self.parking_thread = None
        self.get_logger().info("Precision parking resources cleaned up.")

    # ---------- 상태 전이 ----------ff
    def set_step(self, nxt: int):
        self.step = nxt
        self.waypoint_nav.pinky_nav2_state = "None"

        # Clean up parking resources if we are moving to a new non-parking state
        if self.step not in [Step.PARKING, Step.DOCK_PARKING] and self.parking_controller is not None:
            self._cleanup_parking()

        match self.step:
            case Step.IDLE:
                self.pub_status("idle")
            case Step.GO_TO_ARM:
                self.pub_status("moving_to_arm")
                self.waypoint_nav.send_goal(cfg.PICKUP_ST1)
                # Send item_id to robot arm immediately when starting to move
                if self.current_item_id:
                    item_msg = Int32()
                    item_msg.data = int(self.current_item_id)
                    self.arm_cmd_pub.publish(item_msg)
                    self.get_logger().info(f"Sent item_id {self.current_item_id} to robot arm (early start)")
                    # Reset arm completion flag for later checking
                    self.arm_complete = False
                    self.arm_timeout_start = time.time()
            case Step.PARKING:
                self.pub_status("parking")
                self.get_logger().info("State set to PARKING. Controller will be created in the loop.")
            case Step.PICK:
                self.pub_status("picking")
                # Check if we already sent command in GO_TO_ARM
                if self.current_item_id and self.arm_timeout_start:
                    # Already sent command, just wait for completion
                    self.set_step(Step.WAIT_ARM)
                elif self.current_item_id:
                    # Fallback: send command now if not sent earlier
                    item_msg = Int32()
                    item_msg.data = int(self.current_item_id)
                    self.arm_cmd_pub.publish(item_msg)
                    self.get_logger().info(f"Sent item_id {self.current_item_id} to robot arm (at pickup)")
                    self.arm_complete = False
                    self.arm_timeout_start = time.time()
                    self.set_step(Step.WAIT_ARM)
                else:
                    # Fallback to old behavior if no item_id
                    self.get_logger().warn("No item_id available, using legacy arm control")
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
            case Step.DOCK_PARKING:
                self.pub_status("parking_at_dock")

    def loop(self):
        if self.step == Step.GO_TO_ARM  and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.PARKING)

        elif self.step == Step.PARKING:
            if self.parking_controller is None:
                self.get_logger().info("Creating and running parking controller.")
                self.parking_controller = AdvancedPositionController()
                
                pickup_coords = cfg.PICKUP_ST1
                target_x = pickup_coords[0]
                target_y = pickup_coords[1]
                self.parking_controller.set_target(target_x, target_y)

                self.parking_executor = SingleThreadedExecutor()
                self.parking_executor.add_node(self.parking_controller)
                
                self.parking_thread = threading.Thread(target=self.parking_executor.spin)
                self.parking_thread.daemon = True
                self.parking_thread.start()
                self.get_logger().info("Parking controller thread started.")
            
            else: # Monitor the running parking controller
                if self.parking_controller.state == ControllerState.REACHED:
                    self.get_logger().info("Precision parking successful.")
                    self._cleanup_parking()
                    self.set_step(Step.PICK)
                elif self.parking_controller.state == ControllerState.ERROR:
                    self.get_logger().error("Precision parking failed.")
                    self._cleanup_parking()
                    self.set_step(Step.PICK) # Proceed to PICK even if parking fails

        elif self.step == Step.WAIT_ARM:
            # Check if robot arm completed or timeout
            if self.arm_complete:
                self.get_logger().info("Robot arm task complete, proceeding to user")
                self.arm_complete = False
                self.arm_timeout_start = None
                self.set_step(Step.GO_TO_USER)
            elif self.arm_timeout_start and (time.time() - self.arm_timeout_start > 1000):
                # Timeout after 20 seconds
                self.get_logger().warn("Robot arm timeout, proceeding anyway")
                self.arm_timeout_start = None
                self.set_step(Step.GO_TO_USER)
        
        elif self.step == Step.GO_TO_USER and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.WAIT_CONFIRM)
        
        elif self.step == Step.GO_DOCK and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(Step.DOCK_PARKING)

        elif self.step == Step.DOCK_PARKING:
            if self.parking_controller is None:
                self.get_logger().info("Creating and running parking controller for docking.")
                self.parking_controller = AdvancedPositionController()
                
                # User-specified coordinates
                target_x = -0.3
                target_y = 0.42
                self.parking_controller.set_target(target_x, target_y)

                self.parking_executor = SingleThreadedExecutor()
                self.parking_executor.add_node(self.parking_controller)
                
                self.parking_thread = threading.Thread(target=self.parking_executor.spin)
                self.parking_thread.daemon = True
                self.parking_thread.start()
                self.get_logger().info("Dock parking controller thread started.")
            
            else: # Monitor
                if self.parking_controller.state == ControllerState.REACHED:
                    self.get_logger().info("Dock precision parking successful.")
                    self._cleanup_parking()
                    self.set_step(Step.IDLE) # Final state is IDLE
                elif self.parking_controller.state == ControllerState.ERROR:
                    self.get_logger().error("Dock precision parking failed.")
                    self._cleanup_parking()
                    self.set_step(Step.IDLE) # Go to IDLE even if parking fails

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
            Step.PARKING: "moving_to_arm",
            Step.PICK: "picking",
            Step.WAIT_ARM: "picking",  # Keep showing picking status while waiting for arm
            Step.GO_TO_USER: "moving_to_user",
            Step.WAIT_CONFIRM: "waiting_confirm",
            Step.GO_DOCK: "returning_to_dock",
            Step.DOCK_PARKING: "returning_to_dock"
        }
        
        current_status = status_map.get(self.step, "idle")
        self.stat_pub.publish(String(data=current_status))
        self.get_logger().debug(f"Heartbeat: {current_status}")
