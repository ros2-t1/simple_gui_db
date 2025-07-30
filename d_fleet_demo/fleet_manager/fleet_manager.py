#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
from typing import Dict, List
import threading

# Import shared message definitions
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(__file__)))
from shared.fleet_msgs import TaskRequest, TaskResponse, RobotState, TaskStatus, RobotStatus, TaskType

# Add web directory to path for database access
sys.path.append(os.path.join(os.path.dirname(os.path.dirname(__file__)), 'web'))
from task_db import get_next_pending_task, update_task_status, complete_task, get_robot_current_task, get_timeout_tasks, fail_task

class FleetManager(Node):
    def __init__(self):
        super().__init__('fleet_manager')
        
        # Robot management (tasks now managed in database)
        self.robots: Dict[str, RobotState] = {}
        
        # Robot ID mapping (for database integration)
        # Using HANA_PINKY (hana_bot_id = 10) as our main robot
        self.robot_name_to_id = {"robot_1": 10}  # Maps to HANA_PINKY
        
        # Initialize with one robot (same as original)
        self.robots["robot_1"] = RobotState(
            robot_id="robot_1",
            status=RobotStatus.IDLE
        )
        
        # Fleet-level communication
        self.task_request_sub = self.create_subscription(
            String, '/fleet/task_request', self.handle_task_request, 10)
        self.task_response_pub = self.create_publisher(
            String, '/fleet/task_response', 10)
            
        # Robot communication (per robot)
        self.robot_cmd_pubs = {}
        self.robot_status_subs = {}
        
        # Setup robot communication
        self.setup_robot_communication()
        
        # Initialize robot status
        self.robots["robot_1"].last_update = time.time()
        
        # Status monitoring timer
        self.create_timer(1.0, self.monitor_fleet)
        
        # Task assignment timer (check for pending tasks every 2 seconds)
        self.create_timer(2.0, self.check_and_assign_tasks)
        
        # Task timeout checker (check for timeout tasks every 30 seconds)
        self.create_timer(30.0, self.check_task_timeouts)
        
        # Robot status broadcast (for web frontend)
        self.robot_status_pub = self.create_publisher(String, '/fleet/robot_status', 10)
        
        self.get_logger().info("Fleet Manager initialized")
    
    def setup_robot_communication(self):
        """Setup communication channels for each robot"""
        for robot_id in self.robots.keys():
            # Command publisher to robot (matches robot FSM topic)
            self.robot_cmd_pubs[robot_id] = self.create_publisher(
                String, f'/{robot_id}/user_cmd', 10)
            
            # Status subscriber from robot (matches robot FSM topic)
            self.robot_status_subs[robot_id] = self.create_subscription(
                String, f'/{robot_id}/status', 
                lambda msg, rid=robot_id: self.handle_robot_status(msg, rid), 10)
            
            self.get_logger().info(f"Setup communication for {robot_id}: /{robot_id}/user_cmd, /{robot_id}/status")
    
    def handle_task_request(self, msg: String):
        """Handle incoming task requests from web server"""
        try:
            data = json.loads(msg.data)
            db_task_id = data.get('db_task_id')
            
            self.get_logger().info(f"Received task request: {data['task_id']} (DB ID: {db_task_id})")
            
            # Task is already stored in database, just send acknowledgment
            response = TaskResponse(
                task_id=data['task_id'],
                status=TaskStatus.PENDING,
                message="Task queued successfully"
            )
            self.send_task_response(response)
            
            # Try to assign pending tasks immediately
            self.check_and_assign_tasks()
            
        except Exception as e:
            self.get_logger().error(f"Error handling task request: {e}")
    
    def check_and_assign_tasks(self):
        """Check for pending tasks and assign to available robots (FIFO)"""
        try:
            # Find idle robots
            idle_robots = [robot_id for robot_id, robot_state in self.robots.items() 
                          if robot_state.status == RobotStatus.IDLE]
            
            if not idle_robots:
                return  # No available robots
            
            # Get next pending task from database (FIFO order)
            next_task = get_next_pending_task()
            if not next_task:
                return  # No pending tasks
            
            # Assign to first available robot
            robot_id = idle_robots[0]
            robot_db_id = self.robot_name_to_id[robot_id]
            
            # Update database
            update_task_status(next_task['task_id'], 'assigned', robot_db_id)
            
            # Update robot state
            self.robots[robot_id].status = RobotStatus.BUSY
            self.robots[robot_id].current_task_id = str(next_task['task_id'])
            
            # Send command to robot with resident_id for target location
            if next_task['task_type'] == '배달':
                cmd_data = {
                    "command": "order",
                    "resident_id": next_task['requester_resident_id']
                }
                cmd_msg = String(data=json.dumps(cmd_data))
                self.robot_cmd_pubs[robot_id].publish(cmd_msg)
                
                self.get_logger().info(f"Assigned task {next_task['task_id']} to {robot_id} for resident {next_task['requester_resident_id']}")
            
        except Exception as e:
            self.get_logger().error(f"Error in task assignment: {e}")
    
    
    def handle_robot_status(self, msg: String, robot_id: str):
        """Handle status updates from robots"""
        status_text = msg.data
        self.get_logger().info(f"Robot {robot_id} status: {status_text}")
        
        robot_state = self.robots[robot_id]
        previous_status = robot_state.status
        
        # Update robot status based on message
        if status_text == "idle":
            robot_state.status = RobotStatus.IDLE
            # Note: Task completion is now handled in handle_confirm_request()
            # idle means robot returned to dock after completing all tasks
            
        elif status_text in ["moving_to_arm", "picking", "moving_to_user", "waiting_confirm", "returning_to_dock"]:
            robot_state.status = RobotStatus.BUSY
            
            # Update task status in database
            if robot_state.current_task_id:
                try:
                    update_task_status(int(robot_state.current_task_id), 'in_progress')
                except Exception as e:
                    self.get_logger().error(f"Error updating task status: {e}")
        
        elif status_text == "navigation_failed":
            self.get_logger().error(f"Robot {robot_id} navigation failed!")
            self._handle_robot_failure(robot_id, robot_state)
        
        robot_state.last_update = time.time()
        
        # Broadcast robot status to web frontend
        self.broadcast_robot_status(robot_id, status_text, robot_state.current_task_id)
    
    def _handle_robot_failure(self, robot_id: str, robot_state):
        """Handle robot navigation failure"""
        try:
            # Mark current task as failed in database
            if robot_state.current_task_id:
                task_id = int(robot_state.current_task_id)
                self.get_logger().warn(f"Marking task {task_id} as failed due to navigation failure")
                fail_task(task_id, "Navigation failure")
                robot_state.current_task_id = None
            
            # Reset robot to idle
            robot_state.status = RobotStatus.IDLE
            
            # Try to assign next task if available
            self.get_logger().info("Attempting to assign next task after failure recovery")
            self.check_and_assign_tasks()
            
        except Exception as e:
            self.get_logger().error(f"Error handling robot failure: {e}")
    
    def handle_confirm_request(self, msg: String):
        """Handle confirmation requests from web server"""
        try:
            data = json.loads(msg.data)
            robot_id = data.get('robot_id', 'robot_1')  # Default to robot_1
            
            robot_state = self.robots[robot_id]
            
            # Complete current task
            if robot_state.current_task_id:
                try:
                    complete_task(int(robot_state.current_task_id))
                    self.get_logger().info(f"Completed task {robot_state.current_task_id} on confirm")
                    robot_state.current_task_id = None
                except Exception as e:
                    self.get_logger().error(f"Error completing task on confirm: {e}")
            
            # Check if there's a next pending task
            next_task = get_next_pending_task()
            
            if next_task:
                # Assign next task immediately - robot goes directly to arm
                robot_db_id = self.robot_name_to_id[robot_id]
                update_task_status(next_task['task_id'], 'assigned', robot_db_id)
                
                robot_state.status = RobotStatus.BUSY
                robot_state.current_task_id = str(next_task['task_id'])
                
                # Send order command with resident_id (robot will go to arm directly)
                cmd_data = {
                    "command": "order",
                    "resident_id": next_task['requester_resident_id']
                }
                cmd_msg = String(data=json.dumps(cmd_data))
                self.robot_cmd_pubs[robot_id].publish(cmd_msg)
                
                self.get_logger().info(f"Assigned next task {next_task['task_id']} to {robot_id} for resident {next_task['requester_resident_id']} immediately after confirm")
            else:
                # No more tasks - send confirm to return to dock
                cmd_msg = String(data="confirm")
                self.robot_cmd_pubs[robot_id].publish(cmd_msg)
                self.get_logger().info(f"No more tasks, sent confirm to {robot_id} to return to dock")
                
        except Exception as e:
            self.get_logger().error(f"Error handling confirm request: {e}")
    
    def broadcast_robot_status(self, robot_id: str, status: str, task_id: str = None):
        """Broadcast robot status to web frontend"""
        try:
            status_data = {
                'robot_id': robot_id,
                'status': status,
                'current_task_id': task_id,
                'timestamp': time.time()
            }
            msg = String(data=json.dumps(status_data))
            self.robot_status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Error broadcasting robot status: {e}")
    
    def send_task_response(self, response: TaskResponse):
        """Send task response back to web server"""
        response_data = {
            'task_id': response.task_id,
            'status': response.status.value,
            'robot_id': response.robot_id,
            'message': response.message
        }
        msg = String(data=json.dumps(response_data))
        self.task_response_pub.publish(msg)
    
    def monitor_fleet(self):
        """Periodic fleet monitoring"""
        # Check for stale robot states (5 second timeout since heartbeat is every 2 seconds)
        current_time = time.time()
        for robot_id, robot_state in self.robots.items():
            if current_time - robot_state.last_update > 5.0:  # 5 seconds timeout
                if robot_state.status != RobotStatus.ERROR:
                    self.get_logger().warn(f"Robot {robot_id} appears offline")
                    robot_state.status = RobotStatus.ERROR
            elif robot_state.status == RobotStatus.ERROR and current_time - robot_state.last_update < 3.0:
                # Robot came back online
                self.get_logger().info(f"Robot {robot_id} is back online")
                robot_state.status = RobotStatus.IDLE
    
    def check_task_timeouts(self):
        """Check for tasks that have exceeded timeout and mark them as failed"""
        try:
            timeout_tasks = get_timeout_tasks(timeout_minutes=5)
            
            for task in timeout_tasks:
                task_id = task['task_id']
                minutes_elapsed = task['minutes_elapsed']
                robot_id = task['assigned_bot_id']
                
                self.get_logger().warn(f"Task {task_id} has timed out after {minutes_elapsed:.1f} minutes")
                
                # # Mark task as failed in database
                # fail_task(task_id, f"Timeout after {minutes_elapsed:.1f} minutes")
                
                # # Reset robot to idle if it was assigned to this task
                # for rid, robot_state in self.robots.items():
                #     if robot_state.current_task_id == str(task_id):
                #         robot_state.current_task_id = None
                #         robot_state.status = RobotStatus.IDLE
                #         self.get_logger().info(f"Reset {rid} to idle due to task timeout")
                        
                #         # Send stop command to robot
                #         try:
                #             cmd_msg = String(data="stop")
                #             if rid in self.robot_cmd_pubs:
                #                 self.robot_cmd_pubs[rid].publish(cmd_msg)
                #                 self.get_logger().info(f"Sent stop command to {rid} due to timeout")
                #         except Exception as e:
                #             self.get_logger().error(f"Failed to send stop command to {rid}: {e}")
                        
                #         # Broadcast updated status
                #         self.broadcast_robot_status(rid, "idle")
                #         break
                
                # self.get_logger().info(f"Task {task_id} marked as failed due to timeout")
            
            if timeout_tasks:
                # Try to assign new tasks after handling timeouts
                self.check_and_assign_tasks()
                
        except Exception as e:
            self.get_logger().error(f"Error checking task timeouts: {e}")
        
        # Debug: Log fleet status periodically 
        current_time = time.time()
        if int(current_time) % 5 == 0:  # Every 5 seconds
            for robot_id, robot_state in self.robots.items():
                self.get_logger().info(f"Robot {robot_id}: {robot_state.status.value}, last_update: {current_time - robot_state.last_update:.1f}s ago")

def main():
    rclpy.init()
    fleet_manager = FleetManager()
    
    # Subscribe to confirm requests
    fleet_manager.confirm_sub = fleet_manager.create_subscription(
        String, '/fleet/confirm_request', fleet_manager.handle_confirm_request, 10)
    
    try:
        rclpy.spin(fleet_manager)
    except KeyboardInterrupt:
        pass
    finally:
        fleet_manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()