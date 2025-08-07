#!/usr/bin/env python3
"""
FSM 상태 모니터링 도구
로봇의 FSM 상태를 실시간으로 확인
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime

class FSMMonitor(Node):
    def __init__(self):
        super().__init__('fsm_monitor')
        
        # Subscribe to all robot status topics
        self.robot_status_subs = {}
        self.setup_robot_monitoring()
        
        # Subscribe to fleet status
        self.fleet_status_sub = self.create_subscription(
            String, '/fleet/robot_status', self.handle_fleet_status, 10)
        
        print("🔍 FSM Monitor Started")
        print("=" * 50)
    
    def setup_robot_monitoring(self):
        """Setup monitoring for individual robots"""
        robot_ids = ["robot_1", "robot_2", "robot_3"]  # Add more as needed
        
        for robot_id in robot_ids:
            self.robot_status_subs[robot_id] = self.create_subscription(
                String, f'/{robot_id}/status', 
                lambda msg, rid=robot_id: self.handle_robot_status(msg, rid), 10)
    
    def handle_robot_status(self, msg: String, robot_id: str):
        """Handle individual robot status updates"""
        timestamp = datetime.now().strftime("%H:%M:%S")
        status = msg.data
        
        # Color coding for different states
        color_map = {
            "idle": "🟢",
            "moving_to_arm": "🟡", 
            "picking": "🟠",
            "moving_to_user": "🔵",
            "waiting_confirm": "🟣",
            "returning_to_dock": "⚪"
        }
        
        color = color_map.get(status, "⚫")
        print(f"[{timestamp}] {color} {robot_id}: {status.upper()}")
    
    def handle_fleet_status(self, msg: String):
        """Handle fleet-level status updates"""
        try:
            data = json.loads(msg.data)
            timestamp = datetime.now().strftime("%H:%M:%S")
            
            robot_id = data.get('robot_id', 'unknown')
            status = data.get('status', 'unknown')
            task_id = data.get('current_task_id', 'none')
            
            print(f"[{timestamp}] 🌐 Fleet: {robot_id} -> {status} (Task: {task_id})")
        except json.JSONDecodeError:
            pass

def main():
    rclpy.init()
    monitor = FSMMonitor()
    
    try:
        print("Press Ctrl+C to stop monitoring...")
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        print("\n🛑 FSM Monitor stopped")
    finally:
        monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()