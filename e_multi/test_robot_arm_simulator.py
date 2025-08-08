#!/usr/bin/env python3
"""
Robot Arm Simulator with GUI for testing robot arm integration.
Subscribes to /robot_arm/user_cmd (Int32) and publishes "complete" to /robot_arm/status (String)
when the user clicks the complete button.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32
import time
import threading
import tkinter as tk
from tkinter import ttk


class RobotArmSimulator(Node):
    def __init__(self):
        super().__init__('robot_arm_simulator')
        
        # Subscribe to commands
        self.create_subscription(Int32, '/robot_arm/user_cmd', self.on_command, 10)
        
        # Publisher for status
        self.status_pub = self.create_publisher(String, '/robot_arm/status', 10)
        
        self.get_logger().info('Robot Arm Simulator started')
        self.get_logger().info('Listening on /robot_arm/user_cmd for item_id commands')
        
        # Track current task
        self.current_item = None
        self.work_timer = None
        
        # GUI elements
        self.window = None
        self.status_label = None
        self.item_label = None
        self.complete_button = None
        
        # Start GUI in separate thread
        self.gui_thread = threading.Thread(target=self.create_gui, daemon=True)
        self.gui_thread.start()
        
    def create_gui(self):
        """Create the GUI window"""
        self.window = tk.Tk()
        self.window.title("Robot Arm Simulator")
        self.window.geometry("400x300")
        
        # Title
        title_label = tk.Label(self.window, text="Robot Arm Simulator", font=("Arial", 16, "bold"))
        title_label.pack(pady=10)
        
        # Status frame
        status_frame = tk.Frame(self.window)
        status_frame.pack(pady=20)
        
        tk.Label(status_frame, text="Status:", font=("Arial", 12)).grid(row=0, column=0, sticky="e", padx=5)
        self.status_label = tk.Label(status_frame, text="Waiting for command", font=("Arial", 12, "bold"), fg="gray")
        self.status_label.grid(row=0, column=1, sticky="w")
        
        # Item frame
        item_frame = tk.Frame(self.window)
        item_frame.pack(pady=10)
        
        tk.Label(item_frame, text="Current Item ID:", font=("Arial", 12)).grid(row=0, column=0, sticky="e", padx=5)
        self.item_label = tk.Label(item_frame, text="None", font=("Arial", 12, "bold"))
        self.item_label.grid(row=0, column=1, sticky="w")
        
        # Complete button
        self.complete_button = tk.Button(
            self.window,
            text="Complete Task",
            command=self.complete_work,
            font=("Arial", 14),
            bg="#4CAF50",
            fg="white",
            padx=20,
            pady=10,
            state="disabled"
        )
        self.complete_button.pack(pady=30)
        
        # Instructions
        instructions = tk.Label(
            self.window,
            text="Click 'Complete Task' when robot arm work is done\n(simulating manual operation)",
            font=("Arial", 10),
            fg="gray"
        )
        instructions.pack(pady=10)
        
        # Run GUI main loop
        self.window.mainloop()
        
    def on_command(self, msg: Int32):
        """Handle incoming item_id command"""
        item_id = msg.data
        self.get_logger().info(f'Received command to pick item_id: {item_id}')
        
        # Cancel any existing work timer
        if self.work_timer:
            self.destroy_timer(self.work_timer)
            
        self.current_item = item_id
        
        # Update GUI (thread-safe)
        if self.window:
            self.window.after(0, self.update_gui_working, item_id)
        
    def update_gui_working(self, item_id):
        """Update GUI to show working state"""
        self.status_label.config(text="Working...", fg="orange")
        self.item_label.config(text=str(item_id))
        self.complete_button.config(state="normal")
        
    def update_gui_idle(self):
        """Update GUI to show idle state"""
        self.status_label.config(text="Waiting for command", fg="gray")
        self.item_label.config(text="None")
        self.complete_button.config(state="disabled")
        
    def complete_work(self):
        """Complete the current work and send status"""
        if self.current_item:
            self.get_logger().info(f'Work complete for item {self.current_item}')
            
            # Send complete status
            status_msg = String()
            status_msg.data = "complete"
            self.status_pub.publish(status_msg)
            
            self.get_logger().info('Sent "complete" status to /robot_arm/status')
            
            # Update GUI
            if self.window:
                self.window.after(0, self.update_gui_idle)
            
            # Reset state
            self.current_item = None
            
        # Cancel the timer after completion
        if self.work_timer:
            self.destroy_timer(self.work_timer)
            self.work_timer = None


def main(args=None):
    rclpy.init(args=args)
    
    arm_simulator = RobotArmSimulator()
    
    try:
        rclpy.spin(arm_simulator)
    except KeyboardInterrupt:
        pass
    finally:
        arm_simulator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()