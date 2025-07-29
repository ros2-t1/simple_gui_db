#!/usr/bin/env python3
# Fleet-compatible robot node
import rclpy
from rclpy.executors import MultiThreadedExecutor
import sys
import os

# Add parent directory to path
sys.path.append(os.path.dirname(os.path.dirname(__file__)))

from robot.nav2_waypoint_class import WaypointNavigator
from robot.fsm import DeliveryFSM

def main():
    # Get robot ID from command line argument or default to robot_1
    robot_id = sys.argv[1] if len(sys.argv) > 1 else "robot_1"
    
    print(f"Starting robot node: {robot_id}")
    
    # Initialize ROS
    rclpy.init()
    
    # Create robot components with namespace
    wp_nav = WaypointNavigator()
    wp_nav._node_name = f"{robot_id}_waypoint_navigator"
    
    fsm = DeliveryFSM(robot_id, wp_nav)
    
    # Create executor and add nodes
    executor = MultiThreadedExecutor()
    executor.add_node(wp_nav)
    executor.add_node(fsm)
    
    try:
        print(f"Robot {robot_id} is ready!")
        executor.spin()
    except KeyboardInterrupt:
        print(f"Shutting down robot {robot_id}")
    finally:
        executor.shutdown()
        rclpy.shutdown()

if __name__ == "__main__":
    main()