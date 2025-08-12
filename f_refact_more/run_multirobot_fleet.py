#!/usr/bin/env python3
# Multi-robot fleet system launcher with domain bridge support
import subprocess
import threading
import time
import signal
import sys
import os
from web import create_app

# Set ROS domain ID for central server (Fleet Manager + Web Server)
os.environ['ROS_DOMAIN_ID'] = '129'

# Process list to track subprocesses
processes = []

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\n🛑 Shutting down multi-robot fleet system...")
    for process in processes:
        if process.poll() is None:  # Still running
            process.terminate()
    
    # Wait for processes to terminate
    for process in processes:
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
    
    sys.exit(0)

def main():
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    print("🚀 Starting Multi-Robot Fleet System...")
    print("📡 Central Domain: 129")
    print("🌉 Domain Bridge: Auto-configured for robot domains 18, 19")
    print("🤖 Robot Domains: robot_1=18, robot_2=19")
    print("=" * 70)
    
    # 1. Start Fleet Manager (includes domain bridge)
    print("🤖 Starting Fleet Manager with Domain Bridge...")
    fleet_manager_proc = subprocess.Popen([
        sys.executable, "fleet_manager/fleet_manager.py"
    ])
    processes.append(fleet_manager_proc)
    time.sleep(5)  # Wait for fleet manager and bridge to initialize
    
    # 2. Optional: Start robots locally (for testing)
    start_local_robots = input("🤔 Start robots locally for testing? (y/N): ").strip().lower()
    if start_local_robots == 'y':
        print("🤖 Starting Robot 1 (Domain 18)...")
        robot1_proc = subprocess.Popen([
            sys.executable, "mobile_robot/robot_nodes/robot_node.py", "robot_1"
        ])
        processes.append(robot1_proc)
        time.sleep(2)
        
        print("🤖 Starting Robot 2 (Domain 19)...")
        robot2_proc = subprocess.Popen([
            sys.executable, "mobile_robot/robot_nodes/robot_node.py", "robot_2"
        ])
        processes.append(robot2_proc)
        time.sleep(2)
    else:
        print("📍 Robots should be started manually:")
        print("   Robot Machine 1: ./mobile_robot/start_robot.sh robot_1")
        print("   Robot Machine 2: ./mobile_robot/start_robot.sh robot_2")
    
    # 3. Start Web Server in main thread
    print("🌐 Starting Web Server...")
    print("📱 Multi-Robot Dashboard: http://localhost:8080")
    app = create_app()
    
    try:
        # Run Flask app
        app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)

if __name__ == "__main__":
    main()