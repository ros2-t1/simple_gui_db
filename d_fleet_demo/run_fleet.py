#!/usr/bin/env python3
# Fleet system launcher
import subprocess
import threading
import time
import signal
import sys
from web import create_app

# Process list to track subprocesses
processes = []

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutting down fleet system...")
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
    
    print("Starting Fleet System...")
    
    # 1. Start Fleet Manager
    print("Starting Fleet Manager...")
    fleet_manager_proc = subprocess.Popen([
        sys.executable, "fleet_manager/fleet_manager.py"
    ])
    processes.append(fleet_manager_proc)
    time.sleep(2)  # Wait for fleet manager to initialize
    
    # 2. Start Robot Nodes
    robot_ids = ["robot_1"]  # Can add more robots here
    
    for robot_id in robot_ids:
        print(f"Starting {robot_id}...")
        robot_proc = subprocess.Popen([
            sys.executable, "robot_nodes/robot_node.py", robot_id
        ])
        processes.append(robot_proc)
        time.sleep(1)
    
    print("All robot nodes started")
    
    # 3. Start Web Server in main thread
    print("Starting Web Server...")
    app = create_app()
    
    try:
        # Run Flask app
        app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)

if __name__ == "__main__":
    main()