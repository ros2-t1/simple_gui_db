#!/usr/bin/env python3
# Fleet system launcher - PC (Central Server) only
import subprocess
import threading
import time
import signal
import sys
import os
from web import create_app
from web.db import query_db

# Set ROS domain ID for central server (Fleet Manager + Web Server)
os.environ['ROS_DOMAIN_ID'] = '129'

# Process list to track subprocesses
processes = []

def signal_handler(sig, frame):
    """Handle Ctrl+C gracefully"""
    print("\nShutting down central fleet system...")
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
    
    print("🚀 Starting Fleet Central Server...")
    print("📡 Running on ROS Domain: 129")
    print("📍 Note: Robot nodes should be started separately on robot machines")
    print("📂 Robot code is available in: mobile_robot/")
    print("🌉 Domain Bridge: Will be started automatically by Fleet Manager")
    print("🤖 Robot Domains: robot_1=18, robot_2=19")
    print("=" * 60)
    
    # 1. Start Fleet Manager
    print("🤖 Starting Fleet Manager...")
    fleet_manager_proc = subprocess.Popen([
        sys.executable, "fleet_manager/fleet_manager.py"
    ])
    processes.append(fleet_manager_proc)
    time.sleep(3)  # Wait for fleet manager to initialize
    
    # 2. Start Web Server in main thread
    print("🌐 Starting Web Server...")
    print("📱 Dashboard will be available at: http://localhost:8080")
    app = create_app()
    
    try:
        # Run Flask app
        app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)

if __name__ == "__main__":
    main()
