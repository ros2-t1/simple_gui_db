#!/usr/bin/env python3
# Fleet system launcher - PC (Central Server) only
import subprocess
import threading
import time
import signal
import sys
import os
import rclpy
from web import create_app
from web.db import query_db

# Set ROS domain ID for consistent communication
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
    print("📍 Note: Robot nodes should be started separately on robot machines") 
    print("📂 Robot code is available in: mobile_robot/")
    print("=" * 60)
    
    # Initialize ROS2 once for the entire process
    print("🔧 Initializing ROS2...")
    rclpy.init()
    
    # 1. Start Fleet Manager in same process
    print("🤖 Starting Fleet Manager...")
    from fleet_manager.fleet_manager import FleetManager
    import threading
    
    fleet_manager = FleetManager()
    
    # Subscribe to confirm requests
    from std_msgs.msg import String
    fleet_manager.confirm_sub = fleet_manager.create_subscription(
        String, '/fleet/confirm_request', fleet_manager.handle_confirm_request, 10)
    
    # Start fleet manager in background thread
    def run_fleet_manager():
        try:
            rclpy.spin(fleet_manager)
        except Exception as e:
            print(f"Fleet manager error: {e}")
    
    fleet_thread = threading.Thread(target=run_fleet_manager, daemon=True)
    fleet_thread.start()
    time.sleep(2)  # Wait for fleet manager to initialize
    
    # 2. Start Web Server in main thread  
    print("🌐 Starting Web Server...")
    print("📱 Dashboard will be available at: http://localhost:8080")
    app = create_app()
    
    try:
        # Run Flask app
        app.run(host="0.0.0.0", port=8080, debug=False, use_reloader=False)
    except KeyboardInterrupt:
        signal_handler(signal.SIGINT, None)
    finally:
        fleet_manager.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
