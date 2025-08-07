#!/usr/bin/env python3
"""
Robot deployment script for distributed fleet system
Use this script to deploy robot components to a separate machine
"""

import os
import sys
import argparse
import shutil
from pathlib import Path

def create_robot_package(robot_name: str, fleet_manager_ip: str, output_dir: str = "robot_package"):
    """Create a deployable robot package"""
    
    print(f"Creating robot package for {robot_name}...")
    
    # Create output directory
    package_dir = Path(output_dir)
    package_dir.mkdir(exist_ok=True)
    
    # Copy required directories
    dirs_to_copy = ["robot", "robot_nodes", "shared"]
    
    for dir_name in dirs_to_copy:
        src_dir = Path(dir_name)
        if src_dir.exists():
            dest_dir = package_dir / dir_name
            if dest_dir.exists():
                shutil.rmtree(dest_dir)
            shutil.copytree(src_dir, dest_dir)
            print(f"Copied {dir_name}/ -> {dest_dir}")
        else:
            print(f"Warning: {dir_name} directory not found")
    
    # Copy robot config
    shutil.copy("robot_config.py", package_dir / "robot_config.py")
    print(f"Copied robot_config.py")
    
    # Update robot config with Fleet Manager IP
    config_file = package_dir / "robot_config.py"
    with open(config_file, 'r') as f:
        content = f.read()
    
    content = content.replace(
        'FLEET_MANAGER_HOST = "localhost"',
        f'FLEET_MANAGER_HOST = "{fleet_manager_ip}"'
    )
    
    with open(config_file, 'w') as f:
        f.write(content)
    
    # Create startup script with proper domain ID
    startup_script = package_dir / "start_robot.sh"
    
    # Determine domain ID based on robot name
    domain_id = "18" if robot_name == "robot_1" else "19" if robot_name == "robot_2" else "20"
    
    with open(startup_script, 'w') as f:
        f.write(f"""#!/bin/bash
# Robot startup script for {robot_name}
# Make sure ROS2 is sourced before running this script

echo "Starting robot {robot_name}..."
echo "Fleet Manager: {fleet_manager_ip}"
echo "Robot Domain: {domain_id}"

# Set ROS domain ID for this robot
export ROS_DOMAIN_ID={domain_id}

# Start robot node
python3 robot_nodes/robot_node.py {robot_name}
""")
    
    startup_script.chmod(0o755)
    
    # Create requirements file
    requirements_file = package_dir / "requirements.txt"
    with open(requirements_file, 'w') as f:
        f.write("""rclpy
std_msgs
geometry_msgs
# Add any additional robot-specific dependencies here
""")
    
    # Create README for deployment
    readme_file = package_dir / "README_DEPLOYMENT.md"
    with open(readme_file, 'w') as f:
        f.write(f"""# Robot {robot_name} Deployment Package

## Prerequisites
1. Install ROS2 (Humble recommended)
2. Install Python dependencies: `pip install -r requirements.txt`
3. Ensure network connectivity to Fleet Manager at {fleet_manager_ip}

## Deployment Steps
1. Copy this entire package to the robot computer
2. Source ROS2: `source /opt/ros/humble/setup.bash`
3. Domain ID is automatically set to {domain_id} for {robot_name}
4. Ensure Domain Bridge is running on server
5. Run: `./start_robot.sh`

## Configuration
- Edit `robot_config.py` to adjust robot-specific settings
- The Fleet Manager IP is set to: {fleet_manager_ip}

## Network Requirements
- Robot must be able to reach Fleet Manager on ROS2 DDS ports (7400, 7401, etc.)
- Ensure firewall allows ROS2 traffic

## Troubleshooting
- Check `ros2 topic list` to see available topics
- Verify connection with `ros2 topic echo /fleet/task_request`
- Check logs for database connection errors (should not occur in robot package)
""")
    
    print(f"\n✅ Robot package created in: {package_dir}")
    print(f"📁 Package contents:")
    for item in package_dir.rglob("*"):
        if item.is_file():
            print(f"  - {item.relative_to(package_dir)}")
    
    print(f"\n🚀 Deployment instructions:")
    print(f"1. Copy '{package_dir}' directory to robot computer")
    print(f"2. On robot: cd {package_dir} && ./start_robot.sh")


def main():
    parser = argparse.ArgumentParser(description="Create robot deployment package")
    parser.add_argument("robot_name", help="Robot name (e.g., robot_1, robot_2)")
    parser.add_argument("--fleet-manager-ip", required=True, 
                       help="IP address of the Fleet Manager")
    parser.add_argument("--output", default="robot_package", 
                       help="Output directory name (default: robot_package)")
    
    args = parser.parse_args()
    
    create_robot_package(args.robot_name, args.fleet_manager_ip, args.output)

if __name__ == "__main__":
    main()