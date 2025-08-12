# Robot Configuration
# This file contains robot-specific settings that don't require database access
# Used for distributed deployment where robots run on separate machines

# Import shared constants from main config
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
from config import ROS_CMD_TOPIC, ROS_STAT_TOPIC, PICKUP_ST1, SERVICE_ST1, CHARGING_ST

# Fleet Manager connection (for distributed deployment)
FLEET_MANAGER_HOST = "localhost"  # Change this to Fleet Manager's IP
ROS_DOMAIN_ID = 0  # Set same domain ID across all machines

# Robot camera configuration (per robot)
ROBOT_CAMERA_PORT = 5000  # Default port, can be overridden per robot