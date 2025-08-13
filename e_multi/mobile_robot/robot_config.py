# Robot Configuration
# This file contains robot-specific settings that don't require database access
# Used for distributed deployment where robots run on separate machines

# ROS2 Topics
ROS_CMD_TOPIC   = "user_cmd"
ROS_STAT_TOPIC  = "status"

# Nav2 waypoints (rough approach positions)
PICKUP_ST1_NAV     = [0.241, -0.227, 0.707]  # Arm pickup station (Nav2 target)
SERVICE_ST1        = [0.1, 0.78, 0.707]     # Default service station (fallback)
CHARGING_ST_NAV    = [-0.12, 0.21, -1.0]      # Charging dock (Nav2 target)

# Precision parking positions (fine-tuned final positions)
PICKUP_ST1_PARKING = [0.241, -0.227, 0.707]  # Arm precise parking position
CHARGING_ST_PARKING = [-0.2, 0.21, -1.0]     # Dock precise parking position

# Service station parking configuration
SERVICE_PARKING_ENABLED = True       # Set True to enable precision parking at service stations
SERVICE_PARKING_YAW = -1.57          # Final orientation at service stations (radians)

# Room-specific parking coordinates
# Key: location_name from DB, Value: [x, y] parking coordinates
SERVICE_PARKING_COORDS = {
    "ROOM1": [-0.05, 0.75, -3.14],    # Room 1 precise parking position (DB: ROOM1)
    "ROOM2": [0.35, 0.75, 3.14],    # Room 2 precise parking position (DB: ROOM2)
    # Add more rooms as needed (must match location_name in DB exactly)
}

# # Fallback: If room not in SERVICE_PARKING_COORDS, use offset
# SERVICE_PARKING_OFFSET = [0.0, 0.05]  # Default offset if room not configured

# Legacy names for backward compatibility
PICKUP_ST1 = PICKUP_ST1_NAV
CHARGING_ST = CHARGING_ST_NAV

# Fleet Manager connection (for distributed deployment)
FLEET_MANAGER_HOST = "localhost"  # Change this to Fleet Manager's IP
ROS_DOMAIN_ID = 0  # Set same domain ID across all machines

# Robot camera configuration (per robot)
ROBOT_CAMERA_PORT = 5008  # Default port, can be overridden per robot