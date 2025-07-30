import os

DB_DSN = {
    "dbname": "hana_db",
    "user": "postgres",
    "password": "qwer1234",
    "host": "20.249.209.1",
    "port": "5432",
}

ROS_CMD_TOPIC   = "user_cmd"
ROS_STAT_TOPIC  = "status"

PICKUP_ST1      = [0.33, -0.33, 0.707]
SERVICE_ST1   = [0.1, 0.78, -0.707]
CHARGING_ST    = [0.0, 0.0, 1.0]

GLOBAL_CAMERA_IP= '224.1.1.1'
GLOBAL_CAMERA_PORT = 5000

# Robot camera configurations
ROBOT_CAMERAS = {
    'hanabot_3': {
        'ip': '224.1.1.1',
        'port': 5003,
        'name': 'HanaBot 3',
        'location': 'Station A'
    },
    'hanabot_8': {
        'ip': '224.1.1.1', 
        'port': 5008,
        'name': 'HanaBot 8',
        'location': 'Station B'
    },
    'hanabot_9': {
        'ip': '224.1.1.1',
        'port': 5009,
        'name': 'HanaBot 9', 
        'location': 'Station C'
    }
}

