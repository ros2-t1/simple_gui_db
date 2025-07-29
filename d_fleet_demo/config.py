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
