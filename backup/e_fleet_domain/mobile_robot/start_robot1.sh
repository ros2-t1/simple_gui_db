#!/bin/bash

# Start Robot 1 (Domain 18)
export ROS_DOMAIN_ID=18

echo "Starting Robot 1 (Domain 18)..."
echo "Robot ID: robot_1"

# Start robot node
python robot_nodes/robot_node.py robot_1