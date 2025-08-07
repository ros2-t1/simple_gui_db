#!/bin/bash
# FSM GUI Monitor 시작 스크립트

echo "🚀 Starting FSM GUI Monitor..."
echo "📋 Requirements check..."

# Python 패키지 확인
python3 -c "import tkinter, matplotlib, networkx" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "❌ Missing packages. Installing..."
    pip3 install matplotlib networkx
fi

# ROS2 환경 확인
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo "⚠️  ROS_DOMAIN_ID not set. Using default (0)"
    export ROS_DOMAIN_ID=0
fi

echo "✅ Starting FSM GUI Monitor"
echo "🎯 Make sure Fleet Manager and Robot are running!"
echo "=" * 50

python3 fsm_gui_monitor.py