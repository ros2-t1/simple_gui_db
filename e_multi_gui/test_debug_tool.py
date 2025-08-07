#!/usr/bin/env python3
"""
Test script for the robot debug tool
This script tests the basic functionality of the debug endpoints
"""

import requests
import json
import time

BASE_URL = "http://localhost:8080"

def test_debug_endpoints():
    """Test all debug endpoints"""
    
    print("🧪 Testing Robot Debug Tool")
    print("="*40)
    
    try:
        # Test 1: Get robot status
        print("1. Testing robot status endpoint...")
        response = requests.get(f"{BASE_URL}/robot_status")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✅ Robot status: {len(data.get('robots', {}))} robots found")
        else:
            print(f"   ❌ Failed: {response.status_code}")
        
        # Test 2: Get active tasks
        print("\n2. Testing active tasks endpoint...")
        response = requests.get(f"{BASE_URL}/debug/active_tasks")
        if response.status_code == 200:
            data = response.json()
            print(f"   ✅ Active tasks: {len(data.get('tasks', []))} tasks found")
        else:
            print(f"   ❌ Failed: {response.status_code}")
        
        # Test 3: Update robot status
        print("\n3. Testing robot status update...")
        response = requests.post(
            f"{BASE_URL}/debug/robot_status/robot_1",
            json={"status": "moving_to_arm"}
        )
        if response.status_code == 200:
            print("   ✅ Robot status updated successfully")
        else:
            print(f"   ❌ Failed: {response.status_code}")
        
        # Test 4: Create test task
        print("\n4. Testing test task creation...")
        response = requests.post(
            f"{BASE_URL}/debug/create_test_task",
            json={"task_type": "배달", "resident_id": 9999}
        )
        if response.status_code == 200:
            data = response.json()
            task_id = data.get('task_id')
            print(f"   ✅ Test task created: ID {task_id}")
            
            # Test 5: Update task status
            if task_id:
                print("\n5. Testing task status update...")
                response = requests.post(
                    f"{BASE_URL}/debug/task_status/{task_id}",
                    json={"status": "완료"}
                )
                if response.status_code == 200:
                    print("   ✅ Task status updated successfully")
                else:
                    print(f"   ❌ Failed: {response.status_code}")
        else:
            print(f"   ❌ Failed: {response.status_code}")
        
        # Test 6: Reset robot
        print("\n6. Testing robot reset...")
        response = requests.post(f"{BASE_URL}/debug/robot_reset/robot_1")
        if response.status_code == 200:
            print("   ✅ Robot reset successfully")
        else:
            print(f"   ❌ Failed: {response.status_code}")
        
        # Test 7: Get system state
        print("\n7. Testing system state endpoint...")
        response = requests.get(f"{BASE_URL}/debug/system_state")
        if response.status_code == 200:
            print("   ✅ System state retrieved successfully")
        else:
            print(f"   ❌ Failed: {response.status_code}")
        
        print("\n" + "="*40)
        print("🎉 All tests completed!")
        print("\n📍 Visit the debug interface at:")
        print(f"   {BASE_URL}/robot_debug")
        
    except requests.exceptions.ConnectionError:
        print("❌ Connection failed. Make sure the web server is running:")
        print("   python run_debug_mode.py")
        print("   or")
        print("   python run_fleet.py")
        
    except Exception as e:
        print(f"❌ Test failed with error: {e}")

if __name__ == "__main__":
    test_debug_endpoints()