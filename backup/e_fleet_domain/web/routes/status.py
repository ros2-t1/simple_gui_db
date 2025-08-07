from flask import Blueprint, jsonify, request, session
from ..fleet_client import get_fleet_client
from ..db import query_db
from ..task_db import get_active_tasks_by_bot, get_robot_current_task
import json
import threading
import time

bp = Blueprint("status", __name__)

# Global variable to store latest robot status
_robot_status_cache = {
    "robot_1": {
        "status": "idle",
        "current_task_id": None,
        "last_update": time.time()
    }
}

def update_robot_status_cache(robot_id: str, status: str, task_id: str = None):
    """Update robot status cache"""
    global _robot_status_cache
    _robot_status_cache[robot_id] = {
        "status": status,
        "current_task_id": task_id,
        "last_update": time.time()
    }

def get_robot_status_cache(robot_id: str = None):
    """Get robot status from cache"""
    global _robot_status_cache
    if robot_id:
        return _robot_status_cache.get(robot_id)
    return _robot_status_cache

def initialize_robot_status_from_db():
    """Initialize robot status cache - will be updated by Fleet Manager"""
    global _robot_status_cache
    
    try:
        print(f"🔄 Initializing empty robot status cache...")
        print(f"📡 Robot states will be populated by Fleet Manager via ROS2")
        
        # Start with empty cache - Fleet Manager will populate it
        _robot_status_cache.clear()
        
        print(f"✅ Status cache initialized. Waiting for Fleet Manager updates...")
        
    except Exception as e:
        print(f"❌ Error initializing robot status cache: {e}")
        # Keep default values if initialization fails

@bp.route("/robot_status", methods=["GET"])
def get_robot_status():
    """Get current robot status"""
    try:
        return jsonify({
            "success": True,
            "robots": _robot_status_cache
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/force_cache_refresh", methods=["POST"])
def force_cache_refresh():
    """Force refresh robot status cache from database"""
    try:
        initialize_robot_status_from_db()
        return jsonify({
            "success": True,
            "message": "Cache refreshed successfully",
            "cache": _robot_status_cache
        })
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/robot_status/<robot_id>", methods=["GET"])
def get_specific_robot_status(robot_id: str):
    """Get status of specific robot from database"""
    try:
        # Map robot_id to DB ID
        robot_to_db_id = {"robot_1": 8, "robot_2": 9}
        bot_id = robot_to_db_id.get(robot_id)
        
        if not bot_id:
            return jsonify({"error": "Robot not found"}), 404
            
        # Get robot info from DB
        with query_db() as cur:
            cur.execute("""
                SELECT hana_bot_id, bot_name, status, battery 
                FROM hana_bots 
                WHERE hana_bot_id = %s
            """, (bot_id,))
            robot_data = cur.fetchone()
            
        if not robot_data:
            return jsonify({"error": "Robot not found"}), 404
            
        # Get current task (simplified)
        current_task_id = None
        try:
            with query_db() as cur:
                cur.execute("""
                    SELECT task_id FROM tasks 
                    WHERE assigned_bot_id = %s AND status IN ('할당', '이동중', '집기중', '수령대기')
                    ORDER BY created_at DESC LIMIT 1
                """, (bot_id,))
                task_result = cur.fetchone()
                current_task_id = str(task_result[0]) if task_result else None
        except:
            current_task_id = None
        
        # Check if current task belongs to logged-in user
        is_my_order = False
        resident_id = request.args.get('resident_id') or session.get('resident_id')
        
        if current_task_id and resident_id:
            try:
                with query_db() as cur:
                    cur.execute("""
                        SELECT requester_resident_id 
                        FROM tasks 
                        WHERE task_id = %s
                    """, (int(current_task_id),))
                    result = cur.fetchone()
                    
                    if result and result[0] == int(resident_id):
                        is_my_order = True
            except Exception as e:
                print(f"Error checking task ownership: {e}")
        
        # Map DB status to display status
        try:
            db_status = robot_data['status'] or '오프라인'
        except (KeyError, TypeError):
            db_status = '오프라인'
        
        return jsonify({
            "success": True,
            "robot_id": robot_id,
            "status": db_status,
            "current_task_id": current_task_id,
            "is_waiting_confirm": db_status == "수령대기" and is_my_order,
            "is_my_order": is_my_order,
            "last_update": None  # DB doesn't track this
        })
    except Exception as e:
        print(f"❌ robot_status API error: {e}")
        import traceback
        traceback.print_exc()
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500