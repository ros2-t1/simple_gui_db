from flask import Blueprint, jsonify
from ..fleet_client import get_fleet_client
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

@bp.route("/robot_status/<robot_id>", methods=["GET"])
def get_specific_robot_status(robot_id: str):
    """Get status of specific robot"""
    try:
        if robot_id in _robot_status_cache:
            robot_info = _robot_status_cache[robot_id]
            return jsonify({
                "success": True,
                "robot_id": robot_id,
                "status": robot_info["status"],
                "current_task_id": robot_info["current_task_id"],
                "is_waiting_confirm": robot_info["status"] == "waiting_confirm",
                "last_update": robot_info["last_update"]
            })
        else:
            return jsonify({
                "success": False,
                "error": "Robot not found"
            }), 404
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500