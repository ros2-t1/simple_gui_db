from flask import Blueprint, jsonify, request, render_template
from ..db import query_db, update_db
from ..task_db import get_active_tasks_by_bot
from .status import update_robot_status_cache, get_robot_status_cache
import time
import json

bp = Blueprint("debug", __name__)

@bp.route("/robot_debug")
def robot_debug():
    """Show robot debug interface"""
    return render_template("robot_debug.html")

@bp.route("/debug/robot_status/<robot_id>", methods=["POST"])
def update_robot_debug_status(robot_id: str):
    """Update robot status for debugging purposes"""
    try:
        data = request.get_json()
        new_status = data.get('status')
        
        if not new_status:
            return jsonify({
                "success": False,
                "error": "Status is required"
            }), 400
        
        # Get current robot state
        current_state = get_robot_status_cache(robot_id)
        if not current_state:
            return jsonify({
                "success": False,
                "error": f"Robot {robot_id} not found"
            }), 404
        
        # Update robot status cache
        update_robot_status_cache(
            robot_id=robot_id,
            status=new_status,
            task_id=current_state.get('current_task_id')
        )
        
        # If robot has a current task, update database status accordingly
        current_task_id = current_state.get('current_task_id')
        if current_task_id and new_status in ['waiting_confirm', 'idle', 'returning_to_dock']:
            # Map robot status to database status
            db_status_map = {
                'waiting_confirm': '수령대기',
                'idle': '완료',
                'returning_to_dock': '완료'
            }
            
            if new_status in db_status_map:
                try:
                    with query_db() as cur:
                        cur.execute("""
                            UPDATE tasks SET status = %s 
                            WHERE task_id = %s
                        """, (db_status_map[new_status], int(current_task_id)))
                        
                        # If completing task, also update completion time
                        if new_status in ['idle', 'returning_to_dock']:
                            cur.execute("""
                                UPDATE tasks SET completed_at = NOW()
                                WHERE task_id = %s
                            """, (int(current_task_id),))
                            
                            # Clear robot's current task
                            update_robot_status_cache(robot_id, new_status, None)
                            current_task_id = None  # Clear for response
                            
                except Exception as e:
                    print(f"Failed to update database status: {e}")
        
        return jsonify({
            "success": True,
            "robot_id": robot_id,
            "new_status": new_status,
            "message": f"Robot {robot_id} status updated to {new_status}"
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/debug/robot_task/<robot_id>", methods=["POST"])
def update_robot_debug_task(robot_id: str):
    """Update robot's current task ID for debugging"""
    try:
        data = request.get_json()
        task_id = data.get('task_id')
        
        # Get current robot state
        current_state = get_robot_status_cache(robot_id)
        if not current_state:
            return jsonify({
                "success": False,
                "error": f"Robot {robot_id} not found"
            }), 404
        
        # Update robot status cache with new task ID
        update_robot_status_cache(
            robot_id=robot_id,
            status=current_state.get('status'),
            task_id=str(task_id) if task_id else None
        )
        
        return jsonify({
            "success": True,
            "robot_id": robot_id,
            "new_task_id": task_id,
            "message": f"Robot {robot_id} task ID updated to {task_id or 'None'}"
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/debug/robot_reset/<robot_id>", methods=["POST"])
def reset_robot_debug(robot_id: str):
    """Reset robot to idle state with no task"""
    try:
        # Reset robot status cache
        update_robot_status_cache(robot_id, 'idle', None)
        
        return jsonify({
            "success": True,
            "robot_id": robot_id,
            "message": f"Robot {robot_id} reset to idle state"
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/debug/active_tasks")
def get_active_tasks_debug():
    """Get all active tasks for debugging"""
    try:
        # Get active tasks from database
        with query_db() as cur:
            cur.execute("""
                SELECT task_id, task_type, requester_resident_id, status, 
                       assigned_bot_id, created_at, completed_at
                FROM tasks 
                WHERE status NOT IN ('완료', '실패')
                ORDER BY created_at DESC
            """)
            
            tasks = []
            for row in cur.fetchall():
                tasks.append({
                    'task_id': row[0],
                    'task_type': row[1], 
                    'requester_resident_id': row[2],
                    'status': row[3],
                    'assigned_bot_id': row[4],
                    'created_at': row[5].isoformat() if row[5] else None,
                    'completed_at': row[6].isoformat() if row[6] else None
                })
        
        return jsonify({
            "success": True,
            "tasks": tasks
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/debug/create_test_task", methods=["POST"])
def create_test_task():
    """Create a test task for debugging purposes"""
    try:
        data = request.get_json()
        task_type = data.get('task_type', '배달')
        resident_id = data.get('resident_id', 9999)
        
        # Validate resident exists or create test resident
        with query_db() as cur:
            cur.execute("SELECT resident_id FROM residents WHERE resident_id = %s", (resident_id,))
            if not cur.fetchone():
                # Create test resident
                cur.execute("""
                    INSERT INTO residents (resident_id, name, phone, room_number, unit_id)
                    VALUES (%s, %s, %s, %s, %s)
                    ON CONFLICT (resident_id) DO NOTHING
                """, (resident_id, f"테스트주민{resident_id}", f"010-{resident_id}", f"{resident_id}호", 1))
        
        # Create test task
        with query_db() as cur:
            if task_type == '배달':
                # First get a valid item_id from the items table
                cur.execute("SELECT item_id FROM items LIMIT 1")
                result = cur.fetchone()
                item_id = result[0] if result else None
                
                if item_id:
                    # Create delivery task with valid item_id
                    cur.execute("""
                        INSERT INTO tasks (task_type, requester_resident_id, status, item_id)
                        VALUES (%s, %s, %s, %s)
                        RETURNING task_id
                    """, (task_type, resident_id, '대기', item_id))
                else:
                    # No items available, create without item_id
                    cur.execute("""
                        INSERT INTO tasks (task_type, requester_resident_id, status)
                        VALUES (%s, %s, %s)
                        RETURNING task_id
                    """, (task_type, resident_id, '대기'))
            else:
                # Create call task
                cur.execute("""
                    INSERT INTO tasks (task_type, requester_resident_id, status)
                    VALUES (%s, %s, %s)
                    RETURNING task_id
                """, (task_type, resident_id, '대기'))
            
            task_id = cur.fetchone()[0]
        
        return jsonify({
            "success": True,
            "task_id": task_id,
            "task_type": task_type,
            "resident_id": resident_id,
            "message": f"Test task {task_id} created successfully"
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/debug/task_status/<int:task_id>", methods=["POST"])
def update_task_debug_status(task_id: int):
    """Update task status for debugging"""
    try:
        data = request.get_json()
        new_status = data.get('status')
        
        # Map status to database values
        status_map = {
            '완료': '완료',
            '실패': '실패',
            '대기': '대기',
            '할당': '할당',
            '이동중': '이동중',
            '집기중': '집기중',
            '수령대기': '수령대기'
        }
        
        db_status = status_map.get(new_status, new_status)
        
        with query_db() as cur:
            cur.execute("""
                UPDATE tasks SET status = %s
                WHERE task_id = %s
            """, (db_status, task_id))
            
            # If completing or failing task, also update completion time
            if db_status in ['완료', '실패']:
                cur.execute("""
                    UPDATE tasks SET completed_at = NOW()
                    WHERE task_id = %s
                """, (task_id,))
                
                # Find and reset any robot assigned to this task
                cur.execute("""
                    SELECT assigned_bot_id FROM tasks WHERE task_id = %s
                """, (task_id,))
                result = cur.fetchone()
                
                if result and result[0]:
                    bot_id = result[0]
                    # Map bot_id to robot_name
                    bot_to_robot = {8: "robot_1", 9: "robot_2", 13: "robot_3"}
                    if bot_id in bot_to_robot:
                        robot_name = bot_to_robot[bot_id]
                        update_robot_status_cache(robot_name, 'idle', None)
        
        return jsonify({
            "success": True,
            "task_id": task_id,
            "new_status": db_status,
            "message": f"Task {task_id} status updated to {db_status}"
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/debug/complete_delivery/<robot_id>", methods=["POST"])
def complete_delivery_debug(robot_id: str):
    """Complete delivery for debugging - handles both robot and task status"""
    try:
        # Get current robot state
        current_state = get_robot_status_cache(robot_id)
        if not current_state:
            return jsonify({
                "success": False,
                "error": f"Robot {robot_id} not found"
            }), 404
        
        current_task_id = current_state.get('current_task_id')
        
        if not current_task_id:
            return jsonify({
                "success": False,
                "error": f"Robot {robot_id} has no current task"
            }), 400
        
        # Complete the task in database
        with query_db() as cur:
            cur.execute("""
                UPDATE tasks 
                SET status = '완료', completed_at = NOW() 
                WHERE task_id = %s
            """, (int(current_task_id),))
            
            affected_rows = cur.rowcount
        
        if affected_rows > 0:
            # Clear robot's current task and set to idle
            update_robot_status_cache(robot_id, 'idle', None)
            
            return jsonify({
                "success": True,
                "robot_id": robot_id,
                "completed_task_id": current_task_id,
                "message": f"Task {current_task_id} completed for robot {robot_id}"
            })
        else:
            return jsonify({
                "success": False,
                "error": f"Failed to complete task {current_task_id}"
            }), 500
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500

@bp.route("/debug/system_state")
def get_system_state():
    """Get complete system state for debugging"""
    try:
        # Get all robot states
        robot_states = get_robot_status_cache()
        
        # Get all active tasks
        with query_db() as cur:
            cur.execute("""
                SELECT task_id, task_type, requester_resident_id, status, 
                       assigned_bot_id, created_at, completed_at
                FROM tasks 
                WHERE status NOT IN ('완료', '실패')
                ORDER BY created_at DESC
                LIMIT 20
            """)
            
            active_tasks = []
            for row in cur.fetchall():
                active_tasks.append({
                    'task_id': row[0],
                    'task_type': row[1],
                    'requester_resident_id': row[2], 
                    'status': row[3],
                    'assigned_bot_id': row[4],
                    'created_at': row[5].isoformat() if row[5] else None,
                    'completed_at': row[6].isoformat() if row[6] else None
                })
        
        # Get recent completed tasks
        with query_db() as cur:
            cur.execute("""
                SELECT task_id, task_type, requester_resident_id, status, 
                       assigned_bot_id, created_at, completed_at
                FROM tasks 
                WHERE status IN ('완료', '실패')
                ORDER BY completed_at DESC
                LIMIT 10
            """)
            
            recent_completed = []
            for row in cur.fetchall():
                recent_completed.append({
                    'task_id': row[0],
                    'task_type': row[1],
                    'requester_resident_id': row[2],
                    'status': row[3],
                    'assigned_bot_id': row[4],
                    'created_at': row[5].isoformat() if row[5] else None,
                    'completed_at': row[6].isoformat() if row[6] else None
                })
        
        return jsonify({
            "success": True,
            "system_state": {
                "robots": robot_states,
                "active_tasks": active_tasks,
                "recent_completed": recent_completed,
                "timestamp": time.time()
            }
        })
        
    except Exception as e:
        return jsonify({
            "success": False,
            "error": str(e)
        }), 500