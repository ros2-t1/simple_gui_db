# Fleet monitoring dashboard
from flask import Blueprint, render_template, jsonify, request
from ..task_db import get_pending_tasks, get_robot_current_task
from ..routes.status import get_robot_status_cache
from ..fleet_client import get_fleet_client
import psycopg2
from psycopg2.extras import DictCursor
import config as cfg

bp = Blueprint('fleet_monitor', __name__)

def get_db_connection():
    """Get database connection"""
    return psycopg2.connect(**cfg.DB_DSN)

@bp.route('/test')
def fleet_dashboard():
    """Fleet monitoring dashboard page"""
    return render_template('fleet_dashboard.html')

@bp.route('/test2')
def advanced_fleet_dashboard():
    """Advanced fleet analytics dashboard"""
    return render_template('advanced_fleet_dashboard.html')

@bp.route('/test3')
def admin_dashboard():
    """HANA Admin dashboard page"""
    return render_template('admin_dashboard.html')

@bp.route('/api/fleet/status')
def fleet_status_api():
    """API endpoint for fleet status data"""
    try:
        # Get robot status
        robots = []
        robot_ids = ['robot_1']  # Add more robot IDs as needed
        
        for robot_id in robot_ids:
            # Get robot status from cache
            status_data = get_robot_status_cache(robot_id)
            
            # Get current task for robot
            robot_db_id = 10 if robot_id == 'robot_1' else None  # HANA_PINKY ID
            current_task = None
            if robot_db_id:
                current_task = get_robot_current_task(robot_db_id)
            
            robots.append({
                'id': robot_id,
                'name': 'HANA PINKY' if robot_id == 'robot_1' else robot_id,
                'status': status_data.get('status', 'idle') if status_data else 'offline',
                'current_task': current_task,
                'last_update': status_data.get('last_update') if status_data else None,
                'is_waiting_confirm': status_data.get('is_waiting_confirm', False) if status_data else False
            })
        
        # Get task queue
        pending_tasks = get_pending_tasks()
        
        # Get recent completed tasks
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT t.task_id, t.task_type, t.status, t.requester_resident_id, 
                           t.item_id, t.assigned_bot_id, t.created_at, t.completed_at,
                           r.name as requester_name, i.item_type as item_type
                    FROM tasks t
                    LEFT JOIN residents r ON t.requester_resident_id = r.resident_id
                    LEFT JOIN items i ON t.item_id = i.item_id
                    WHERE t.status IN ('완료', '실패')
                    ORDER BY t.completed_at DESC, t.created_at DESC
                    LIMIT 10
                """)
                completed_tasks = [dict(row) for row in cur.fetchall()]
        
        # Get active tasks
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT t.task_id, t.task_type, t.status, t.requester_resident_id, 
                           t.item_id, t.assigned_bot_id, t.created_at,
                           r.name as requester_name, i.item_type as item_type,
                           b.bot_name
                    FROM tasks t
                    LEFT JOIN residents r ON t.requester_resident_id = r.resident_id
                    LEFT JOIN items i ON t.item_id = i.item_id
                    LEFT JOIN hana_bots b ON t.assigned_bot_id = b.hana_bot_id
                    WHERE t.status IN ('할당', '이동중', '집기중', '수령대기')
                    ORDER BY t.created_at ASC
                """)
                active_tasks = [dict(row) for row in cur.fetchall()]
        
        # Calculate fleet statistics
        total_tasks_today = 0
        completed_tasks_today = 0
        
        with get_db_connection() as conn:
            with conn.cursor() as cur:
                cur.execute("""
                    SELECT COUNT(*) FROM tasks 
                    WHERE DATE(created_at) = CURRENT_DATE
                """)
                total_tasks_today = cur.fetchone()[0]
                
                cur.execute("""
                    SELECT COUNT(*) FROM tasks 
                    WHERE DATE(created_at) = CURRENT_DATE AND status = '완료'
                """)
                completed_tasks_today = cur.fetchone()[0]
        
        return jsonify({
            'success': True,
            'data': {
                'robots': robots,
                'task_queue': {
                    'pending': pending_tasks,
                    'active': active_tasks,
                    'completed': completed_tasks
                },
                'statistics': {
                    'total_tasks_today': total_tasks_today,
                    'completed_tasks_today': completed_tasks_today,
                    'success_rate': round(completed_tasks_today / total_tasks_today * 100, 1) if total_tasks_today > 0 else 0,
                    'pending_count': len(pending_tasks),
                    'active_count': len(active_tasks)
                }
            }
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/fleet/analytics')
def fleet_analytics_api():
    """API endpoint for advanced fleet analytics"""
    try:
        # Time-based task statistics (last 24 hours)
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                # Hourly task distribution
                cur.execute("""
                    SELECT 
                        EXTRACT(HOUR FROM created_at) as hour,
                        COUNT(*) as task_count,
                        SUM(CASE WHEN status = '완료' THEN 1 ELSE 0 END) as completed_count,
                        SUM(CASE WHEN status = '실패' THEN 1 ELSE 0 END) as failed_count
                    FROM tasks 
                    WHERE created_at >= NOW() - INTERVAL '24 hours'
                    GROUP BY EXTRACT(HOUR FROM created_at)
                    ORDER BY hour
                """)
                hourly_stats = [dict(row) for row in cur.fetchall()]

                # Robot performance metrics
                cur.execute("""
                    SELECT 
                        b.bot_name,
                        b.hana_bot_id,
                        COUNT(t.task_id) as total_tasks,
                        SUM(CASE WHEN t.status = '완료' THEN 1 ELSE 0 END) as completed_tasks,
                        SUM(CASE WHEN t.status = '실패' THEN 1 ELSE 0 END) as failed_tasks,
                        ROUND(AVG(EXTRACT(EPOCH FROM (t.completed_at - t.created_at))/60), 2) as avg_completion_minutes
                    FROM hana_bots b
                    LEFT JOIN tasks t ON b.hana_bot_id = t.assigned_bot_id 
                        AND t.created_at >= NOW() - INTERVAL '7 days'
                    GROUP BY b.bot_name, b.hana_bot_id
                    ORDER BY total_tasks DESC
                """)
                robot_performance = [dict(row) for row in cur.fetchall()]

                # Top requesters (last 7 days)
                cur.execute("""
                    SELECT 
                        r.name,
                        r.resident_id,
                        COUNT(t.task_id) as order_count,
                        SUM(CASE WHEN t.status = '완료' THEN 1 ELSE 0 END) as completed_orders
                    FROM residents r
                    LEFT JOIN tasks t ON r.resident_id = t.requester_resident_id 
                        AND t.created_at >= NOW() - INTERVAL '7 days'
                    WHERE t.task_id IS NOT NULL
                    GROUP BY r.name, r.resident_id
                    ORDER BY order_count DESC
                    LIMIT 10
                """)
                top_requesters = [dict(row) for row in cur.fetchall()]

                # Popular items (last 7 days)
                cur.execute("""
                    SELECT 
                        i.item_type,
                        i.item_id,
                        COUNT(t.task_id) as order_count,
                        SUM(CASE WHEN t.status = '완료' THEN 1 ELSE 0 END) as delivered_count
                    FROM items i
                    LEFT JOIN tasks t ON i.item_id = t.item_id 
                        AND t.created_at >= NOW() - INTERVAL '7 days'
                    WHERE t.task_id IS NOT NULL
                    GROUP BY i.item_type, i.item_id
                    ORDER BY order_count DESC
                """)
                popular_items = [dict(row) for row in cur.fetchall()]

                # Daily trend (last 30 days)
                cur.execute("""
                    SELECT 
                        DATE(created_at) as date,
                        COUNT(*) as total_tasks,
                        SUM(CASE WHEN status = '완료' THEN 1 ELSE 0 END) as completed_tasks,
                        SUM(CASE WHEN status = '실패' THEN 1 ELSE 0 END) as failed_tasks,
                        ROUND(AVG(EXTRACT(EPOCH FROM (completed_at - created_at))/60), 2) as avg_completion_minutes
                    FROM tasks 
                    WHERE created_at >= NOW() - INTERVAL '30 days'
                    GROUP BY DATE(created_at)
                    ORDER BY date DESC
                    LIMIT 30
                """)
                daily_trends = [dict(row) for row in cur.fetchall()]

                # System performance metrics
                cur.execute("""
                    SELECT 
                        COUNT(*) as total_tasks_today,
                        SUM(CASE WHEN status = '완료' THEN 1 ELSE 0 END) as completed_today,
                        SUM(CASE WHEN status = '실패' THEN 1 ELSE 0 END) as failed_today,
                        SUM(CASE WHEN status IN ('대기', '할당', '이동중', '집기중', '수령대기') THEN 1 ELSE 0 END) as active_today,
                        ROUND(AVG(CASE WHEN status = '완료' AND completed_at IS NOT NULL 
                                      THEN EXTRACT(EPOCH FROM (completed_at - created_at))/60 END), 2) as avg_completion_time,
                        COUNT(DISTINCT requester_resident_id) as unique_users_today,
                        COUNT(DISTINCT assigned_bot_id) as active_robots_today
                    FROM tasks 
                    WHERE DATE(created_at) = CURRENT_DATE
                """)
                system_metrics = dict(cur.fetchone())

        return jsonify({
            'success': True,
            'data': {
                'hourly_distribution': hourly_stats,
                'robot_performance': robot_performance,
                'top_requesters': top_requesters,
                'popular_items': popular_items,
                'daily_trends': daily_trends,
                'system_metrics': system_metrics
            }
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/fleet/control', methods=['POST'])
def fleet_control_api():
    """API endpoint for fleet control actions"""
    try:
        data = request.get_json()
        action = data.get('action')
        target_id = data.get('target_id')
        
        if action == 'cancel_task':
            # Cancel specific task
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        UPDATE tasks 
                        SET status = '실패', completed_at = CURRENT_TIMESTAMP
                        WHERE task_id = %s AND status NOT IN ('완료', '실패')
                    """, (target_id,))
                    conn.commit()
            
            return jsonify({
                'success': True,
                'message': f'Task #{target_id} cancelled successfully'
            })
            
        elif action == 'reset_robot':
            # Reset robot to idle (this would need ROS integration for real implementation)
            return jsonify({
                'success': True,
                'message': f'Robot {target_id} reset command sent'
            })
            
        elif action == 'fail_all_incomplete':
            # Fail all incomplete tasks
            with get_db_connection() as conn:
                with conn.cursor() as cur:
                    cur.execute("""
                        UPDATE tasks 
                        SET status = '실패', completed_at = CURRENT_TIMESTAMP
                        WHERE status NOT IN ('완료', '실패')
                    """)
                    affected_rows = cur.rowcount
                    conn.commit()
            
            return jsonify({
                'success': True,
                'message': f'{affected_rows} incomplete tasks marked as failed'
            })
            
        else:
            return jsonify({
                'success': False,
                'error': 'Unknown action'
            })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

# GPT가 수정함 0730

@bp.route('/api/admin/robots')
def admin_robots_api():
    """API endpoint for all robot data"""
    try:
        robots = []
        # For now, we only have robot_1 (HANA_PINKY) in the fleet manager's direct control
        # In a real system, this would query the database for all registered robots
        # and their real-time status from the fleet manager.
        # For demonstration, we'll combine DB info with cached ROS status.
        
        # Get all robots from DB
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT hana_bot_id, bot_name, battery, status
                    FROM hana_bots
                    ORDER BY hana_bot_id ASC
                """)
                db_robots = {r['hana_bot_id']: dict(r) for r in cur.fetchall()}

        # Get real-time status from cache (from fleet_client)
        from ..routes.status import get_robot_status_cache
        ros_status_cache = get_robot_status_cache()

        for bot_id, db_robot in db_robots.items():
            robot_id_str = f"robot_{bot_id}" if bot_id == 10 else str(bot_id) # Map 10 to robot_1
            ros_status = ros_status_cache.get(robot_id_str, {})
            
            current_task = None
            if bot_id:
                current_task = get_robot_current_task(bot_id)

            robots.append({
                'id': bot_id,
                'name': db_robot['bot_name'],
                'battery': db_robot['battery'],
                'status': ros_status.get('status', db_robot['status'] or 'offline'), # Prefer ROS status
                'current_task': current_task,
                'last_update': ros_status.get('last_update')
            })
        
        return jsonify({
            'success': True,
            'data': robots
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/admin/tasks')
def admin_tasks_api():
    """API endpoint for all task data"""
    try:
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT 
                        t.task_id, t.task_type, t.status, t.created_at, t.completed_at,
                        r.name as requester_name,
                        i.item_type as item_type,
                        b.bot_name
                    FROM tasks t
                    LEFT JOIN residents r ON t.requester_resident_id = r.resident_id
                    LEFT JOIN items i ON t.item_id = i.item_id
                    LEFT JOIN hana_bots b ON t.assigned_bot_id = b.hana_bot_id
                    ORDER BY t.created_at DESC
                """)
                tasks = [dict(row) for row in cur.fetchall()]
        return jsonify({
            'success': True,
            'data': tasks
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/admin/users')
def admin_users_api():
    """API endpoint for all user data"""
    try:
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT resident_id, name, gender, birth_date, login_id
                    FROM residents
                    ORDER BY resident_id ASC
                """)
                users = [dict(row) for row in cur.fetchall()]
        return jsonify({
            'success': True,
            'data': users
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })

@bp.route('/api/admin/items')
def admin_items_api():
    """API endpoint for all item data"""
    try:
        with get_db_connection() as conn:
            with conn.cursor(cursor_factory=DictCursor) as cur:
                cur.execute("""
                    SELECT item_id, item_type, item_quantity
                    FROM items
                    ORDER BY item_id ASC
                """)
                items = [dict(row) for row in cur.fetchall()]
        return jsonify({
            'success': True,
            'data': items
        })
    except Exception as e:
        return jsonify({
            'success': False,
            'error': str(e)
        })