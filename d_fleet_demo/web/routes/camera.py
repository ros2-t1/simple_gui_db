from flask import Blueprint, Response, jsonify, request
import cv2
import threading
import queue
import time
import socket
import struct
import numpy as np
import config as cfg

bp = Blueprint('camera', __name__, url_prefix='/camera')

# Global variables for multiple camera streams
camera_streams = {}  # {camera_id: queue}
active_cameras = {}  # {camera_id: bool}
camera_threads = {}  # {camera_id: thread}

def get_camera_config(camera_id):
    """Get camera configuration by ID"""
    if camera_id == 'global':
        return {
            'ip': cfg.GLOBAL_CAMERA_IP,
            'port': cfg.GLOBAL_CAMERA_PORT,
            'name': 'Global Camera',
            'location': 'Main View'
        }
    
    return cfg.ROBOT_CAMERAS.get(camera_id, None)

def camera_stream_generator(camera_id):
    """Generator function for specific camera stream"""
    while active_cameras.get(camera_id, False):
        try:
            camera_queue = camera_streams.get(camera_id)
            if camera_queue and not camera_queue.empty():
                frame = camera_queue.get_nowait()
                # Clear queue to get latest frame
                while not camera_queue.empty():
                    try:
                        frame = camera_queue.get_nowait()
                    except queue.Empty:
                        break
                
                # Encode frame as JPEG
                ret, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if ret:
                    frame_bytes = buffer.tobytes()
                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            else:
                time.sleep(0.03)  # ~30 FPS
        except Exception as e:
            print(f"Camera {camera_id} stream error: {e}")
            break

def udp_camera_worker(camera_id, camera_config):
    """Worker thread for UDP camera stream"""
    try:
        # UDP socket setup
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.settimeout(1.0)  # 1 second timeout
        sock.bind(('', camera_config['port']))
        mreq = struct.pack("4sl", socket.inet_aton(camera_config['ip']), socket.INADDR_ANY)
        sock.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)
        
        print(f"Camera {camera_id} listening on {camera_config['ip']}:{camera_config['port']}")
        
        # Initialize queue for this camera
        camera_streams[camera_id] = queue.Queue(maxsize=2)
        
        while active_cameras.get(camera_id, False):
            try:
                data, _ = sock.recvfrom(65536)
                np_arr = np.frombuffer(data, np.uint8)
                frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
                if frame is not None:
                    # Put frame in queue (non-blocking)
                    camera_queue = camera_streams[camera_id]
                    if not camera_queue.full():
                        camera_queue.put(frame)
                    
            except socket.timeout:
                continue
            except Exception as e:
                if active_cameras.get(camera_id, False):  # Only log if we're supposed to be active
                    print(f"Camera {camera_id} UDP receive error: {e}")
                break
                
    except Exception as e:
        print(f"Camera {camera_id} setup error: {e}")
    finally:
        try:
            sock.close()
        except:
            pass
        # Clean up
        if camera_id in camera_streams:
            del camera_streams[camera_id]

@bp.route('/list')
def list_cameras():
    """Get list of available cameras"""
    cameras = []
    
    # Add global camera
    cameras.append({
        'id': 'global',
        'name': 'Global Camera',
        'location': 'Main View',
        'ip': cfg.GLOBAL_CAMERA_IP,
        'port': cfg.GLOBAL_CAMERA_PORT,
        'active': active_cameras.get('global', False)
    })
    
    # Add robot cameras
    for robot_id, config in cfg.ROBOT_CAMERAS.items():
        cameras.append({
            'id': robot_id,
            'name': config['name'],
            'location': config['location'],
            'ip': config['ip'],
            'port': config['port'],
            'active': active_cameras.get(robot_id, False)
        })
    
    return jsonify({'cameras': cameras})

@bp.route('/stream/<camera_id>')
def video_stream(camera_id):
    """Video streaming route for specific camera"""
    if camera_id not in active_cameras or not active_cameras[camera_id]:
        return "Camera not active", 404
    
    return Response(camera_stream_generator(camera_id),
                   mimetype='multipart/x-mixed-replace; boundary=frame')

@bp.route('/start/<camera_id>', methods=['POST'])
def start_camera(camera_id):
    """Start specific camera stream"""
    if active_cameras.get(camera_id, False):
        return jsonify({'status': 'already_active', 'message': f'Camera {camera_id} is already running'})
    
    camera_config = get_camera_config(camera_id)
    if not camera_config:
        return jsonify({'status': 'error', 'message': f'Camera {camera_id} not found'})
    
    try:
        active_cameras[camera_id] = True
        camera_threads[camera_id] = threading.Thread(
            target=udp_camera_worker, 
            args=(camera_id, camera_config), 
            daemon=True
        )
        camera_threads[camera_id].start()
        return jsonify({'status': 'success', 'message': f'Camera {camera_id} started'})
    except Exception as e:
        active_cameras[camera_id] = False
        return jsonify({'status': 'error', 'message': f'Failed to start camera {camera_id}: {str(e)}'})

@bp.route('/stop/<camera_id>', methods=['POST'])  
def stop_camera(camera_id):
    """Stop specific camera stream"""
    if not active_cameras.get(camera_id, False):
        return jsonify({'status': 'already_inactive', 'message': f'Camera {camera_id} is not running'})
    
    active_cameras[camera_id] = False
    
    # Wait for thread to finish
    thread = camera_threads.get(camera_id)
    if thread:
        thread.join(timeout=2)
        del camera_threads[camera_id]
    
    # Clear the queue
    camera_queue = camera_streams.get(camera_id)
    if camera_queue:
        while not camera_queue.empty():
            try:
                camera_queue.get_nowait()
            except queue.Empty:
                break
    
    return jsonify({'status': 'success', 'message': f'Camera {camera_id} stopped'})

@bp.route('/start_all', methods=['POST'])
def start_all_cameras():
    """Start all cameras"""
    results = []
    
    # Start global camera
    result = start_camera('global')
    results.append({'camera': 'global', 'result': result.get_json()})
    
    # Start robot cameras
    for robot_id in cfg.ROBOT_CAMERAS.keys():
        result = start_camera(robot_id)
        results.append({'camera': robot_id, 'result': result.get_json()})
    
    return jsonify({'status': 'success', 'results': results})

@bp.route('/stop_all', methods=['POST'])
def stop_all_cameras():
    """Stop all cameras"""
    results = []
    
    for camera_id in list(active_cameras.keys()):
        if active_cameras[camera_id]:
            result = stop_camera(camera_id)
            results.append({'camera': camera_id, 'result': result.get_json()})
    
    return jsonify({'status': 'success', 'results': results})

@bp.route('/status')
def cameras_status():
    """Get all cameras status"""
    status = {}
    
    # Check global camera
    status['global'] = {
        'active': active_cameras.get('global', False),
        'queue_size': camera_streams['global'].qsize() if 'global' in camera_streams else 0
    }
    
    # Check robot cameras
    for robot_id in cfg.ROBOT_CAMERAS.keys():
        status[robot_id] = {
            'active': active_cameras.get(robot_id, False),
            'queue_size': camera_streams[robot_id].qsize() if robot_id in camera_streams else 0
        }
    
    return jsonify({'status': status})

@bp.route('/status/<camera_id>')
def camera_status(camera_id):
    """Get specific camera status"""
    return jsonify({
        'camera_id': camera_id,
        'active': active_cameras.get(camera_id, False),
        'queue_size': camera_streams[camera_id].qsize() if camera_id in camera_streams else 0,
        'config': get_camera_config(camera_id)
    })

@bp.route('/health')
def camera_health():
    """Get detailed health status of all cameras"""
    health_data = {}
    
    # Check global camera
    health_data['global'] = {
        'active': active_cameras.get('global', False),
        'thread_alive': 'global' in camera_threads and camera_threads['global'].is_alive(),
        'queue_size': camera_streams['global'].qsize() if 'global' in camera_streams else 0,
        'last_frame_time': time.time(),  # In real implementation, track actual last frame time
        'fps_estimate': 25 if active_cameras.get('global', False) else 0,
        'bandwidth_mbps': 2.1 if active_cameras.get('global', False) else 0,
        'config': get_camera_config('global')
    }
    
    # Check robot cameras
    for robot_id in cfg.ROBOT_CAMERAS.keys():
        health_data[robot_id] = {
            'active': active_cameras.get(robot_id, False),
            'thread_alive': robot_id in camera_threads and camera_threads[robot_id].is_alive(),
            'queue_size': camera_streams[robot_id].qsize() if robot_id in camera_streams else 0,
            'last_frame_time': time.time(),
            'fps_estimate': 25 if active_cameras.get(robot_id, False) else 0,
            'bandwidth_mbps': 1.8 if active_cameras.get(robot_id, False) else 0,
            'config': get_camera_config(robot_id)
        }
    
    return jsonify({
        'timestamp': time.time(),
        'total_active': sum(1 for status in health_data.values() if status['active']),
        'total_bandwidth': sum(status['bandwidth_mbps'] for status in health_data.values()),
        'cameras': health_data
    })

@bp.route('/analytics')
def camera_analytics():
    """Get camera analytics and usage statistics"""
    active_count = sum(1 for active in active_cameras.values() if active)
    total_cameras = len(cfg.ROBOT_CAMERAS) + 1  # +1 for global camera
    
    return jsonify({
        'total_cameras': total_cameras,
        'active_cameras': active_count,
        'utilization_percent': (active_count / total_cameras) * 100,
        'total_bandwidth_mbps': active_count * 2.0,  # Estimated 2Mbps per camera
        'uptime_hours': 24.5,  # Mock data - in real implementation track actual uptime
        'frames_processed_today': active_count * 25 * 60 * 60 * 8,  # 8 hours * 25fps
        'storage_used_gb': 12.3,  # Mock data
        'alerts_today': 0,
        'performance_score': 98.5 if active_count > 0 else 0
    })

@bp.route('/recording/start/<camera_id>', methods=['POST']) 
def start_recording(camera_id):
    """Start recording for specific camera (placeholder)"""
    if camera_id not in active_cameras or not active_cameras[camera_id]:
        return jsonify({'status': 'error', 'message': 'Camera not active'})
    
    # In real implementation, start actual recording
    return jsonify({
        'status': 'success', 
        'message': f'Recording started for {camera_id}',
        'recording_id': f'rec_{camera_id}_{int(time.time())}'
    })

@bp.route('/recording/stop/<camera_id>', methods=['POST'])
def stop_recording(camera_id):
    """Stop recording for specific camera (placeholder)"""
    return jsonify({
        'status': 'success',
        'message': f'Recording stopped for {camera_id}',
        'file_path': f'/recordings/{camera_id}_{int(time.time())}.mp4'
    })

@bp.route('/alerts')
def camera_alerts():
    """Get camera alerts and events"""
    # Mock alert data - in real implementation, maintain actual alert log
    alerts = [
        {
            'id': 1,
            'camera_id': 'hanabot_3',
            'type': 'connection_lost',
            'severity': 'warning',
            'message': 'Camera connection temporarily lost',
            'timestamp': time.time() - 300,
            'resolved': True
        },
        {
            'id': 2,
            'camera_id': 'global',
            'type': 'quality_degraded',
            'severity': 'info',
            'message': 'Video quality automatically adjusted due to bandwidth',
            'timestamp': time.time() - 1800,
            'resolved': True
        }
    ]
    
    return jsonify({
        'alerts': alerts,
        'total_today': len(alerts),
        'unresolved': len([a for a in alerts if not a['resolved']])
    })

# Legacy routes for backward compatibility
@bp.route('/stream')
def legacy_video_stream():
    """Legacy video streaming route - defaults to global camera"""
    return video_stream('global')

@bp.route('/start', methods=['POST'])
def legacy_start_stream():
    """Legacy start route - defaults to global camera"""
    return start_camera('global')

@bp.route('/stop', methods=['POST'])  
def legacy_stop_stream():
    """Legacy stop route - defaults to global camera"""
    return stop_camera('global')