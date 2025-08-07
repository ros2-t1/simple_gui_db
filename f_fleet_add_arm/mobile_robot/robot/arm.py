import time
import json
from std_msgs.msg import String

# ROS2 퍼블리셔는 FSM에서 전달받음
_arm_cmd_pub = None
_arm_status_sub = None
_arm_status = "idle"

def init_arm_communication(node):
    """FSM에서 로봇암 통신 초기화"""
    global _arm_cmd_pub, _arm_status_sub
    
    _arm_cmd_pub = node.create_publisher(String, 'arm_cmd', 10)
    _arm_status_sub = node.create_subscription(String, 'arm_status', _on_arm_status, 10)
    print("[ARM] ROS2 통신 초기화 완료")

def _on_arm_status(msg: String):
    """로봇암 상태 수신"""
    global _arm_status
    try:
        status_data = json.loads(msg.data)
        _arm_status = status_data.get("status", "idle")
    except:
        _arm_status = msg.data

def arm_pick(item: str) -> bool:
    """로봇암으로 아이템 픽업 (ROS2 토픽 사용)"""
    global _arm_status
    
    if not _arm_cmd_pub:
        print("[ARM] ROS2 통신이 초기화되지 않음")
        return False
    
    print(f"[ARM] pick {item} 명령 전송")
    
    # 픽업 명령 전송
    cmd_msg = json.dumps({"command": "pick", "item": item})
    _arm_cmd_pub.publish(String(data=cmd_msg))
    _arm_status = "picking"
    
    # 완료 대기 (최대 30초)
    timeout = time.time() + 30
    while time.time() < timeout:
        if _arm_status == "completed":
            print(f"[ARM] {item} 픽업 완료")
            return True
        elif _arm_status == "failed":
            print(f"[ARM] {item} 픽업 실패")
            return False
        time.sleep(0.5)
    
    print("[ARM] 픽업 타임아웃")
    return False

def arm_place_on_robot() -> bool:
    """로봇 위에 아이템 배치 (픽업에 포함됨)"""
    print("[ARM] place on robot 완료")
    return True

def get_arm_status() -> str:
    """현재 로봇암 상태 반환"""
    return _arm_status
