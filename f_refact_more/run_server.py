# run_server.py
import threading, rclpy
from rclpy.executors import MultiThreadedExecutor
from robot.nav2_waypoint_class import WaypointNavigator
from robot.fsm import DeliveryFSM
import robot               # ros_node 주입용
from web import create_app
import config as cfg

def main():
    # ---------- ROS 노드 준비 ----------
    rclpy.init()
    wp_nav = WaypointNavigator()          # 한 대만 쓰므로 기본 이름/네임스페이스
    fsm    = DeliveryFSM("robot", wp_nav) # robot 이름만 넣어둠

    # 주문·확인 라우트에서 쓰도록 주입
    robot.ros_node = fsm                 

    exec = MultiThreadedExecutor()
    exec.add_node(wp_nav)
    exec.add_node(fsm)

    threading.Thread(target=exec.spin, daemon=True).start()

    # ---------- Flask 앱 ----------
    app = create_app()
    try:
        # use_reloader=False → 로더가 두 번 실행되는 현상 방지
        app.run(host="0.0.0.0", port=8080, debug=True, use_reloader=False)
    finally:
        rclpy.shutdown()

if __name__ == "__main__":
    main()
