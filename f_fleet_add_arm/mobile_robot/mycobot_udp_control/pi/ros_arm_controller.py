#!/usr/bin/env python3
"""
ROS2 기반 로봇암 컨트롤러
Fleet Manager와 동일한 토픽 구조로 통신
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time
import threading
from pymycobot.mycobot import MyCobot
import socket
import numpy as np

class ROSArmController(Node):
    def __init__(self):
        super().__init__('arm_controller', namespace='robot_1')  # robot_1 네임스페이스 사용
        
        # ROS2 통신 설정
        self.cmd_sub = self.create_subscription(
            String, 'arm_cmd', self.on_arm_command, 10)
        self.status_pub = self.create_publisher(
            String, 'arm_status', 10)
        
        # 로봇암 하드웨어 초기화
        self.mc = MyCobot("/dev/ttyUSB0", 1000000)
        print("로봇암 ROS2 컨트롤러 초기화 완료")
        
        # UDP 수신 설정 (ArUco 포즈 데이터용)
        self.latest_pose = None
        self.pose_lock = threading.Lock()
        
        # 제어 파라미터
        self.P_GAIN = 900
        self.STEP = 6.5
        self.SPEED = 25
        self.THRESHOLD = 0.005
        
        # 상태 관리
        self.current_status = "idle"
        self.is_picking = False
        
        # UDP 수신 스레드 시작 (ArUco 데이터 대기)
        self.start_udp_receiver()
        
        # 주기적 상태 보고
        self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("로봇암 서비스 준비 완료 - 명령 대기 중")
    
    def start_udp_receiver(self):
        """ArUco 포즈 데이터 UDP 수신 스레드"""
        def udp_receiver():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.bind(("192.168.0.161", 9999))
            
            while rclpy.ok():
                try:
                    data, _ = sock.recvfrom(4096)
                    pose_list = json.loads(data.decode())
                    if pose_list and isinstance(pose_list, list):
                        with self.pose_lock:
                            self.latest_pose = pose_list[0]
                except Exception as e:
                    self.get_logger().error(f"UDP 수신 오류: {e}")
        
        threading.Thread(target=udp_receiver, daemon=True).start()
    
    def on_arm_command(self, msg: String):
        """Fleet Manager로부터 로봇암 명령 수신"""
        try:
            cmd_data = json.loads(msg.data)
            command = cmd_data.get("command")
            
            if command == "pick" and not self.is_picking:
                item = cmd_data.get("item", "vitamin")
                self.get_logger().info(f"픽업 명령 수신: {item}")
                self.start_pick_sequence()
            elif command == "stop":
                self.stop_all_operations()
                
        except json.JSONDecodeError:
            # 단순 문자열 명령 처리
            if msg.data == "pick" and not self.is_picking:
                self.get_logger().info("픽업 명령 수신 (단순)")
                self.start_pick_sequence()
    
    def start_pick_sequence(self):
        """비동기 픽업 시퀀스 시작"""
        def pick_worker():
            self.is_picking = True
            self.current_status = "picking"
            
            try:
                success = self.perform_aruco_guided_pick()
                
                if success:
                    self.current_status = "completed"
                    self.get_logger().info("픽업 완료")
                else:
                    self.current_status = "failed"
                    self.get_logger().error("픽업 실패")
                    
            except Exception as e:
                self.current_status = "failed"
                self.get_logger().error(f"픽업 중 오류: {e}")
            finally:
                self.is_picking = False
                
        threading.Thread(target=pick_worker, daemon=True).start()
    
    def perform_aruco_guided_pick(self) -> bool:
        """ArUco 마커 기반 자동 픽업"""
        self.get_logger().info("ArUco 기반 픽업 시작")
        
        # 기존 udp_aruco_centering_controller 로직 통합
        initial_coords = self.mc.get_coords()
        base_x, base_y, base_z = initial_coords[0], initial_coords[1], initial_coords[2]
        
        ema_dx, ema_dy = 0.0, 0.0
        ema_initialized = False
        xy_aligned = False
        pick_timeout = time.time() + 20  # 20초 타임아웃
        
        while time.time() < pick_timeout and not xy_aligned:
            with self.pose_lock:
                pose = self.latest_pose
            
            if not pose:
                time.sleep(0.1)
                continue
                
            tvec = pose.get("tvec")
            if tvec is None:
                continue
                
            dx, dy, dz = tvec
            
            # EMA 필터
            if not ema_initialized:
                ema_dx, ema_dy = dx, dy
                ema_initialized = True
            else:
                ema_dx = 0.2 * dx + 0.8 * ema_dx
                ema_dy = 0.2 * dy + 0.8 * ema_dy
            
            # 정렬 확인
            if abs(ema_dx) < self.THRESHOLD and abs(ema_dy) < self.THRESHOLD:
                xy_aligned = True
                break
            
            # 로봇암 이동
            offset_x = np.clip(ema_dx * self.P_GAIN, -self.STEP, self.STEP)
            offset_y = np.clip(-ema_dy * self.P_GAIN, -self.STEP, self.STEP)
            
            current_coords = self.mc.get_coords()
            target_x = current_coords[0] + offset_x
            target_y = current_coords[1] + offset_y
            
            self.mc.send_coords([target_x, target_y, base_z, 180, 0, 0], self.SPEED)
            time.sleep(0.1)
        
        if not xy_aligned:
            self.get_logger().error("정렬 타임아웃")
            return False
        
        # 픽업 동작 수행
        return self.execute_pick_motion()
    
    def execute_pick_motion(self) -> bool:
        """실제 픽업 동작 실행"""
        try:
            # 픽업 위치로 이동
            base_coords = self.mc.get_coords()
            pick_coords = [base_coords[0], base_coords[1] + 42, base_coords[2] - 55, 180, 0, 0]
            
            self.mc.send_coords(pick_coords, self.SPEED)
            time.sleep(2)
            
            # 그리퍼 동작
            self.mc.set_gripper_value(50, 50)
            time.sleep(2)
            
            # 들어올리기
            lift_coords = pick_coords.copy()
            lift_coords[2] += 50
            self.mc.send_coords(lift_coords, self.SPEED)
            time.sleep(1)
            
            # 로봇 위에 배치
            self.mc.send_angles([90, 0, -45, -45, 0, 0], 50)
            time.sleep(2)
            self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
            time.sleep(2)
            self.mc.send_angles([0, -60, -25, 50, 0, 0], 50)
            time.sleep(2)
            self.mc.set_gripper_value(100, 50)
            time.sleep(2)
            self.mc.send_angles([0, 0, 0, 0, 0, 0], 50)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"픽업 동작 실패: {e}")
            return False
    
    def stop_all_operations(self):
        """모든 동작 중지"""
        self.current_status = "stopped"
        self.is_picking = False
        self.get_logger().info("로봇암 동작 중지")
    
    def publish_status(self):
        """Fleet Manager에 상태 보고"""
        status_msg = {
            "status": self.current_status,
            "is_picking": self.is_picking,
            "timestamp": time.time()
        }
        self.status_pub.publish(String(data=json.dumps(status_msg)))

def main():
    rclpy.init()
    arm_controller = ROSArmController()
    
    try:
        rclpy.spin(arm_controller)
    except KeyboardInterrupt:
        pass
    finally:
        arm_controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()