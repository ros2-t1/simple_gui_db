#!/usr/bin/env python3
"""
FSM GUI 모니터링 도구
로봇의 FSM 상태를 시각적으로 모니터링하고 전이 조건을 표시
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
from datetime import datetime
import threading
import time

try:
    import tkinter as tk
    from tkinter import ttk, scrolledtext
    import matplotlib.pyplot as plt
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    import matplotlib.patches as patches
    from matplotlib.animation import FuncAnimation
    import networkx as nx
    import numpy as np
    import matplotlib.font_manager as fm
    
    # 한글 폰트 설정
    plt.rcParams['font.family'] = 'DejaVu Sans'
    # 시스템에 한글 폰트가 있다면 사용
    font_list = [f.name for f in fm.fontManager.ttflist]
    if 'Noto Sans CJK KR' in font_list:
        plt.rcParams['font.family'] = 'Noto Sans CJK KR'
    elif 'Malgun Gothic' in font_list:
        plt.rcParams['font.family'] = 'Malgun Gothic'
    elif 'AppleGothic' in font_list:
        plt.rcParams['font.family'] = 'AppleGothic'
    
    plt.rcParams['axes.unicode_minus'] = False  # 마이너스 폰트 깨짐 방지
    
except ImportError as e:
    print(f"Required packages not installed: {e}")
    print("Install with: pip install tkinter matplotlib networkx")
    sys.exit(1)

class FSMGUIMonitor(Node):
    def __init__(self, gui_callback=None):
        super().__init__('fsm_gui_monitor')
        self.gui_callback = gui_callback
        self.current_states = {}
        self.state_history = []
        
        # FSM 정의 (robot/fsm.py와 동일)
        self.fsm_states = {
            'IDLE': {'color': '#4CAF50', 'description': 'Waiting'},
            'GO_TO_ARM': {'color': '#FF9800', 'description': 'Moving to Pickup'},
            'PICK': {'color': '#F44336', 'description': 'Picking Items'},
            'GO_TO_USER': {'color': '#2196F3', 'description': 'Moving to User'},
            'WAIT_CONFIRM': {'color': '#9C27B0', 'description': 'Waiting Confirm'},
            'GO_DOCK': {'color': '#607D8B', 'description': 'Returning to Dock'}
        }
        
        # 상태 전이 조건
        self.transitions = [
            ('IDLE', 'GO_TO_ARM', 'Delivery Task'),
            ('IDLE', 'GO_TO_USER', 'Call Task'),
            ('GO_TO_ARM', 'PICK', 'Arrived at Pickup'),
            ('GO_TO_ARM', 'IDLE', 'Navigation Failed'),
            ('PICK', 'GO_TO_USER', 'Pickup Complete'),
            ('GO_TO_USER', 'WAIT_CONFIRM', 'Arrived at User'),
            ('GO_TO_USER', 'IDLE', 'Navigation Failed'),
            ('WAIT_CONFIRM', 'GO_DOCK', 'Delivery Confirmed'),
            ('WAIT_CONFIRM', 'GO_TO_ARM', 'New Task Assigned'),
            ('GO_DOCK', 'IDLE', 'Docked'),
            ('GO_DOCK', 'GO_TO_ARM', 'Task Interrupted')
        ]
        
        # ROS2 구독 설정
        self.setup_subscriptions()
        
    def setup_subscriptions(self):
        """ROS2 토픽 구독 설정"""
        robot_ids = ["robot_1", "robot_2", "robot_3"]
        
        for robot_id in robot_ids:
            self.create_subscription(
                String, f'/{robot_id}/status',
                lambda msg, rid=robot_id: self.handle_robot_status(msg, rid), 10)
        
        # Fleet 상태 구독
        self.create_subscription(
            String, '/fleet/robot_status', self.handle_fleet_status, 10)
    
    def handle_robot_status(self, msg: String, robot_id: str):
        """로봇 상태 업데이트 처리"""
        status = msg.data.upper()
        timestamp = datetime.now()
        
        # 상태 매핑
        status_map = {
            'IDLE': 'IDLE',
            'MOVING_TO_ARM': 'GO_TO_ARM', 
            'PICKING': 'PICK',
            'MOVING_TO_USER': 'GO_TO_USER',
            'WAITING_CONFIRM': 'WAIT_CONFIRM',
            'RETURNING_TO_DOCK': 'GO_DOCK'
        }
        
        mapped_status = status_map.get(status, status)
        
        if mapped_status in self.fsm_states:
            self.current_states[robot_id] = {
                'state': mapped_status,
                'timestamp': timestamp,
                'raw_status': status
            }
            
            # 히스토리 추가
            self.state_history.append({
                'robot_id': robot_id,
                'state': mapped_status,
                'timestamp': timestamp
            })
            
            # 최근 100개만 유지
            if len(self.state_history) > 100:
                self.state_history = self.state_history[-100:]
            
            # GUI 업데이트 콜백
            if self.gui_callback:
                self.gui_callback(robot_id, mapped_status, timestamp)
    
    def handle_fleet_status(self, msg: String):
        """Fleet 상태 업데이트 처리"""
        try:
            data = json.loads(msg.data)
            # Fleet 상태는 추가 정보로 활용
            pass
        except json.JSONDecodeError:
            pass


class FSMGUIApp:
    def __init__(self):
        self.root = tk.Tk()
        self.root.title("🤖 FSM Monitor - Robot State Visualization")
        self.root.geometry("1600x1000")
        self.root.configure(bg='#2b2b2b')
        
        self.fsm_node = None
        
        # 상태 전이 조건 (FSMGUIMonitor와 동일)
        self.transitions = [
            ('IDLE', 'GO_TO_ARM', 'Delivery Task'),
            ('IDLE', 'GO_TO_USER', 'Call Task'),
            ('GO_TO_ARM', 'PICK', 'Arrived at Pickup'),
            ('GO_TO_ARM', 'IDLE', 'Navigation Failed'),
            ('PICK', 'GO_TO_USER', 'Pickup Complete'),
            ('GO_TO_USER', 'WAIT_CONFIRM', 'Arrived at User'),
            ('GO_TO_USER', 'IDLE', 'Navigation Failed'),
            ('WAIT_CONFIRM', 'GO_DOCK', 'Delivery Confirmed'),
            ('WAIT_CONFIRM', 'GO_TO_ARM', 'New Task Assigned'),
            ('GO_DOCK', 'IDLE', 'Docked'),
            ('GO_DOCK', 'GO_TO_ARM', 'Task Interrupted')
        ]
        
        self.setup_ui()
        self.setup_ros()
        
    def setup_ui(self):
        """UI 구성 요소 설정"""
        # 메인 프레임
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        # 상단: 제어 패널
        control_frame = ttk.LabelFrame(main_frame, text="🎛️ Control Panel", padding=10)
        control_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.status_label = ttk.Label(control_frame, text="🔍 ROS2 연결 중...", 
                                     font=('Arial', 12, 'bold'))
        self.status_label.pack(side=tk.LEFT)
        
        # 새로고침 버튼
        refresh_btn = ttk.Button(control_frame, text="🔄 Refresh", 
                                command=self.refresh_display)
        refresh_btn.pack(side=tk.RIGHT, padx=(10, 0))
        
        # 메인 컨텐츠 - 좌우 분할
        content_frame = ttk.Frame(main_frame)
        content_frame.pack(fill=tk.BOTH, expand=True)
        
        # 좌측: FSM 다이어그램
        left_frame = ttk.LabelFrame(content_frame, text="🔄 FSM State Diagram", padding=10)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 5))
        
        # matplotlib 캔버스 (크기 확대)
        self.fig, self.ax = plt.subplots(figsize=(12, 10), facecolor='#2b2b2b')
        self.ax.set_facecolor('#2b2b2b')
        self.canvas = FigureCanvasTkAgg(self.fig, left_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # 우측: 상태 정보
        right_frame = ttk.Frame(content_frame)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, padx=(5, 0))
        
        # 현재 로봇 상태
        robot_frame = ttk.LabelFrame(right_frame, text="🤖 Robot States", padding=10)
        robot_frame.pack(fill=tk.X, pady=(0, 10))
        
        self.robot_tree = ttk.Treeview(robot_frame, height=6)
        self.robot_tree['columns'] = ('State', 'Time')
        self.robot_tree.heading('#0', text='Robot ID')
        self.robot_tree.heading('State', text='Current State')
        self.robot_tree.heading('Time', text='Last Update')
        self.robot_tree.pack(fill=tk.X)
        
        # 상태 히스토리
        history_frame = ttk.LabelFrame(right_frame, text="📋 State History", padding=10)
        history_frame.pack(fill=tk.BOTH, expand=True)
        
        self.history_text = scrolledtext.ScrolledText(history_frame, height=15, width=40,
                                                     bg='#1e1e1e', fg='#ffffff', 
                                                     font=('Consolas', 9))
        self.history_text.pack(fill=tk.BOTH, expand=True)
        
        self.setup_fsm_diagram()
        
    def setup_fsm_diagram(self):
        """FSM 다이어그램 초기 설정"""
        self.ax.clear()
        self.ax.set_facecolor('#1e1e1e')
        
        # NetworkX 그래프 생성
        self.G = nx.DiGraph()
        
        # FSM 상태들을 노드로 추가
        states = ['IDLE', 'GO_TO_ARM', 'PICK', 'GO_TO_USER', 'WAIT_CONFIRM', 'GO_DOCK']
        self.G.add_nodes_from(states)
        
        # 전이를 엣지로 추가 (조건 정보 포함)
        for src, dst, condition in self.transitions:
            self.G.add_edge(src, dst, condition=condition)
        
        # 수동으로 노드 위치 설정 (깔끔한 플로우 차트 형태)
        self.pos = {
            'IDLE': (0, 0),           # 중앙 하단 시작점
            'GO_TO_ARM': (-3, 2),     # 좌측 배달 플로우
            'PICK': (-3, 4),          # 좌측 상단
            'GO_TO_USER': (0, 6),     # 중앙 상단
            'WAIT_CONFIRM': (0, 8),   # 최상단
            'GO_DOCK': (3, 4),        # 우측 복귀 플로우
        }
        
        self.draw_fsm()
        
    def draw_fsm(self, current_robot_states=None):
        """FSM 다이어그램 그리기"""
        self.ax.clear()
        self.ax.set_facecolor('#1e1e1e')
        
        # 기본 노드 색상
        node_colors = ['#4CAF50' if state == 'IDLE' else '#607D8B' 
                      for state in self.G.nodes()]
        
        # 현재 활성 상태 하이라이트
        if current_robot_states:
            for i, state in enumerate(self.G.nodes()):
                active_robots = [rid for rid, info in current_robot_states.items() 
                               if info.get('state') == state]
                if active_robots:
                    node_colors[i] = '#FF5722'  # 활성 상태 빨간색
        
        # 노드 그리기 (크기 확대)
        nx.draw_networkx_nodes(self.G, self.pos, node_color=node_colors,
                              node_size=4000, alpha=0.9, ax=self.ax)
        
        # 엣지 그리기 (화살표 크기 확대)
        nx.draw_networkx_edges(self.G, self.pos, edge_color='#ffffff',
                              arrows=True, arrowsize=30, arrowstyle='->', 
                              alpha=0.6, width=2, ax=self.ax)
        
        # 노드 라벨 그리기 (폰트 크기 확대)
        labels = {state: state.replace('_', '\n') for state in self.G.nodes()}
        nx.draw_networkx_labels(self.G, self.pos, labels, font_size=12,
                               font_color='white', font_weight='bold', ax=self.ax)
        
        # 전이 조건 라벨 그리기
        edge_labels = nx.get_edge_attributes(self.G, 'condition')
        # 라벨을 짧게 줄여서 표시
        short_labels = {}
        for edge, condition in edge_labels.items():
            # 조건을 짧게 줄임
            if 'Delivery Task' in condition:
                short_labels[edge] = 'Delivery\nTask'
            elif 'Call Task' in condition:
                short_labels[edge] = 'Call\nTask'
            elif 'Arrived' in condition:
                short_labels[edge] = 'Arrived'
            elif 'Failed' in condition:
                short_labels[edge] = 'Failed'
            elif 'Complete' in condition:
                short_labels[edge] = 'Complete'
            elif 'Confirmed' in condition:
                short_labels[edge] = 'Confirmed'
            elif 'Assigned' in condition:
                short_labels[edge] = 'New Task'
            elif 'Interrupted' in condition:
                short_labels[edge] = 'Interrupt'
            elif 'Docked' in condition:
                short_labels[edge] = 'Docked'
            else:
                # 긴 텍스트는 첫 2단어만
                words = condition.split()
                short_labels[edge] = '\n'.join(words[:2]) if len(words) > 1 else condition[:8]
        
        # 특정 엣지에 대해 라벨 위치를 수동으로 설정하여 겹침 방지
        label_positions = {}
        
        # 각 엣지에 대해 개별적으로 위치 조정
        edge_offsets = {
            ('IDLE', 'GO_TO_ARM'): (0.2, 0.1),
            ('IDLE', 'GO_TO_USER'): (-0.2, 0.1),
            ('GO_TO_ARM', 'PICK'): (0.1, 0.0),
            ('GO_TO_ARM', 'IDLE'): (0.3, -0.1),
            ('PICK', 'GO_TO_USER'): (0.0, 0.2),
            ('GO_TO_USER', 'WAIT_CONFIRM'): (0.1, 0.0),
            ('GO_TO_USER', 'IDLE'): (-0.3, -0.3),
            ('WAIT_CONFIRM', 'GO_DOCK'): (0.2, -0.1),
            ('WAIT_CONFIRM', 'GO_TO_ARM'): (-0.2, -0.1),
            ('GO_DOCK', 'IDLE'): (0.0, -0.2),
            ('GO_DOCK', 'GO_TO_ARM'): (-0.1, 0.2),
        }
        
        for edge, label in short_labels.items():
            src, dst = edge
            src_pos = self.pos[src]
            dst_pos = self.pos[dst]
            
            # 기본 중점 위치
            mid_x = (src_pos[0] + dst_pos[0]) / 2
            mid_y = (src_pos[1] + dst_pos[1]) / 2
            
            # 개별 offset 적용
            if edge in edge_offsets:
                offset_x, offset_y = edge_offsets[edge]
                label_positions[edge] = (mid_x + offset_x, mid_y + offset_y)
            else:
                label_positions[edge] = (mid_x, mid_y)
        
        # 조정된 위치로 라벨 그리기
        for edge, label in short_labels.items():
            pos = label_positions[edge]
            self.ax.text(pos[0], pos[1], label,
                       fontsize=9, color='#FFD700', 
                       bbox=dict(boxstyle='round,pad=0.3', 
                               facecolor='black', alpha=0.8, 
                               edgecolor='none'),
                       ha='center', va='center')
        
        self.ax.set_title("🔄 Robot FSM State Diagram", color='white', 
                         fontsize=18, fontweight='bold', pad=20)
        self.ax.axis('off')
        self.canvas.draw()
    
    def setup_ros(self):
        """ROS2 노드 설정"""
        def ros_thread():
            rclpy.init()
            self.fsm_node = FSMGUIMonitor(gui_callback=self.update_display)
            
            # GUI에서 ROS 상태 업데이트
            self.root.after(100, lambda: self.status_label.configure(
                text="✅ ROS2 Connected - Monitoring..."))
            
            try:
                rclpy.spin(self.fsm_node)
            except Exception as e:
                print(f"ROS error: {e}")
            finally:
                if self.fsm_node:
                    self.fsm_node.destroy_node()
                rclpy.shutdown()
        
        self.ros_thread = threading.Thread(target=ros_thread, daemon=True)
        self.ros_thread.start()
    
    def update_display(self, robot_id, state, timestamp):
        """GUI 디스플레이 업데이트"""
        def gui_update():
            # 로봇 상태 트리 업데이트
            time_str = timestamp.strftime("%H:%M:%S")
            
            # 기존 항목 찾기 또는 새로 추가
            existing = None
            for item in self.robot_tree.get_children():
                if self.robot_tree.item(item)['text'] == robot_id:
                    existing = item
                    break
            
            if existing:
                self.robot_tree.item(existing, values=(state, time_str))
            else:
                self.robot_tree.insert('', tk.END, text=robot_id, 
                                     values=(state, time_str))
            
            # 히스토리 텍스트 업데이트
            history_entry = f"[{time_str}] {robot_id}: {state}\n"
            self.history_text.insert(tk.END, history_entry)
            self.history_text.see(tk.END)
            
            # FSM 다이어그램 업데이트
            if self.fsm_node:
                self.draw_fsm(self.fsm_node.current_states)
        
        # GUI 업데이트는 메인 스레드에서 실행
        self.root.after(0, gui_update)
    
    def refresh_display(self):
        """디스플레이 새로고침"""
        if self.fsm_node:
            self.draw_fsm(self.fsm_node.current_states)
        self.status_label.configure(text="🔄 Display Refreshed")
        self.root.after(2000, lambda: self.status_label.configure(
            text="✅ ROS2 Connected - Monitoring..."))
    
    def run(self):
        """GUI 실행"""
        def on_closing():
            if self.fsm_node:
                self.fsm_node.destroy_node()
            self.root.quit()
            
        self.root.protocol("WM_DELETE_WINDOW", on_closing)
        self.root.mainloop()


def main():
    """메인 실행 함수"""
    print("🚀 Starting FSM GUI Monitor...")
    
    try:
        app = FSMGUIApp()
        app.run()
    except KeyboardInterrupt:
        print("\n🛑 FSM GUI Monitor stopped")
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()