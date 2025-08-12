#!/usr/bin/env python3
"""
FINAL GUI VERSION: Multi-Robot Collision Avoidance Simulator with PyQt6
완전한 GUI 시뮬레이터 - 병목 해결 + 동시 이동 유지
"""

import sys
import json
import math
import time
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Set
import numpy as np
from collections import deque

try:
    from PyQt6.QtWidgets import *
    from PyQt6.QtCore import *
    from PyQt6.QtGui import *
    PYQT_AVAILABLE = True
except ImportError:
    print("⚠️ PyQt6 not installed. Installing...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "PyQt6"])
    from PyQt6.QtWidgets import *
    from PyQt6.QtCore import *
    from PyQt6.QtGui import *
    PYQT_AVAILABLE = True

class RobotState(Enum):
    IDLE = "🟢 Idle"
    MOVING = "🔵 Moving"
    YIELDING = "🟡 Yielding"
    AT_DESTINATION = "✅ Arrived"

class TaskType(Enum):
    DELIVERY = "delivery"
    CHARGING = "charging"
    ARM_PICKUP = "arm_pickup"

@dataclass
class Task:
    task_id: str
    robot_id: str
    start_pos: str
    end_pos: str
    task_type: TaskType
    priority: int = 1
    description: str = ""

@dataclass 
class Robot:
    robot_id: str
    current_pos: List[float]
    target_pos: List[float]
    state: RobotState = RobotState.IDLE
    current_task: Optional[Task] = None
    planned_path: List[str] = field(default_factory=list)
    path_index: int = 0
    speed: float = 0.05
    current_waypoint: str = ""
    yield_timer: float = 0.0
    distance_to_goal: float = float('inf')
    color: QColor = field(default_factory=lambda: QColor(0, 0, 255))
    reroute_count: int = 0

class EnhancedWaypointNetwork:
    """Enhanced network with complete connectivity"""
    
    def __init__(self):
        with open('/home/sang/dev/simple_gui_db/e_multi/waypoint_coordinates.json', 'r') as f:
            data = json.load(f)
        
        self.locations = data['existing_locations']
        self.waypoints = data['strategic_waypoints']
        self.all_positions = {**self.locations, **self.waypoints}
        
        # Reservations with priority
        self.reservations: Dict[str, Tuple[str, float]] = {}
        
        # Build enhanced connections
        self.connections = self._build_enhanced_connections()
        
    def _build_enhanced_connections(self) -> Dict[str, Set[str]]:
        """Build enhanced bidirectional connections"""
        connections = {
            # Main paths
            "south_entry": {"intersection", "south_bypass", "arm"},
            "intersection": {"south_entry", "north_hub", "west_safe", "east_extension"}, 
            "north_hub": {"intersection", "north_west", "room1", "room2"},
            
            # Charging cluster - fully connected
            "charging_hub": {"charging1", "charging2", "west_safe"},
            "charging1": {"charging_hub", "west_safe"},
            "charging2": {"charging_hub", "west_safe"},
            "west_safe": {"charging_hub", "charging1", "charging2", "intersection", "north_west"},
            "north_west": {"west_safe", "north_hub", "room1"},
            
            # Extensions with more connections
            "east_extension": {"intersection", "room2", "north_hub"},
            "south_bypass": {"south_entry", "arm", "east_extension"},
            
            # Rooms with multiple access
            "arm": {"south_entry", "south_bypass"},
            "room1": {"north_hub", "north_west"},
            "room2": {"north_hub", "east_extension"},
        }
        
        # Make bidirectional
        bidirectional = {}
        for node, neighbors in connections.items():
            if node not in bidirectional:
                bidirectional[node] = set()
            bidirectional[node].update(neighbors)
            for neighbor in neighbors:
                if neighbor not in bidirectional:
                    bidirectional[neighbor] = set()
                bidirectional[neighbor].add(node)
        
        return bidirectional
    
    def get_shortest_path(self, start: str, end: str, avoid: List[str] = None) -> Tuple[List[str], float]:
        """Find shortest path avoiding certain waypoints"""
        if avoid is None:
            avoid = []
        
        if start == end:
            return [start], 0.0
        
        queue = deque([(start, [start], 0.0)])
        visited = {start}
        
        while queue:
            current, path, dist = queue.popleft()
            
            if current == end:
                return path, dist
            
            for neighbor in self.connections.get(current, set()):
                if neighbor in visited or neighbor in avoid:
                    continue
                
                visited.add(neighbor)
                edge_dist = self._distance(current, neighbor)
                new_path = path + [neighbor]
                new_dist = dist + edge_dist
                queue.append((neighbor, new_path, new_dist))
        
        return [], float('inf')
    
    def _distance(self, wp1: str, wp2: str) -> float:
        """Calculate distance between waypoints"""
        pos1 = self.all_positions[wp1]
        pos2 = self.all_positions[wp2]
        return math.sqrt((pos1[0] - pos2[0])**2 + (pos1[1] - pos2[1])**2)
    
    def try_reserve(self, waypoint: str, robot_id: str, priority: float) -> bool:
        """Try to reserve waypoint with priority"""
        if waypoint not in self.reservations:
            self.reservations[waypoint] = (robot_id, priority)
            return True
        
        current_holder, current_priority = self.reservations[waypoint]
        
        if current_holder == robot_id:
            return True
        
        # Higher priority (lower number) wins
        if priority < current_priority - 0.1:
            self.reservations[waypoint] = (robot_id, priority)
            return True
        
        return False
    
    def release(self, waypoint: str, robot_id: str):
        """Release waypoint reservation"""
        if waypoint in self.reservations and self.reservations[waypoint][0] == robot_id:
            del self.reservations[waypoint]

class SmartFleetManager:
    """Fleet manager with smart collision avoidance"""
    
    def __init__(self, network: EnhancedWaypointNetwork):
        self.network = network
        self.robots: Dict[str, Robot] = {}
        self.pending_tasks: List[Task] = []
        self.completed_tasks: List[Task] = []
        self.log_messages: List[str] = []
        
    def add_robot(self, robot: Robot):
        self.robots[robot.robot_id] = robot
        
    def add_task(self, task: Task):
        self.pending_tasks.append(task)
        self.log(f"📋 New task: {task.description}")
        
    def log(self, message: str):
        """Add message to log"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_messages.append(f"[{timestamp}] {message}")
        # Keep only last 100 messages
        if len(self.log_messages) > 100:
            self.log_messages.pop(0)
    
    def assign_tasks(self):
        """Assign pending tasks to idle robots"""
        idle_robots = [r for r in self.robots.values() if r.state == RobotState.IDLE]
        
        while self.pending_tasks and idle_robots:
            task = self.pending_tasks.pop(0)
            robot = idle_robots.pop(0)
            
            # Find path
            path, distance = self.network.get_shortest_path(
                self._get_current_waypoint(robot),
                task.end_pos
            )
            
            if path:
                robot.current_task = task
                robot.planned_path = path
                robot.path_index = 0
                robot.state = RobotState.MOVING
                robot.distance_to_goal = distance
                robot.current_waypoint = path[0]
                robot.reroute_count = 0
                
                if len(path) > 1:
                    robot.target_pos = list(self.network.all_positions[path[1]])
                else:
                    robot.target_pos = list(self.network.all_positions[path[0]])
                
                self.log(f"🚀 {robot.robot_id}: {' → '.join(path)}")
    
    def _get_current_waypoint(self, robot: Robot) -> str:
        """Get nearest waypoint to robot position"""
        min_dist = float('inf')
        nearest = None
        
        for name, pos in self.network.all_positions.items():
            dist = math.sqrt((robot.current_pos[0] - pos[0])**2 + 
                           (robot.current_pos[1] - pos[1])**2)
            if dist < min_dist:
                min_dist = dist
                nearest = name
        
        return nearest
    
    def update_robots(self, dt: float):
        """Update all robots"""
        # Update distances for priority
        for robot in self.robots.values():
            if robot.planned_path and robot.path_index < len(robot.planned_path):
                remaining_path = robot.planned_path[robot.path_index:]
                robot.distance_to_goal = self._calculate_remaining_distance(remaining_path)
        
        # Process yielding robots
        for robot in self.robots.values():
            if robot.state == RobotState.YIELDING:
                self._handle_yielding(robot, dt)
        
        # Process moving robots
        moving_count = 0
        for robot in self.robots.values():
            if robot.state == RobotState.MOVING:
                self._update_movement(robot, dt)
                moving_count += 1
        
        if moving_count > 1:
            self.log(f"🤖🤖 {moving_count} robots moving simultaneously!")
    
    def _calculate_remaining_distance(self, path: List[str]) -> float:
        """Calculate remaining path distance"""
        if len(path) < 2:
            return 0.0
        
        total = 0.0
        for i in range(len(path) - 1):
            total += self.network._distance(path[i], path[i+1])
        return total
    
    def _update_movement(self, robot: Robot, dt: float):
        """Update robot movement"""
        current = np.array(robot.current_pos)
        target = np.array(robot.target_pos)
        
        direction = target - current
        distance = np.linalg.norm(direction)
        
        if distance < 0.01:
            self._on_waypoint_reached(robot)
        else:
            # Move towards target
            direction_normalized = direction / distance
            move_distance = robot.speed * dt
            new_pos = current + direction_normalized * min(move_distance, distance)
            robot.current_pos = list(new_pos)
            
            # Check next waypoint
            if distance < 0.05 and robot.path_index + 1 < len(robot.planned_path):
                self._try_advance(robot)
    
    def _on_waypoint_reached(self, robot: Robot):
        """Handle reaching waypoint"""
        # Release current waypoint
        if robot.current_waypoint:
            self.network.release(robot.current_waypoint, robot.robot_id)
        
        # Advance to next
        robot.path_index += 1
        
        if robot.path_index >= len(robot.planned_path):
            # Task completed!
            robot.state = RobotState.AT_DESTINATION
            if robot.current_task:
                self.completed_tasks.append(robot.current_task)
                self.log(f"✅ {robot.robot_id} completed: {robot.current_task.description}")
                robot.current_task = None
        else:
            robot.current_waypoint = robot.planned_path[robot.path_index]
            
            if robot.path_index + 1 < len(robot.planned_path):
                robot.target_pos = list(self.network.all_positions[robot.current_waypoint])
    
    def _try_advance(self, robot: Robot):
        """Try to advance to next waypoint"""
        next_wp = robot.planned_path[robot.path_index + 1]
        priority = robot.distance_to_goal
        
        if self.network.try_reserve(next_wp, robot.robot_id, priority):
            return  # Success
        
        # Blocked - find alternative
        self.log(f"⚠️ {robot.robot_id} blocked at {next_wp}")
        
        current_wp = robot.planned_path[robot.path_index]
        destination = robot.planned_path[-1]
        
        # Get waypoints to avoid
        avoid = [next_wp]
        for wp, (holder, _) in self.network.reservations.items():
            if holder != robot.robot_id:
                avoid.append(wp)
        
        alt_path, alt_dist = self.network.get_shortest_path(current_wp, destination, avoid)
        
        if alt_path and len(alt_path) > 1:
            # Reroute!
            robot.planned_path = [current_wp] + alt_path[1:]
            robot.path_index = 0
            robot.distance_to_goal = alt_dist
            robot.reroute_count += 1
            self.log(f"🔄 {robot.robot_id} rerouting (#{robot.reroute_count})")
        else:
            # Yield briefly
            robot.state = RobotState.YIELDING
            robot.yield_timer = 1.0
            self.log(f"⏸️ {robot.robot_id} yielding")
    
    def _handle_yielding(self, robot: Robot, dt: float):
        """Handle yielding robot"""
        robot.yield_timer -= dt
        
        if robot.yield_timer <= 0:
            robot.state = RobotState.MOVING
            self.log(f"▶️ {robot.robot_id} resuming")

class SimulationCanvas(QWidget):
    """Canvas for drawing the simulation"""
    
    def __init__(self):
        super().__init__()
        self.network = None
        self.fleet = None
        self.scale = 400  # pixels per meter
        self.offset_x = 200
        self.offset_y = 400
        
    def set_simulation_data(self, network: EnhancedWaypointNetwork, fleet: SmartFleetManager):
        """Set simulation references"""
        self.network = network
        self.fleet = fleet
        
    def paintEvent(self, event):
        """Paint the simulation"""
        if not self.network or not self.fleet:
            return
            
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Background
        painter.fillRect(self.rect(), QColor(240, 240, 240))
        
        # Draw connections
        self._draw_connections(painter)
        
        # Draw waypoints
        self._draw_waypoints(painter)
        
        # Draw robots
        self._draw_robots(painter)
        
        # Draw legend
        self._draw_legend(painter)
    
    def _world_to_screen(self, world_pos: List[float]) -> Tuple[int, int]:
        """Convert world coordinates to screen"""
        x = int(world_pos[0] * self.scale + self.offset_x)
        y = int(-world_pos[1] * self.scale + self.offset_y)
        return x, y
    
    def _draw_connections(self, painter: QPainter):
        """Draw waypoint connections"""
        painter.setPen(QPen(QColor(200, 200, 200), 2))
        
        drawn = set()  # Avoid drawing twice
        
        for wp1, neighbors in self.network.connections.items():
            if wp1 in self.network.all_positions:
                pos1 = self._world_to_screen(self.network.all_positions[wp1])
                
                for wp2 in neighbors:
                    if wp2 in self.network.all_positions:
                        # Avoid duplicate lines
                        edge = tuple(sorted([wp1, wp2]))
                        if edge not in drawn:
                            drawn.add(edge)
                            pos2 = self._world_to_screen(self.network.all_positions[wp2])
                            painter.drawLine(pos1[0], pos1[1], pos2[0], pos2[1])
    
    def _draw_waypoints(self, painter: QPainter):
        """Draw waypoints and locations"""
        for name, pos in self.network.all_positions.items():
            screen_pos = self._world_to_screen(pos)
            
            # Different colors for different types
            if name in self.network.locations:
                # Destinations (rooms, charging, arm)
                if "room" in name:
                    color = QColor(0, 150, 0)  # Green
                elif "charging" in name:
                    color = QColor(255, 165, 0)  # Orange
                elif name == "arm":
                    color = QColor(255, 0, 255)  # Magenta
                else:
                    color = QColor(255, 0, 0)  # Red
                size = 15
            else:
                # Waypoints
                color = QColor(100, 100, 255)  # Blue
                size = 10
            
            # Check if reserved
            if name in self.network.reservations:
                painter.setBrush(QBrush(color.lighter(150)))
                painter.setPen(QPen(color, 3))
            else:
                painter.setBrush(QBrush(color))
                painter.setPen(QPen(color.darker(150), 2))
            
            painter.drawEllipse(screen_pos[0]-size, screen_pos[1]-size, size*2, size*2)
            
            # Draw label
            painter.setPen(QPen(QColor(0, 0, 0), 1))
            painter.setFont(QFont("Arial", 8))
            painter.drawText(screen_pos[0]+size+5, screen_pos[1]+5, name)
    
    def _draw_robots(self, painter: QPainter):
        """Draw robots and their paths"""
        colors = [QColor(0, 0, 255), QColor(255, 0, 0), QColor(0, 200, 0)]
        
        for i, (robot_id, robot) in enumerate(self.fleet.robots.items()):
            color = colors[i % len(colors)]
            robot.color = color
            
            # Draw planned path
            if robot.planned_path and len(robot.planned_path) > 1:
                painter.setPen(QPen(color.lighter(150), 2, Qt.PenStyle.DashLine))
                
                for j in range(robot.path_index, len(robot.planned_path) - 1):
                    wp1 = robot.planned_path[j]
                    wp2 = robot.planned_path[j + 1]
                    
                    if wp1 in self.network.all_positions and wp2 in self.network.all_positions:
                        pos1 = self._world_to_screen(self.network.all_positions[wp1])
                        pos2 = self._world_to_screen(self.network.all_positions[wp2])
                        painter.drawLine(pos1[0], pos1[1], pos2[0], pos2[1])
            
            # Draw robot
            screen_pos = self._world_to_screen(robot.current_pos)
            
            # Robot body
            if robot.state == RobotState.YIELDING:
                painter.setBrush(QBrush(QColor(255, 255, 0)))  # Yellow when yielding
            else:
                painter.setBrush(QBrush(color))
            
            painter.setPen(QPen(color.darker(150), 2))
            painter.drawEllipse(screen_pos[0]-12, screen_pos[1]-12, 24, 24)
            
            # Robot ID
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.setFont(QFont("Arial", 10, QFont.Weight.Bold))
            painter.drawText(screen_pos[0]-10, screen_pos[1]+5, robot_id[-1])
    
    def _draw_legend(self, painter: QPainter):
        """Draw legend"""
        x, y = 10, 10
        painter.setFont(QFont("Arial", 10))
        
        # Title
        painter.setPen(QPen(QColor(0, 0, 0), 2))
        painter.drawText(x, y, "Legend:")
        y += 20
        
        # Robot states
        states = [
            ("🟢 Idle", QColor(100, 200, 100)),
            ("🔵 Moving", QColor(100, 100, 255)),
            ("🟡 Yielding", QColor(255, 255, 0)),
            ("✅ Arrived", QColor(0, 200, 0))
        ]
        
        for state_text, color in states:
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color.darker(150), 1))
            painter.drawEllipse(x, y-8, 12, 12)
            painter.setPen(QPen(QColor(0, 0, 0), 1))
            painter.drawText(x+20, y, state_text)
            y += 18

class CollisionAvoidanceGUI(QMainWindow):
    """Main GUI window"""
    
    def __init__(self):
        super().__init__()
        self.init_ui()
        self.init_simulation()
        
        # Animation timer
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_simulation)
        self.timer.start(50)  # 20 FPS
        
    def init_ui(self):
        """Initialize UI"""
        self.setWindowTitle("🤖 Multi-Robot Collision Avoidance Simulator")
        self.setGeometry(100, 100, 1200, 800)
        
        # Central widget
        central = QWidget()
        self.setCentralWidget(central)
        
        # Main layout
        layout = QHBoxLayout()
        central.setLayout(layout)
        
        # Left: Canvas
        self.canvas = SimulationCanvas()
        layout.addWidget(self.canvas, 2)
        
        # Right: Control panel
        control_panel = self.create_control_panel()
        layout.addWidget(control_panel, 1)
        
    def create_control_panel(self):
        """Create control panel"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Title
        title = QLabel("Control Panel")
        title.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        layout.addWidget(title)
        
        # Scenario buttons
        layout.addWidget(QLabel("Test Scenarios:"))
        
        btn1 = QPushButton("🚚 Simple Delivery")
        btn1.clicked.connect(self.scenario_simple_delivery)
        layout.addWidget(btn1)
        
        btn2 = QPushButton("⚡ Charging Test")
        btn2.clicked.connect(self.scenario_charging)
        layout.addWidget(btn2)
        
        btn3 = QPushButton("🔧 ARM Bottleneck")
        btn3.clicked.connect(self.scenario_arm_bottleneck)
        layout.addWidget(btn3)
        
        btn4 = QPushButton("🎯 Complex Cycle")
        btn4.clicked.connect(self.scenario_complex)
        layout.addWidget(btn4)
        
        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        layout.addWidget(line)
        
        # Reset button
        btn_reset = QPushButton("🔄 Reset Simulation")
        btn_reset.clicked.connect(self.reset_simulation)
        btn_reset.setStyleSheet("background-color: #ff6b6b; color: white;")
        layout.addWidget(btn_reset)
        
        # Robot status
        layout.addWidget(QLabel("Robot Status:"))
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(200)
        layout.addWidget(self.status_text)
        
        # Log
        layout.addWidget(QLabel("Activity Log:"))
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        
        # Statistics
        self.stats_label = QLabel("Statistics: 0 tasks completed")
        layout.addWidget(self.stats_label)
        
        return panel
    
    def init_simulation(self):
        """Initialize simulation"""
        self.network = EnhancedWaypointNetwork()
        self.fleet = SmartFleetManager(self.network)
        
        # Create robots
        robot1 = Robot("robot_1", 
                      list(self.network.locations["charging1"]),
                      list(self.network.locations["charging1"]))
        robot2 = Robot("robot_2",
                      list(self.network.locations["charging2"]),
                      list(self.network.locations["charging2"]))
        
        self.fleet.add_robot(robot1)
        self.fleet.add_robot(robot2)
        
        # Set canvas data
        self.canvas.set_simulation_data(self.network, self.fleet)
        
    def reset_simulation(self):
        """Reset simulation to initial state"""
        self.init_simulation()
        self.fleet.log("🔄 Simulation reset")
        
    def scenario_simple_delivery(self):
        """Simple delivery scenario"""
        task1 = Task("delivery_1", "", "charging1", "room1", 
                    TaskType.DELIVERY, description="Deliver to Room 1")
        task2 = Task("delivery_2", "", "charging2", "room2",
                    TaskType.DELIVERY, description="Deliver to Room 2")
        
        self.fleet.add_task(task1)
        self.fleet.add_task(task2)
        self.fleet.assign_tasks()
    
    def scenario_charging(self):
        """Charging scenario"""
        # Move robots to rooms first
        self.fleet.robots["robot_1"].current_pos = list(self.network.locations["room1"])
        self.fleet.robots["robot_1"].state = RobotState.IDLE
        self.fleet.robots["robot_2"].current_pos = list(self.network.locations["room2"])
        self.fleet.robots["robot_2"].state = RobotState.IDLE
        
        task1 = Task("charge_1", "", "room1", "charging1",
                    TaskType.CHARGING, description="Return to charging")
        task2 = Task("charge_2", "", "room2", "charging2",
                    TaskType.CHARGING, description="Return to charging")
        
        self.fleet.add_task(task1)
        self.fleet.add_task(task2)
        self.fleet.assign_tasks()
    
    def scenario_arm_bottleneck(self):
        """ARM bottleneck scenario"""
        # Both robots go to ARM
        task1 = Task("arm_1", "", "charging1", "arm",
                    TaskType.ARM_PICKUP, description="Pick from ARM")
        task2 = Task("arm_2", "", "charging2", "arm",
                    TaskType.ARM_PICKUP, description="Pick from ARM")
        
        self.fleet.add_task(task1)
        self.fleet.add_task(task2)
        self.fleet.assign_tasks()
    
    def scenario_complex(self):
        """Complex delivery cycle"""
        # Robot 1: charging1 → room1 → arm → charging1
        task1 = Task("cycle_1", "", "charging1", "room1",
                    TaskType.DELIVERY, description="Complex cycle 1")
        
        # Robot 2: charging2 → room2 → arm → charging2  
        task2 = Task("cycle_2", "", "charging2", "room2",
                    TaskType.DELIVERY, description="Complex cycle 2")
        
        self.fleet.add_task(task1)
        self.fleet.add_task(task2)
        self.fleet.assign_tasks()
    
    def update_simulation(self):
        """Update simulation state"""
        self.fleet.update_robots(0.05)  # 50ms timestep
        self.canvas.update()
        self.update_displays()
    
    def update_displays(self):
        """Update status displays"""
        # Robot status
        status = []
        for robot_id, robot in self.fleet.robots.items():
            status.append(f"{robot_id}: {robot.state.value}")
            if robot.current_task:
                status.append(f"  Task: {robot.current_task.description}")
            if robot.planned_path and robot.path_index < len(robot.planned_path):
                progress = f"{robot.path_index+1}/{len(robot.planned_path)}"
                status.append(f"  Progress: {progress}")
                if robot.reroute_count > 0:
                    status.append(f"  Reroutes: {robot.reroute_count}")
            status.append("")
        
        self.status_text.setText("\n".join(status))
        
        # Log
        log_text = "\n".join(self.fleet.log_messages[-20:])  # Last 20 messages
        self.log_text.setText(log_text)
        
        # Statistics
        completed = len(self.fleet.completed_tasks)
        pending = len(self.fleet.pending_tasks)
        self.stats_label.setText(f"Statistics: {completed} completed, {pending} pending")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = CollisionAvoidanceGUI()
    gui.show()
    sys.exit(app.exec())