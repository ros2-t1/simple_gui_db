#!/usr/bin/env python3
"""
Dense Waypoint Network GUI - 조밀한 노드로 유동적 경로 계획
"""

import sys
import json
import math
import time
import random
from enum import Enum
from dataclasses import dataclass, field
from typing import List, Dict, Optional, Tuple, Set
import numpy as np
from collections import deque
import heapq

try:
    from PyQt6.QtWidgets import *
    from PyQt6.QtCore import *
    from PyQt6.QtGui import *
except ImportError:
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "PyQt6"])
    from PyQt6.QtWidgets import *
    from PyQt6.QtCore import *
    from PyQt6.QtGui import *

class RobotState(Enum):
    IDLE = "🟢 Idle"
    MOVING = "🔵 Moving"
    PLANNING = "🟡 Planning"
    ARRIVED = "✅ Arrived"

@dataclass
class Task:
    task_id: str
    start: str
    goal: str
    priority: float = 1.0
    description: str = ""

@dataclass
class Robot:
    robot_id: str
    position: np.ndarray  # Current position [x, y]
    velocity: np.ndarray = field(default_factory=lambda: np.array([0.0, 0.0]))
    state: RobotState = RobotState.IDLE
    current_task: Optional[Task] = None
    task_queue: List[Task] = field(default_factory=list)
    planned_path: List[Tuple[int, int]] = field(default_factory=list)
    path_index: int = 0
    speed: float = 0.08
    radius: float = 0.03  # Robot size for collision
    color: QColor = field(default_factory=lambda: QColor(0, 100, 255))
    stuck_counter: int = 0
    last_replan_time: float = 0

class DenseWaypointNetwork:
    """Dense grid-based waypoint network for flexible path planning"""
    
    def __init__(self, grid_size: float = 0.1, map_bounds: Tuple[float, float, float, float] = None):
        """
        grid_size: Distance between waypoints in meters
        map_bounds: (min_x, min_y, max_x, max_y)
        """
        self.grid_size = grid_size
        
        # Load existing locations
        with open('/home/sang/dev/simple_gui_db/e_multi/waypoint_coordinates.json', 'r') as f:
            data = json.load(f)
        
        self.locations = data['existing_locations']
        self.map_info = data['map_info']
        
        # Determine map bounds
        if map_bounds is None:
            # Calculate from existing locations
            all_pos = list(self.locations.values())
            xs = [p[0] for p in all_pos]
            ys = [p[1] for p in all_pos]
            self.bounds = (min(xs) - 0.5, min(ys) - 0.5, max(xs) + 0.5, max(ys) + 0.5)
        else:
            self.bounds = map_bounds
        
        # Generate dense waypoint grid
        self.waypoints = {}  # (i, j) -> [x, y]
        self.connections = {}  # (i, j) -> set of connected (i, j)
        self._generate_grid()
        
        # Dynamic reservations
        self.reservations = {}  # (i, j) -> (robot_id, timestamp, priority)
        self.reservation_timeout = 5.0
        
    def _generate_grid(self):
        """Generate a dense grid of waypoints"""
        min_x, min_y, max_x, max_y = self.bounds
        
        # Calculate grid dimensions
        self.grid_width = int((max_x - min_x) / self.grid_size) + 1
        self.grid_height = int((max_y - min_y) / self.grid_size) + 1
        
        print(f"📊 Creating {self.grid_width}x{self.grid_height} waypoint grid")
        
        # Create waypoints
        for i in range(self.grid_width):
            for j in range(self.grid_height):
                x = min_x + i * self.grid_size
                y = min_y + j * self.grid_size
                self.waypoints[(i, j)] = [x, y]
        
        # Create connections (8-connected grid)
        for i in range(self.grid_width):
            for j in range(self.grid_height):
                self.connections[(i, j)] = set()
                
                # Check all 8 neighbors
                for di in [-1, 0, 1]:
                    for dj in [-1, 0, 1]:
                        if di == 0 and dj == 0:
                            continue
                        
                        ni, nj = i + di, j + dj
                        if 0 <= ni < self.grid_width and 0 <= nj < self.grid_height:
                            self.connections[(i, j)].add((ni, nj))
        
        print(f"✅ Created {len(self.waypoints)} waypoints with {sum(len(c) for c in self.connections.values())} connections")
    
    def world_to_grid(self, pos: List[float]) -> Tuple[int, int]:
        """Convert world position to grid coordinates"""
        min_x, min_y, _, _ = self.bounds
        i = int((pos[0] - min_x) / self.grid_size)
        j = int((pos[1] - min_y) / self.grid_size)
        i = max(0, min(i, self.grid_width - 1))
        j = max(0, min(j, self.grid_height - 1))
        return (i, j)
    
    def grid_to_world(self, grid_pos: Tuple[int, int]) -> List[float]:
        """Convert grid coordinates to world position"""
        return self.waypoints.get(grid_pos, [0, 0])
    
    def find_path_astar(self, start: Tuple[int, int], goal: Tuple[int, int],
                       avoid_robots: Dict[str, np.ndarray] = None,
                       robot_id: str = None) -> List[Tuple[int, int]]:
        """
        A* pathfinding with dynamic obstacle avoidance
        """
        if start == goal:
            return [start]
        
        # Check which waypoints to avoid
        avoided_waypoints = set()
        current_time = time.time()
        
        # Add reserved waypoints
        for wp, (holder, timestamp, _) in self.reservations.items():
            if holder != robot_id and current_time - timestamp < self.reservation_timeout:
                avoided_waypoints.add(wp)
        
        # Add waypoints near other robots
        if avoid_robots:
            for other_id, other_pos in avoid_robots.items():
                if other_id != robot_id:
                    other_grid = self.world_to_grid(other_pos)
                    # Avoid the cell and adjacent cells
                    for di in [-1, 0, 1]:
                        for dj in [-1, 0, 1]:
                            avoided_waypoints.add((other_grid[0] + di, other_grid[1] + dj))
        
        # A* algorithm
        open_set = []
        heapq.heappush(open_set, (0, start, [start]))
        closed_set = set()
        g_score = {start: 0}
        
        while open_set:
            _, current, path = heapq.heappop(open_set)
            
            if current == goal:
                return path
            
            if current in closed_set:
                continue
            
            closed_set.add(current)
            
            # Check neighbors
            for neighbor in self.connections.get(current, set()):
                if neighbor in closed_set or neighbor in avoided_waypoints:
                    continue
                
                # Calculate g score
                tentative_g = g_score[current] + self._grid_distance(current, neighbor)
                
                if neighbor not in g_score or tentative_g < g_score[neighbor]:
                    g_score[neighbor] = tentative_g
                    
                    # Calculate f score with heuristic
                    h = self._grid_distance(neighbor, goal)
                    f = tentative_g + h
                    
                    new_path = path + [neighbor]
                    heapq.heappush(open_set, (f, neighbor, new_path))
        
        # No path found - try without some constraints
        if avoided_waypoints:
            # Retry with only critical obstacles
            critical_avoided = set()
            for wp, (holder, _, _) in self.reservations.items():
                if holder != robot_id:
                    critical_avoided.add(wp)
            
            # Simplified A* with fewer constraints
            return self._find_emergency_path(start, goal, critical_avoided)
        
        return []
    
    def _find_emergency_path(self, start: Tuple[int, int], goal: Tuple[int, int],
                            avoided: Set[Tuple[int, int]]) -> List[Tuple[int, int]]:
        """Emergency pathfinding with minimal constraints"""
        queue = deque([(start, [start])])
        visited = {start}
        
        while queue:
            current, path = queue.popleft()
            
            if current == goal:
                return path
            
            for neighbor in self.connections.get(current, set()):
                if neighbor not in visited and neighbor not in avoided:
                    visited.add(neighbor)
                    queue.append((neighbor, path + [neighbor]))
        
        return []
    
    def _grid_distance(self, a: Tuple[int, int], b: Tuple[int, int]) -> float:
        """Manhattan distance between grid positions"""
        return abs(a[0] - b[0]) + abs(a[1] - b[1])
    
    def reserve_waypoint(self, waypoint: Tuple[int, int], robot_id: str, priority: float = 1.0) -> bool:
        """Try to reserve a waypoint"""
        current_time = time.time()
        
        if waypoint not in self.reservations:
            self.reservations[waypoint] = (robot_id, current_time, priority)
            return True
        
        holder, timestamp, holder_priority = self.reservations[waypoint]
        
        if holder == robot_id:
            # Refresh reservation
            self.reservations[waypoint] = (robot_id, current_time, priority)
            return True
        
        # Check timeout
        if current_time - timestamp > self.reservation_timeout:
            self.reservations[waypoint] = (robot_id, current_time, priority)
            return True
        
        # Priority override
        if priority < holder_priority * 0.8:
            self.reservations[waypoint] = (robot_id, current_time, priority)
            return True
        
        return False
    
    def release_waypoint(self, waypoint: Tuple[int, int], robot_id: str):
        """Release a waypoint reservation"""
        if waypoint in self.reservations:
            holder, _, _ = self.reservations[waypoint]
            if holder == robot_id:
                del self.reservations[waypoint]

class FleetController:
    """Fleet controller for multiple robots with dense waypoints"""
    
    def __init__(self, network: DenseWaypointNetwork):
        self.network = network
        self.robots = {}
        self.pending_tasks = []
        self.completed_tasks = []
        self.log_messages = []
        
    def add_robot(self, robot_id: str, start_location: str):
        """Add a robot to the fleet"""
        if start_location in self.network.locations:
            pos = np.array(self.network.locations[start_location])
        else:
            pos = np.array([0.0, 0.0])
        
        robot = Robot(robot_id, pos)
        robot.color = QColor(random.randint(50, 255), random.randint(50, 255), random.randint(50, 255))
        self.robots[robot_id] = robot
        self.log(f"🤖 Added {robot_id} at {start_location}")
    
    def add_task(self, task: Task):
        """Add a task to the queue"""
        self.pending_tasks.append(task)
        self.log(f"📋 New task: {task.description}")
    
    def log(self, message: str):
        """Add log message"""
        timestamp = time.strftime("%H:%M:%S")
        self.log_messages.append(f"[{timestamp}] {message}")
        if len(self.log_messages) > 100:
            self.log_messages.pop(0)
    
    def update(self, dt: float):
        """Update all robots"""
        # Assign tasks
        self._assign_tasks()
        
        # Get robot positions for collision avoidance
        robot_positions = {rid: r.position for rid, r in self.robots.items()}
        
        # Update each robot
        for robot_id, robot in self.robots.items():
            if robot.state == RobotState.MOVING:
                self._update_robot(robot, dt, robot_positions)
            elif robot.state == RobotState.PLANNING:
                self._replan_robot(robot, robot_positions)
    
    def _assign_tasks(self):
        """Assign pending tasks to idle robots"""
        # First, assign tasks from individual robot queues
        for robot in self.robots.values():
            if robot.state == RobotState.IDLE and robot.task_queue:
                task = robot.task_queue.pop(0)
                
                # Plan path
                start_grid = self.network.world_to_grid(robot.position)
                
                if task.goal in self.network.locations:
                    goal_pos = self.network.locations[task.goal]
                    goal_grid = self.network.world_to_grid(goal_pos)
                else:
                    self.log(f"❌ {robot.robot_id}: Unknown location in task queue: {task.goal}")
                    continue
                
                robot_positions = {rid: r.position for rid, r in self.robots.items()}
                path = self.network.find_path_astar(start_grid, goal_grid, robot_positions, robot.robot_id)
                
                if path:
                    robot.current_task = task
                    robot.planned_path = path
                    robot.path_index = 0
                    robot.state = RobotState.MOVING
                    robot.stuck_counter = 0
                    self.log(f"🚀 {robot.robot_id}: Starting queued task to {task.goal}")
                else:
                    self.log(f"❌ {robot.robot_id}: No path for queued task to {task.goal}")
                    # Put task back at the front of the queue to retry
                    robot.task_queue.insert(0, task)

        # Then, assign tasks from the global pending list
        idle_robots = [r for r in self.robots.values() if r.state == RobotState.IDLE and not r.current_task]
        
        while self.pending_tasks and idle_robots:
            task = self.pending_tasks.pop(0)
            robot = idle_robots.pop(0)
            
            # Plan path
            start_grid = self.network.world_to_grid(robot.position)
            
            if task.goal in self.network.locations:
                goal_pos = self.network.locations[task.goal]
                goal_grid = self.network.world_to_grid(goal_pos)
            else:
                self.log(f"❌ Unknown location: {task.goal}")
                continue
            
            robot_positions = {rid: r.position for rid, r in self.robots.items()}
            path = self.network.find_path_astar(start_grid, goal_grid, robot_positions, robot.robot_id)
            
            if path:
                robot.current_task = task
                robot.planned_path = path
                robot.path_index = 0
                robot.state = RobotState.MOVING
                robot.stuck_counter = 0
                self.log(f"🚀 {robot.robot_id}: Path with {len(path)} waypoints to {task.goal}")
            else:
                self.log(f"❌ {robot.robot_id}: No path to {task.goal}")
                # Put task back in global queue
                self.pending_tasks.insert(0, task)
    
    def _update_robot(self, robot: Robot, dt: float, robot_positions: Dict):
        """Update robot movement"""
        if not robot.planned_path or robot.path_index >= len(robot.planned_path):
            # Reached destination
            if robot.current_task:
                robot.state = RobotState.ARRIVED
                self.completed_tasks.append(robot.current_task)
                self.log(f"✅ {robot.robot_id} completed: {robot.current_task.description}")
                robot.current_task = None
            else:
                robot.state = RobotState.IDLE
            return
        
        # Get target waypoint
        target_grid = robot.planned_path[robot.path_index]
        target_pos = np.array(self.network.grid_to_world(target_grid))
        
        # Move towards target
        direction = target_pos - robot.position
        distance = np.linalg.norm(direction)
        
        if distance < 0.05:  # Close enough to waypoint
            # Try to reserve next waypoint
            if robot.path_index + 1 < len(robot.planned_path):
                next_grid = robot.planned_path[robot.path_index + 1]
                priority = 1.0 - (robot.path_index / len(robot.planned_path))
                
                if self.network.reserve_waypoint(next_grid, robot.robot_id, priority):
                    # Release current waypoint
                    self.network.release_waypoint(target_grid, robot.robot_id)
                    robot.path_index += 1
                    robot.stuck_counter = 0
                else:
                    # Blocked - increment stuck counter
                    robot.stuck_counter += 1
                    
                    if robot.stuck_counter > 20:  # Stuck for too long
                        robot.state = RobotState.PLANNING
                        robot.last_replan_time = time.time()
                        self.log(f"🔄 {robot.robot_id} replanning due to blockage")
            else:
                # Move to next waypoint
                robot.path_index += 1
        else:
            # Check for nearby robots
            too_close = False
            for other_id, other_pos in robot_positions.items():
                if other_id != robot.robot_id:
                    dist = np.linalg.norm(robot.position - other_pos)
                    if dist < robot.radius * 3:  # Safety distance
                        too_close = True
                        break
            
            if not too_close:
                # Move towards target
                direction_normalized = direction / (distance + 0.001)
                robot.velocity = direction_normalized * robot.speed
                robot.position += robot.velocity * dt
                robot.stuck_counter = 0
            else:
                # Too close to another robot
                robot.stuck_counter += 1
                
                if robot.stuck_counter > 30:
                    robot.state = RobotState.PLANNING
                    self.log(f"🔄 {robot.robot_id} replanning due to proximity")
    
    def _replan_robot(self, robot: Robot, robot_positions: Dict):
        """Replan robot path"""
        if not robot.current_task:
            robot.state = RobotState.IDLE
            return
        
        # Don't replan too frequently
        if time.time() - robot.last_replan_time < 1.0:
            return
        
        # Find new path
        start_grid = self.network.world_to_grid(robot.position)
        goal_pos = self.network.locations[robot.current_task.goal]
        goal_grid = self.network.world_to_grid(goal_pos)
        
        path = self.network.find_path_astar(start_grid, goal_grid, robot_positions, robot.robot_id)
        
        if path:
            robot.planned_path = path
            robot.path_index = 0
            robot.state = RobotState.MOVING
            robot.stuck_counter = 0
            self.log(f"✅ {robot.robot_id} found new path with {len(path)} waypoints")
        else:
            # Still no path - wait
            robot.stuck_counter += 1
            
            if robot.stuck_counter > 100:
                # Give up on task
                self.pending_tasks.append(robot.current_task)
                robot.current_task = None
                robot.state = RobotState.IDLE
                self.log(f"❌ {robot.robot_id} gave up on task")

class SimulationCanvas(QWidget):
    """Canvas for visualizing the simulation"""
    
    def __init__(self):
        super().__init__()
        self.network = None
        self.controller = None
        self.scale = 500  # pixels per meter
        self.offset_x = 250
        self.offset_y = 400
        
        # Display options
        self.show_grid = True
        self.show_paths = True
        self.show_reservations = True
        
    def set_data(self, network: DenseWaypointNetwork, controller: FleetController):
        """Set simulation data"""
        self.network = network
        self.controller = controller
    
    def paintEvent(self, event):
        """Paint the simulation"""
        if not self.network or not self.controller:
            return
        
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        
        # Background
        painter.fillRect(self.rect(), QColor(245, 245, 245))
        
        # Draw layers
        if self.show_grid:
            self._draw_grid(painter)
        
        if self.show_reservations:
            self._draw_reservations(painter)
        
        self._draw_locations(painter)
        
        if self.show_paths:
            self._draw_paths(painter)
        
        self._draw_robots(painter)
        
        self._draw_info(painter)
    
    def _world_to_screen(self, world_pos: List[float]) -> Tuple[int, int]:
        """Convert world to screen coordinates"""
        x = int(world_pos[0] * self.scale + self.offset_x)
        y = int(-world_pos[1] * self.scale + self.offset_y)
        return (x, y)
    
    def _draw_grid(self, painter: QPainter):
        """Draw waypoint grid"""
        # Draw connections as thin lines
        painter.setPen(QPen(QColor(220, 220, 220), 1))
        
        for (i, j), neighbors in self.network.connections.items():
            pos1 = self._world_to_screen(self.network.waypoints[(i, j)])
            
            for (ni, nj) in neighbors:
                if (ni, nj) in self.network.waypoints:
                    pos2 = self._world_to_screen(self.network.waypoints[(ni, nj)])
                    painter.drawLine(pos1[0], pos1[1], pos2[0], pos2[1])
        
        # Draw waypoints as small dots
        painter.setPen(QPen(QColor(200, 200, 200), 1))
        painter.setBrush(QBrush(QColor(240, 240, 240)))
        
        for (i, j), pos in self.network.waypoints.items():
            screen_pos = self._world_to_screen(pos)
            painter.drawEllipse(screen_pos[0] - 2, screen_pos[1] - 2, 4, 4)
    
    def _draw_reservations(self, painter: QPainter):
        """Draw reserved waypoints"""
        current_time = time.time()
        
        for wp, (robot_id, timestamp, _) in self.network.reservations.items():
            if current_time - timestamp < self.network.reservation_timeout:
                pos = self._world_to_screen(self.network.waypoints[wp])
                
                # Get robot color
                if robot_id in self.controller.robots:
                    color = self.controller.robots[robot_id].color
                    color.setAlpha(100)
                    painter.setBrush(QBrush(color))
                    painter.setPen(QPen(color.darker(150), 1))
                    painter.drawEllipse(pos[0] - 5, pos[1] - 5, 10, 10)
    
    def _draw_locations(self, painter: QPainter):
        """Draw named locations"""
        for name, pos in self.network.locations.items():
            screen_pos = self._world_to_screen(pos)
            
            # Choose color based on type
            if "room" in name:
                color = QColor(0, 150, 0)
            elif "charging" in name:
                color = QColor(255, 165, 0)
            elif name == "arm":
                color = QColor(255, 0, 255)
            else:
                color = QColor(100, 100, 100)
            
            # Draw location
            painter.setBrush(QBrush(color))
            painter.setPen(QPen(color.darker(150), 2))
            painter.drawEllipse(screen_pos[0] - 12, screen_pos[1] - 12, 24, 24)
            
            # Draw label
            painter.setPen(QPen(QColor(0, 0, 0), 1))
            painter.setFont(QFont("Arial", 9))
            painter.drawText(screen_pos[0] + 15, screen_pos[1] + 5, name)
    
    def _draw_paths(self, painter: QPainter):
        """Draw robot paths"""
        for robot in self.controller.robots.values():
            if robot.planned_path and robot.path_index < len(robot.planned_path):
                # Draw remaining path
                color = robot.color
                color.setAlpha(150)
                painter.setPen(QPen(color, 2, Qt.PenStyle.DashLine))
                
                # Start from current position
                points = [QPointF(*self._world_to_screen(robot.position))]
                
                # Add remaining waypoints
                for i in range(robot.path_index, len(robot.planned_path)):
                    wp = robot.planned_path[i]
                    pos = self.network.grid_to_world(wp)
                    points.append(QPointF(*self._world_to_screen(pos)))
                
                # Draw path
                if len(points) > 1:
                    path = QPainterPath()
                    path.moveTo(points[0])
                    for point in points[1:]:
                        path.lineTo(point)
                    painter.drawPath(path)
    
    def _draw_robots(self, painter: QPainter):
        """Draw robots"""
        for robot_id, robot in self.controller.robots.items():
            screen_pos = self._world_to_screen(robot.position)
            
            # Robot body
            painter.setBrush(QBrush(robot.color))
            painter.setPen(QPen(robot.color.darker(150), 2))
            painter.drawEllipse(screen_pos[0] - 15, screen_pos[1] - 15, 30, 30)
            
            # Robot ID
            painter.setPen(QPen(QColor(255, 255, 255), 2))
            painter.setFont(QFont("Arial", 11, QFont.Weight.Bold))
            painter.drawText(screen_pos[0] - 8, screen_pos[1] + 5, robot_id[-1])
            
            # State indicator
            if robot.state == RobotState.PLANNING:
                painter.setPen(QPen(QColor(255, 255, 0), 3))
                painter.drawEllipse(screen_pos[0] - 18, screen_pos[1] - 18, 36, 36)
    
    def _draw_info(self, painter: QPainter):
        """Draw information panel"""
        x, y = 10, 10
        painter.setFont(QFont("Arial", 10))
        painter.setPen(QPen(QColor(0, 0, 0), 1))
        
        # Grid info
        painter.drawText(x, y, f"Grid: {self.network.grid_width}x{self.network.grid_height}")
        y += 20
        painter.drawText(x, y, f"Waypoints: {len(self.network.waypoints)}")
        y += 20
        painter.drawText(x, y, f"Reservations: {len(self.network.reservations)}")
        y += 30
        
        # Robot states
        painter.drawText(x, y, "Robots:")
        y += 20
        
        for robot in self.controller.robots.values():
            painter.setBrush(QBrush(robot.color))
            painter.drawEllipse(x, y - 8, 12, 12)
            painter.drawText(x + 20, y, f"{robot.robot_id}: {robot.state.value}")
            y += 18

class DenseWaypointGUI(QMainWindow):
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
        self.setWindowTitle("🤖 Dense Waypoint Network - Flexible Path Planning")
        self.setGeometry(100, 100, 1400, 900)
        
        central = QWidget()
        self.setCentralWidget(central)
        
        layout = QHBoxLayout()
        central.setLayout(layout)
        
        # Canvas
        self.canvas = SimulationCanvas()
        layout.addWidget(self.canvas, 3)
        
        # Control panel
        control = self.create_control_panel()
        layout.addWidget(control, 1)
    
    def create_control_panel(self):
        """Create control panel"""
        panel = QWidget()
        layout = QVBoxLayout()
        panel.setLayout(layout)
        
        # Title
        title = QLabel("Dense Waypoint Control")
        title.setFont(QFont("Arial", 14, QFont.Weight.Bold))
        layout.addWidget(title)
        
        # Grid density control
        layout.addWidget(QLabel("Grid Density:"))
        self.density_slider = QSlider(Qt.Orientation.Horizontal)
        self.density_slider.setRange(5, 20)  # 0.05m to 0.20m
        self.density_slider.setValue(10)  # 0.10m default
        self.density_slider.valueChanged.connect(self.update_grid_density)
        layout.addWidget(self.density_slider)
        
        self.density_label = QLabel("Grid size: 0.10m")
        layout.addWidget(self.density_label)
        
        # Display options
        layout.addWidget(QLabel("Display Options:"))
        
        self.show_grid_cb = QCheckBox("Show Grid")
        self.show_grid_cb.setChecked(True)
        self.show_grid_cb.stateChanged.connect(self.update_display)
        layout.addWidget(self.show_grid_cb)
        
        self.show_paths_cb = QCheckBox("Show Paths")
        self.show_paths_cb.setChecked(True)
        self.show_paths_cb.stateChanged.connect(self.update_display)
        layout.addWidget(self.show_paths_cb)
        
        self.show_reservations_cb = QCheckBox("Show Reservations")
        self.show_reservations_cb.setChecked(True)
        self.show_reservations_cb.stateChanged.connect(self.update_display)
        layout.addWidget(self.show_reservations_cb)
        
        # Separator
        line = QFrame()
        line.setFrameShape(QFrame.Shape.HLine)
        layout.addWidget(line)
        
        # Scenarios
        layout.addWidget(QLabel("Test Scenarios:"))
        
        btn1 = QPushButton("📦 2 Robots: Simple Delivery")
        btn1.clicked.connect(self.scenario_simple)
        layout.addWidget(btn1)
        
        btn2 = QPushButton("🔀 2 Robots: Cross Paths")
        btn2.clicked.connect(self.scenario_cross)
        layout.addWidget(btn2)
        
        btn3 = QPushButton("🎯 2 Robots: Bottleneck")
        btn3.clicked.connect(self.scenario_bottleneck)
        layout.addWidget(btn3)
        
        btn4 = QPushButton("🌟 2 Robots: Complex")
        btn4.clicked.connect(self.scenario_complex)
        layout.addWidget(btn4)
        
        # Reset
        btn_reset = QPushButton("🔄 Reset")
        btn_reset.clicked.connect(self.reset_simulation)
        btn_reset.setStyleSheet("background-color: #ff6b6b; color: white;")
        layout.addWidget(btn_reset)
        
        # Status
        layout.addWidget(QLabel("Status:"))
        self.status_text = QTextEdit()
        self.status_text.setReadOnly(True)
        self.status_text.setMaximumHeight(150)
        layout.addWidget(self.status_text)
        
        # Log
        layout.addWidget(QLabel("Activity Log:"))
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        layout.addWidget(self.log_text)
        
        return panel
    
    def init_simulation(self):
        """Initialize simulation"""
        grid_size = self.density_slider.value() / 100.0
        self.network = DenseWaypointNetwork(grid_size=grid_size)
        self.controller = FleetController(self.network)
        
        # Add initial robots
        self.controller.add_robot("R1", "charging1")
        self.controller.add_robot("R2", "charging2")
        
        # Set canvas data
        self.canvas.set_data(self.network, self.controller)
    
    def update_grid_density(self):
        """Update grid density"""
        grid_size = self.density_slider.value() / 100.0
        self.density_label.setText(f"Grid size: {grid_size:.2f}m")
        
        # Recreate network with new density
        self.network = DenseWaypointNetwork(grid_size=grid_size)
        self.controller.network = self.network
        self.canvas.set_data(self.network, self.controller)
    
    def update_display(self):
        """Update display options"""
        self.canvas.show_grid = self.show_grid_cb.isChecked()
        self.canvas.show_paths = self.show_paths_cb.isChecked()
        self.canvas.show_reservations = self.show_reservations_cb.isChecked()
        self.canvas.update()
    
    def reset_simulation(self):
        """Reset simulation"""
        self.init_simulation()
        self.controller.log("🔄 Simulation reset")
    
    def scenario_simple(self):
        """Simple delivery scenario"""
        task1 = Task("T1", "charging1", "room1", description="R1 → Room1")
        task2 = Task("T2", "charging2", "room2", description="R2 → Room2")
        
        self.controller.add_task(task1)
        self.controller.add_task(task2)
    
    def scenario_cross(self):
        """Cross paths scenario"""
        task1 = Task("T1", "charging1", "room2", description="R1 → Room2")
        task2 = Task("T2", "charging2", "room1", description="R2 → Room1")
        
        self.controller.add_task(task1)
        self.controller.add_task(task2)
    
    def scenario_bottleneck(self):
        """Bottleneck scenario with a sequence of tasks for 2 robots"""
        self.reset_simulation()
        self.controller.log("🔄 Reset & Starting Bottleneck Sequence")

        if "R1" in self.controller.robots:
            r1 = self.controller.robots["R1"]
            r1.task_queue = [
                Task("T1a", "charging1", "arm", description="R1 → ARM"),
                Task("T1b", "arm", "room1", description="R1 → Room 1"),
                Task("T1c", "room1", "charging1", description="R1 → Charging 1")
            ]
            self.controller.log("📝 Queued task sequence for R1")

        if "R2" in self.controller.robots:
            r2 = self.controller.robots["R2"]
            r2.task_queue = [
                Task("T2a", "charging2", "arm", description="R2 → ARM"),
                Task("T2b", "arm", "room2", description="R2 → Room 2"),
                Task("T2c", "room2", "charging2", description="R2 → Charging 2")
            ]
            self.controller.log("📝 Queued task sequence for R2")
    
    def scenario_complex(self):
        """Complex scenario with 2 robots"""
        # Create complex task pattern
        tasks = [
            Task("T1", "charging1", "room2", description="R1 → Room2"),
            Task("T2", "charging2", "room1", description="R2 → Room1"),
        ]
        
        for task in tasks:
            self.controller.add_task(task)
    
    def update_simulation(self):
        """Update simulation"""
        self.controller.update(0.05)
        self.canvas.update()
        self.update_displays()
    
    def update_displays(self):
        """Update status displays"""
        # Status
        status = []
        for robot_id, robot in self.controller.robots.items():
            status.append(f"{robot_id}: {robot.state.value}")
            if robot.current_task:
                status.append(f"  Task: {robot.current_task.description}")
                if robot.planned_path:
                    progress = f"{robot.path_index}/{len(robot.planned_path)}"
                    status.append(f"  Progress: {progress}")
        
        self.status_text.setText("\n".join(status))
        
        # Log
        log_text = "\n".join(self.controller.log_messages[-15:])
        self.log_text.setText(log_text)

if __name__ == "__main__":
    app = QApplication(sys.argv)
    gui = DenseWaypointGUI()
    gui.show()
    sys.exit(app.exec())