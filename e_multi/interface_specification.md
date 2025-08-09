# Multi-Robot Fleet Management System Interface Specification

## 1. System Architecture Overview

```
┌─────────────┐     HTTP/WebSocket    ┌──────────────┐
│  Web Client ├──────────────────────►│  Web Server  │
└─────────────┘                       └──────┬───────┘
                                             │ ROS2
                                    ┌────────▼────────┐
                                    │ Fleet Manager   │
                                    │ (Domain 129)    │
                                    └────────┬────────┘
                                             │ Domain Bridge
                          ┌──────────────────┼──────────────────┐
                          │                  │                  │
                    ┌─────▼─────┐     ┌─────▼─────┐     ┌─────▼─────┐
                    │  Robot 1  │     │  Robot 2  │     │ Robot Arm │
                    │(Domain 18)│     │(Domain 19)│     │(Domain 14)│
                    └───────────┘     └───────────┘     └───────────┘
```

## 2. ROS2 Topic Interface Specification

### 2.1 Fleet Management Topics

#### `/fleet/task_request` 
- **Publisher**: Web Server
- **Subscriber**: Fleet Manager
- **Message Type**: `std_msgs/String`
- **Format**: JSON
```json
{
  "task_id": "unique_id",
  "db_task_id": 123,
  "task_type": "배달|호출",
  "requester_resident_id": 1,
  "item_id": 10
}
```

#### `/fleet/task_response`
- **Publisher**: Fleet Manager
- **Subscriber**: Web Server
- **Message Type**: `std_msgs/String`
- **Format**: JSON
```json
{
  "task_id": "unique_id",
  "status": "pending|assigned|completed|failed",
  "robot_id": "robot_1",
  "message": "Task status message"
}
```

#### `/fleet/confirm_request`
- **Publisher**: Web Server
- **Subscriber**: Fleet Manager
- **Message Type**: `std_msgs/String`
- **Data**: "confirm" or robot_id

#### `/fleet/robot_status`
- **Publisher**: Fleet Manager
- **Subscriber**: Web Server
- **Message Type**: `std_msgs/String`
- **Format**: JSON
```json
{
  "robot_id": "robot_1",
  "status": "idle|moving_to_arm|picking|moving_to_user|waiting_confirm|returning_to_dock",
  "current_task_id": "123",
  "timestamp": 1234567890.123
}
```

### 2.2 Robot Control Topics (Per Robot Namespace)

#### `/{robot_id}/user_cmd`
- **Publisher**: Fleet Manager
- **Subscriber**: Robot FSM
- **Message Type**: `std_msgs/String`
- **Format**: JSON
```json
{
  "command": "order|confirm|stop",
  "resident_id": 1,
  "task_type": "배달|호출",
  "item_id": 10,
  "target_coordinates": [x, y, z]
}
```

#### `/{robot_id}/status`
- **Publisher**: Robot FSM
- **Subscriber**: Fleet Manager
- **Message Type**: `std_msgs/String`
- **Values**: `idle|moving_to_arm|picking|moving_to_user|waiting_confirm|returning_to_dock|navigation_failed`

### 2.3 Robot Arm Interface Topics

#### `/robot_arm/user_cmd`
- **Publisher**: Robot FSM (any robot)
- **Subscriber**: Robot Arm Controller
- **Message Type**: `std_msgs/Int32`
- **Data**: item_id to pick

#### `/robot_arm/status`
- **Publisher**: Robot Arm Controller
- **Subscriber**: Robot FSM (all robots)
- **Message Type**: `std_msgs/String`
- **Values**: `complete|error|busy`

### 2.4 Domain Bridge Configuration

```yaml
Domains:
  - Central Server: 129 (Fleet Manager, Web Server)
  - Robot 1: 18
  - Robot 2: 19
  - Robot Arm: 14

Bridge Topics:
  - robot_1/* : Domain 129 ↔ Domain 18
  - robot_2/* : Domain 129 ↔ Domain 19
  - robot_arm/* : Domains 18,19 ↔ Domain 14
```

## 3. HTTP REST API Specification

### 3.1 Authentication Endpoints

#### `POST /login`
Login authentication
- **Request Body**:
```json
{
  "username": "string",
  "password": "string"
}
```
- **Response**: 
```json
{
  "success": true,
  "resident_id": 1,
  "name": "김복자",
  "message": "Login successful"
}
```

### 3.2 Order Management Endpoints

#### `POST /order`
Create delivery order
- **Request Body**:
```json
{
  "item_type": "물|영양제|생필품|식판",
  "quantity": 1
}
```
- **Response**:
```json
{
  "success": true,
  "task_id": "uuid",
  "message": "Order placed successfully"
}
```

#### `POST /call`
Request robot summon
- **Request Body**:
```json
{
  "reason": "string"
}
```
- **Response**:
```json
{
  "success": true,
  "task_id": "uuid",
  "message": "Robot called successfully"
}
```

#### `POST /confirm`
Confirm task completion
- **Request Body**:
```json
{
  "robot_id": "robot_1"
}
```
- **Response**:
```json
{
  "success": true,
  "message": "Task confirmed"
}
```

### 3.3 Status & Monitoring Endpoints

#### `GET /robot_status`
Get all robots status
- **Response**:
```json
{
  "robot_1": {
    "status": "idle",
    "task_id": null,
    "battery": 85,
    "last_update": "2025-08-08T10:30:00Z"
  },
  "robot_2": {
    "status": "moving_to_user",
    "task_id": "123",
    "battery": 72,
    "last_update": "2025-08-08T10:31:00Z"
  }
}
```

#### `GET /robot_status/{robot_id}`
Get specific robot status
- **Query Parameters**: 
  - `resident_id`: Optional, check if robot coming to this resident
- **Response**:
```json
{
  "robot_id": "robot_1",
  "status": "moving_to_user",
  "task_id": "123",
  "is_for_resident": true,
  "eta_seconds": 120
}
```

#### `GET /items`
Get inventory status
- **Response**:
```json
[
  {
    "item_id": 9,
    "item_type": "물",
    "quantity": 45
  },
  {
    "item_id": 10,
    "item_type": "영양제",
    "quantity": 30
  }
]
```

### 3.4 Fleet Management API

#### `GET /api/fleet/status`
Fleet overview status
- **Response**:
```json
{
  "robots": {
    "total": 2,
    "idle": 1,
    "busy": 1,
    "offline": 0
  },
  "tasks": {
    "pending": 3,
    "in_progress": 2,
    "completed_today": 45
  },
  "system_health": "operational"
}
```

#### `GET /api/fleet/analytics`
Fleet performance analytics
- **Response**:
```json
{
  "performance": {
    "avg_delivery_time": 180,
    "success_rate": 0.95,
    "tasks_per_hour": 12
  },
  "utilization": {
    "robot_1": 0.75,
    "robot_2": 0.82
  }
}
```

#### `POST /api/fleet/control`
Fleet control commands
- **Request Body**:
```json
{
  "command": "pause|resume|reset",
  "target": "all|robot_1|robot_2"
}
```

### 3.5 Admin API Endpoints

#### `GET /api/admin/robots`
Admin robot management
- **Response**: List of all robots with detailed status

#### `GET /api/admin/tasks`
Admin task management
- **Query Parameters**: 
  - `status`: Filter by status
  - `date`: Filter by date
  - `robot_id`: Filter by robot
- **Response**: List of tasks with full details

#### `GET /api/admin/users`
Admin user management
- **Response**: List of all residents with statistics

#### `GET /api/admin/items`
Admin inventory management
- **Response**: Full inventory with history

### 3.6 Camera Streaming Endpoints

#### `GET /camera/stream/{camera_id}`
MJPEG video stream
- **Response**: Multipart MJPEG stream

#### `POST /camera/start/{camera_id}`
Start camera stream
- **Response**: `{"success": true, "message": "Camera started"}`

#### `POST /camera/stop/{camera_id}`
Stop camera stream
- **Response**: `{"success": true, "message": "Camera stopped"}`

#### `GET /camera/status`
All cameras status
- **Response**: Array of camera statuses

## 4. WebSocket Events (Future Enhancement)

### Events from Server
- `robot_status_update`: Real-time robot status changes
- `task_created`: New task created
- `task_completed`: Task completed
- `alert`: System alerts and notifications

### Events from Client
- `subscribe_robot`: Subscribe to specific robot updates
- `unsubscribe_robot`: Unsubscribe from robot updates

## 5. Error Response Format

All API endpoints follow consistent error response format:
```json
{
  "success": false,
  "error": {
    "code": "ERROR_CODE",
    "message": "Human-readable error message",
    "details": {}  // Optional additional error details
  }
}
```

Common Error Codes:
- `AUTH_REQUIRED`: Authentication required
- `AUTH_FAILED`: Authentication failed
- `INVALID_REQUEST`: Invalid request parameters
- `RESOURCE_NOT_FOUND`: Requested resource not found
- `ROBOT_UNAVAILABLE`: No robots available
- `TASK_FAILED`: Task execution failed
- `SYSTEM_ERROR`: Internal system error

## 6. Security Considerations

1. **Authentication**: Session-based authentication with bcrypt password hashing
2. **Authorization**: Role-based access control (resident vs admin)
3. **Rate Limiting**: API rate limiting per IP/user
4. **Input Validation**: All inputs sanitized and validated
5. **HTTPS**: All HTTP communication over TLS
6. **ROS2 Security**: DDS security extensions for encrypted ROS communication

## 7. Performance Requirements

- **API Response Time**: < 200ms for queries, < 500ms for operations
- **Robot Status Update**: Real-time with < 100ms latency
- **Task Assignment**: < 2 seconds from request to robot dispatch
- **Camera Stream**: 30 FPS with < 100ms latency
- **System Capacity**: Support 10+ robots, 100+ concurrent users

## 8. Data Flow Examples

### Delivery Task Flow
1. User → `POST /order` → Web Server
2. Web Server → `/fleet/task_request` → Fleet Manager
3. Fleet Manager → `/{robot_id}/user_cmd` → Robot
4. Robot → `/robot_arm/user_cmd` → Robot Arm (at pickup)
5. Robot Arm → `/robot_arm/status` → Robot
6. Robot → `/{robot_id}/status` → Fleet Manager
7. Fleet Manager → `/fleet/robot_status` → Web Server
8. User → `POST /confirm` → Web Server
9. Web Server → `/fleet/confirm_request` → Fleet Manager

### Status Monitoring Flow
1. Robot → `/{robot_id}/status` → Fleet Manager (every 2s)
2. Fleet Manager → `/fleet/robot_status` → Web Server
3. Client → `GET /robot_status` → Web Server
4. Web Server → Return cached status → Client