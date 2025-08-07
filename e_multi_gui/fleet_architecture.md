# Fleet Architecture Design

## Current Structure Issues
- Web server directly controls robot via `robot.ros_node`
- Tight coupling between HTTP requests and robot FSM
- Single robot hardcoded

## New Fleet Architecture

### Components
1. **Web Server** - Only handles HTTP requests, no robot logic
2. **Fleet Manager** - Manages robot allocation and task distribution  
3. **Robot Nodes** - Individual robot controllers with namespaces

### Communication Flow
```
HTTP Request → Web Server → Fleet Manager → Robot Node
HTTP Response ← Web Server ← Fleet Manager ← Robot Node
```

### ROS Topics
- `/fleet/task_request` - Web server sends tasks to fleet manager
- `/fleet/task_response` - Fleet manager responds to web server
- `/robot_{id}/cmd` - Fleet manager sends commands to specific robots
- `/robot_{id}/status` - Robots report status back to fleet manager

### Key Benefits
- Separation of concerns
- Easy to add more robots
- Centralized robot management
- Web server stays simple