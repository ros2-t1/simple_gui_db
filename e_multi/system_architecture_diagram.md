# Multi-Robot Fleet Management System Architecture

## System Components Diagram

```mermaid
graph TB
    subgraph "User Layer"
        U1[Resident Users]
        U2[Admin Users]
        U3[Mobile Apps]
    end
    
    subgraph "Web Layer"
        WS[Web Server<br/>Flask/Python]
        AUTH[Auth Module]
        API[REST API]
        SESS[Session Manager]
    end
    
    subgraph "Database Layer"
        DB[(PostgreSQL<br/>Database)]
        subgraph "Tables"
            T1[residents]
            T2[tasks]
            T3[items]
            T4[locations]
            T5[hana_bots]
            T6[login_logs]
        end
    end
    
    subgraph "ROS2 Middleware"
        FM[Fleet Manager<br/>Domain 129]
        DB_BRIDGE[Domain Bridge]
    end
    
    subgraph "Robot Fleet"
        R1[Robot 1<br/>Domain 18]
        R2[Robot 2<br/>Domain 19]
        RN[Robot N<br/>Domain N]
    end
    
    subgraph "Hardware Layer"
        ARM[Robot Arm<br/>Domain 14]
        NAV[Navigation<br/>System]
        CAM[Camera<br/>System]
    end
    
    U1 & U2 & U3 --> WS
    WS --> AUTH & API & SESS
    API --> DB
    WS <--> FM
    FM <--> DB_BRIDGE
    DB_BRIDGE <--> R1 & R2 & RN
    R1 & R2 & RN <--> ARM
    R1 & R2 & RN --> NAV
    WS --> CAM
```

## Data Flow Architecture

```mermaid
sequenceDiagram
    participant User
    participant WebServer
    participant FleetManager
    participant Robot
    participant RobotArm
    participant Database
    
    User->>WebServer: POST /order (item request)
    WebServer->>Database: Create task record
    WebServer->>FleetManager: /fleet/task_request
    FleetManager->>Database: Check robot availability
    FleetManager->>Robot: /{robot_id}/user_cmd
    Robot->>Robot: Navigate to pickup
    Robot->>RobotArm: /robot_arm/user_cmd (item_id)
    RobotArm->>RobotArm: Pick item
    RobotArm->>Robot: /robot_arm/status (complete)
    Robot->>Robot: Navigate to user
    Robot->>FleetManager: /{robot_id}/status (waiting_confirm)
    FleetManager->>Database: Update task status
    FleetManager->>WebServer: /fleet/robot_status
    WebServer->>User: Show confirmation button
    User->>WebServer: POST /confirm
    WebServer->>FleetManager: /fleet/confirm_request
    FleetManager->>Robot: Confirm received
    Robot->>Robot: Return to dock
    FleetManager->>Database: Complete task
```

## Component Interaction Matrix

| Component | Communicates With | Protocol | Purpose |
|-----------|------------------|----------|---------|
| Web Server | Database | SQL/psycopg2 | Data persistence |
| Web Server | Fleet Manager | ROS2 Topics | Task coordination |
| Web Server | Camera System | OpenCV | Video streaming |
| Fleet Manager | Robots | ROS2 Topics | Command & control |
| Fleet Manager | Database | SQL | State synchronization |
| Robots | Robot Arm | ROS2 Topics | Item handling |
| Robots | Navigation | ROS2 Actions | Movement control |
| Domain Bridge | All Domains | DDS | Cross-domain routing |

## Domain Isolation Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  Central Server (Domain 129)            │
│  ┌────────────┐  ┌──────────────┐  ┌────────────────┐ │
│  │ Web Server │  │Fleet Manager │  │  PostgreSQL    │ │
│  └────────────┘  └──────────────┘  └────────────────┘ │
└─────────────────────────┬───────────────────────────────┘
                          │ Domain Bridge
        ┌─────────────────┼─────────────────┐
        │                 │                 │
┌───────▼──────┐  ┌───────▼──────┐  ┌──────▼───────┐
│   Robot 1    │  │   Robot 2    │  │  Robot Arm  │
│  Domain 18   │  │  Domain 19   │  │  Domain 14  │
│              │  │              │  │             │
│ ┌──────────┐ │  │ ┌──────────┐ │  │ ┌─────────┐│
│ │   FSM    │ │  │ │   FSM    │ │  │ │Control  ││
│ ├──────────┤ │  │ ├──────────┤ │  │ ├─────────┤│
│ │Navigation│ │  │ │Navigation│ │  │ │Gripper  ││
│ ├──────────┤ │  │ ├──────────┤ │  │ ├─────────┤│
│ │  Sensors │ │  │ │  Sensors │ │  │ │Vision   ││
│ └──────────┘ │  │ └──────────┘ │  │ └─────────┘│
└──────────────┘  └──────────────┘  └─────────────┘
```

## Deployment Architecture

### Development Environment
```yaml
Single Machine Deployment:
  - All components on localhost
  - Single domain (129) or multi-domain with bridge
  - SQLite or local PostgreSQL
  - Simulated robots and arm
```

### Production Environment
```yaml
Distributed Deployment:
  Central Server:
    - Web Server (0.0.0.0:8080)
    - Fleet Manager (ROS2)
    - PostgreSQL Database
    - Domain Bridge
    
  Robot Machines:
    - Robot Node (ROS2)
    - Navigation Stack
    - Sensor Drivers
    - Local Config
    
  Robot Arm Station:
    - Arm Controller (ROS2)
    - Vision System
    - Safety Systems
```

## Network Architecture

```
                    Internet
                        │
                   [Firewall]
                        │
              ┌─────────┴─────────┐
              │   Load Balancer   │
              └─────────┬─────────┘
                        │
        ┌───────────────┼───────────────┐
        │               │               │
   [Web Server 1]  [Web Server 2]  [Web Server N]
        │               │               │
        └───────────────┼───────────────┘
                        │
                 [Message Queue]
                        │
                 [Fleet Manager]
                        │
              ┌─────────┴─────────┐
              │   Domain Bridge   │
              └─────────┬─────────┘
                        │
        ┌───────┬───────┼───────┬───────┐
        │       │       │       │       │
    [Robot 1] [Robot 2] [Arm] [Robot N] [...]
```

## State Management

### Task State Machine
```
    ┌──────┐
    │ 대기 │ (Pending)
    └───┬──┘
        │ Assign
    ┌───▼──┐
    │ 할당 │ (Assigned)
    └───┬──┘
        │ Start
    ┌───▼────┐
    │ 집기중 │ (Picking)
    └───┬────┘
        │ Move
    ┌───▼────┐
    │ 이동중 │ (Moving)
    └───┬────┘
        │ Arrive
    ┌───▼──────┐
    │ 수령대기 │ (Waiting)
    └───┬──────┘
        │ Confirm
    ┌───▼──┐
    │ 완료 │ (Completed)
    └──────┘
```

### Robot State Machine
```
    ┌──────┐
    │ IDLE │
    └───┬──┘
        │ Order
    ┌───▼────────┐
    │ GO_TO_ARM  │
    └───┬────────┘
        │ Arrived
    ┌───▼──┐
    │ PICK │────────┐
    └───┬──┘        │
        │       ┌───▼──────┐
        │       │ WAIT_ARM │
        │       └───┬──────┘
        │           │ Complete
    ┌───▼──────────▼┐
    │  GO_TO_USER   │
    └───┬───────────┘
        │ Arrived
    ┌───▼───────────┐
    │ WAIT_CONFIRM  │
    └───┬───────────┘
        │ Confirmed
    ┌───▼──────┐
    │ GO_DOCK  │
    └───┬──────┘
        │ Docked
    ┌───▼──┐
    │ IDLE │
    └──────┘
```

## Security Architecture

### Authentication Flow
```
User → Login → Web Server → Validate → Database
                  │                        │
                  └── Create Session ←─────┘
                            │
                      [Session Token]
                            │
                  All subsequent requests
```

### Authorization Levels
1. **Guest**: View only
2. **Resident**: Order, call, confirm own tasks
3. **Staff**: Monitor all tasks, manual control
4. **Admin**: Full system control, configuration

### Security Measures
- **Password**: Bcrypt hashing with salt
- **Session**: Server-side session storage
- **API**: Rate limiting and input validation
- **ROS2**: DDS security (optional)
- **Network**: HTTPS/TLS encryption
- **Database**: Connection pooling, prepared statements

## Performance Metrics

### Key Performance Indicators (KPIs)
- **Task Completion Rate**: > 95%
- **Average Delivery Time**: < 5 minutes
- **Robot Utilization**: > 70%
- **System Uptime**: > 99.9%
- **API Response Time**: < 200ms
- **Robot Response Time**: < 100ms

### Monitoring Points
1. Database query performance
2. ROS2 topic latency
3. Robot navigation accuracy
4. Task queue length
5. Error rates by component
6. Resource utilization (CPU, Memory, Network)

## Scalability Considerations

### Horizontal Scaling
- **Web Servers**: Load balanced instances
- **Robots**: Add more units with unique domains
- **Database**: Read replicas for queries

### Vertical Scaling
- **Fleet Manager**: Multi-threaded executor
- **Database**: Connection pooling, indexing
- **Domain Bridge**: Optimized routing tables

### Bottleneck Mitigation
- **Task Queue**: Priority-based scheduling
- **Robot Assignment**: Proximity-based allocation
- **Database**: Caching frequently accessed data
- **Network**: Local domain isolation