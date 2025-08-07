# Fleet Demo

Fleet architecture version of the delivery robot system.

## Architecture

### Components
- **Web Server**: Handles HTTP requests, communicates with Fleet Manager
- **Fleet Manager**: Manages robot allocation and task distribution
- **Robot Nodes**: Individual robot controllers with namespaces

### Communication Flow
```
HTTP Request → Web Server → Fleet Manager → Robot Node
HTTP Response ← Web Server ← Fleet Manager ← Robot Node
```

## Running the System

### Single Command (Recommended)
```bash
python run_fleet.py
```
This starts all components in the correct order.

### Manual Start (for debugging)
1. Start Fleet Manager:
   ```bash
   python fleet_manager/fleet_manager.py
   ```

2. Start Robot Node(s):
   ```bash
   python robot_nodes/robot_node.py robot_1
   # Add more robots: python robot_nodes/robot_node.py robot_2
   ```

3. Start Web Server:
   ```bash
   python -c "from web import create_app; create_app().run(host='0.0.0.0', port=8080)"
   ```

## Key Differences from Original

1. **Separation of Concerns**: Web server only handles HTTP, no robot control
2. **Fleet Manager**: Central robot management and task allocation
3. **Scalable**: Easy to add more robots by starting additional robot nodes
4. **ROS Topics**: Proper namespacing for multi-robot support

## ROS Topics
- `/fleet/task_request` - Web server → Fleet Manager
- `/fleet/task_response` - Fleet Manager → Web server  
- `/fleet/confirm_request` - Web server → Fleet Manager
- `/robot_1/user_cmd` - Fleet Manager → Robot
- `/robot_1/status` - Robot → Fleet Manager

## Adding New Features

Fleet 구조에서 기능을 추가할 때는 **어떤 종류의 기능인지**에 따라 추가할 위치가 달라집니다.

### 1. **웹 API 관련 기능** → `web/routes/`
새로운 HTTP 엔드포인트나 웹 인터페이스 기능

**예시: 로봇 상태 조회 API**
```python
# web/routes/status.py (새 파일)
@bp.route("/robot_status", methods=["GET"])
def get_robot_status():
    # Fleet Manager에게 상태 요청
    pass
```

### 2. **로봇 동작/제어 기능** → `robot/`
로봇의 새로운 동작이나 FSM 상태

**예시: 청소 기능 추가**
```python
# robot/fsm.py 수정
class Step(IntEnum):
    CLEANING = auto()  # 새 상태 추가

# robot/cleaning.py (새 파일)
def start_cleaning():
    pass
```

### 3. **Fleet 관리 기능** → `fleet_manager/`
다중 로봇 스케줄링, 작업 우선순위, 로봇 할당 로직

**예시: 작업 우선순위 기능**
```python
# fleet_manager/task_scheduler.py (새 파일)
class TaskScheduler:
    def assign_high_priority_task(self):
        pass

# fleet_manager/fleet_manager.py 수정
from .task_scheduler import TaskScheduler
```

### 4. **공통 데이터/설정** → `shared/`
메시지 타입, 설정 값, 유틸리티 함수

**예시: 새로운 작업 타입**
```python
# shared/fleet_msgs.py 수정
class TaskType(Enum):
    DELIVERY = "delivery"
    CLEANING = "cleaning"  # 새 타입 추가
```

### 5. **데이터베이스 관련** → `web/`
재고, 주문, 사용자 관련 기능

**예시: 주문 히스토리**
```python
# web/data_access.py 수정
def get_order_history(resident_id):
    pass
```

### 기능별 추가 가이드

| 기능 유형 | 주요 위치 | 추가 고려사항 |
|-----------|-----------|---------------|
| 새로운 HTTP API | `web/routes/` | Flask Blueprint 등록 필요 |
| 로봇 새 동작 | `robot/` | FSM 상태 추가, ROS 토픽 고려 |
| Fleet 로직 | `fleet_manager/` | 다중 로봇 영향 고려 |
| UI 변경 | `web/templates/`, `web/static/` | 프론트엔드 수정 |
| 설정/상수 | `config.py`, `shared/` | 모든 컴포넌트에서 접근 가능 |

### 구체적인 추가 예시: 로봇 배터리 모니터링

만약 "로봇 배터리 모니터링" 기능을 추가한다면:

1. **shared/fleet_msgs.py** - 배터리 정보 메시지 정의
2. **robot/battery_monitor.py** - 배터리 상태 읽기
3. **robot/fsm.py** - 저배터리 시 자동 충전 로직  
4. **fleet_manager/fleet_manager.py** - 배터리 정보 수집/관리
5. **web/routes/status.py** - 배터리 상태 API