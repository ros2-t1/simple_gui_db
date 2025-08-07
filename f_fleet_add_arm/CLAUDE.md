# CLAUDE.md

이 파일은 이 저장소의 코드와 함께 작업할 때 Claude Code (claude.ai/code)에게 지침을 제공합니다.

## 명령어

### 개발환경 (단일 머신)
```bash
python run_fleet.py  # 모든 구성 요소를 단일 머신에서 실행
```

### 수동 구성 요소 시작 (디버깅용)
```bash
# 1. Fleet Manager 시작
python fleet_manager/fleet_manager.py

# 2. Robot Node(s) 시작
python robot_nodes/robot_node.py robot_1

# 3. Web Server 시작
python -c "from web import create_app; create_app().run(host='0.0.0.0', port=8080)"
```

### 분산 배포 (실제 운영)
```bash
# 중앙 서버에서
python fleet_manager/fleet_manager.py  # Fleet Manager 시작
python -c "from web import create_app; create_app().run(host='0.0.0.0', port=8080)"  # Web Server 시작

# 로봇 패키지 생성 및 배포
python deploy_robot.py robot_1 --fleet-manager-ip <중앙서버IP>  # 로봇 배포 패키지 생성

# 각 로봇 머신에서 (배포 패키지 복사 후)
cd robot_package && ./start_robot.sh
```

### 데이터베이스 작업
```bash
python check_db.py  # 데이터베이스 연결 및 상태 확인
```

## 아키텍처

이것은 주거용 건물의 배송 로봇을 위한 ROS2 통합 다중 로봇 플릿 관리 시스템입니다.

### 핵심 구성 요소
- **Web Server** (`web/`): Flask 기반 HTTP API 및 대시보드, 사용자 요청 및 웹 인터페이스 처리
- **Fleet Manager** (`fleet_manager/`): ROS2를 사용한 로봇 할당 및 작업 분배를 위한 중앙 조정자
- **Robot Nodes** (`robot_nodes/`): 다중 로봇 지원을 위한 ROS2 네임스페이싱이 있는 개별 로봇 컨트롤러
- **Robot Control** (`robot/`): 팔 제어 및 내비게이션을 포함한 FSM 기반 로봇 동작
- **Shared** (`shared/`): 공통 메시지 정의 및 데이터 구조

### 통신 흐름
```
HTTP Request → Web Server → Fleet Manager (ROS2) → Robot Node → Robot FSM
```

### ROS2 토픽
- `/fleet/task_request` - Web server → Fleet Manager
- `/fleet/task_response` - Fleet Manager → Web server  
- `/robot_1/user_cmd` - Fleet Manager → Robot
- `/robot_1/status` - Robot → Fleet Manager

### 데이터베이스 통합
- 주민 데이터, 주문 및 재고가 있는 PostgreSQL 데이터베이스
- `web/task_db.py`를 통한 작업 관리
- `config.py`의 데이터베이스 구성

### 주요 디자인 패턴
- **관심사 분리**: Web server는 HTTP만 처리하고, Fleet Manager는 로봇 조정을 처리
- **ROS2 네임스페이싱**: 개별 네임스페이스가 있는 적절한 다중 로봇 지원
- **FSM 로봇 제어**: `robot/fsm.py`의 상태 머신 기반 로봇 동작
- **데이터베이스 기반 작업**: PostgreSQL을 통해 관리되는 작업 생명주기
- **분산 아키텍처 지원**: 로봇이 별도 머신에서 실행 가능하도록 DB 의존성 제거

### 새 기능 추가
다양한 유형의 기능을 추가할 위치에 대한 자세한 지침은 README.md 59-140줄을 참조하세요:
- Web API 기능 → `web/routes/`
- 로봇 동작 → `robot/`
- 플릿 관리 → `fleet_manager/`
- 공통 데이터/메시지 → `shared/`
- 데이터베이스 작업 → `web/`

### 구성
- **중앙 서버**: `config.py`의 데이터베이스 연결 및 전역 설정
- **로봇 개별 설정**: `robot_config.py`의 로봇별 독립 설정 (DB 접근 없음)
- **배포 도구**: `deploy_robot.py`로 로봇별 배포 패키지 생성
- 단일 로봇 배포 (robot_1이 데이터베이스 ID 10 - HANA_PINKY에 매핑됨)

### 분산 배포 아키텍처
```
중앙 서버 (데이터베이스 접근)
├── Web Server (Flask)
├── Fleet Manager (ROS2) - 좌표 조회 및 전송
└── PostgreSQL Database

로봇 머신들 (DB 접근 없음)  
├── Robot Node (ROS2)
├── Robot FSM - 좌표를 Fleet Manager에서 수신
├── Navigation & Control
└── robot_config.py (독립 설정)
```

## 최근 수정사항 (2025-08-06)

### 문제: 서버 재시작 후 확인 알림 미표시 및 다중 작업 할당

**증상:**
- 서버 재시작 시 기존 '수령대기' 상태 작업의 확인 알림이 웹에서 표시되지 않음
- 단일 로봇에 여러 작업이 동시 할당되는 race condition 발생

### 해결책

#### 1. 웹 상태 캐시 초기화 개선 (`web/routes/status.py`)
```python
def initialize_robot_status_from_db():
    # 시간 기반 휴리스틱 추가
    if task['task_type'] == '배달' and db_status == '이동중':
        elapsed_minutes = (datetime.now(timezone.utc) - created_at).total_seconds() / 60
        # 3분 이상 경과 시 도착한 것으로 추정
        if elapsed_minutes > 3:
            robot_status = 'waiting_confirm'
```

**개선사항:**
- 서버 재시작 시 기존 '이동중' 작업의 실제 진행 상황을 시간으로 추정
- 3분 이상 진행된 배달 작업은 도착 완료 상태로 가정
- 상세한 디버깅 로그로 캐시 초기화 과정 추적

#### 2. Fleet Manager 다중 할당 방지 (`fleet_manager/fleet_manager.py`)
```python
def _sync_robot_states_with_db(self):
    # 로봇 상태를 데이터베이스와 동기화
    current_db_task = get_robot_current_task(robot_db_id)
    if current_db_task:
        robot_state.current_task_id = str(current_db_task['task_id'])
        robot_state.status = RobotStatus.BUSY

def check_and_assign_tasks(self):
    # DB 동기화 후 가용성 검사
    self._sync_robot_states_with_db()
    
    # 이중 확인으로 race condition 방지
    double_check_task = get_robot_current_task(robot_db_id)
    if double_check_task:
        self.get_logger().warn(f"RACE CONDITION PREVENTED")
        return
```

**개선사항:**
- `_sync_robot_states_with_db()` 함수로 로컬 상태와 DB 상태 동기화
- 작업 할당 전 데이터베이스 이중 확인으로 동시 할당 방지
- 데이터베이스를 신뢰할 수 있는 단일 정보원(Single Source of Truth)으로 활용

#### 3. 핵심 원리
- **상태 동기화**: Fleet Manager 로컬 상태 ↔ 데이터베이스 상태 일치
- **시간 휴리스틱**: 장시간 진행된 작업의 실제 상태 추정
- **Race Condition 방지**: 원자적 데이터베이스 확인 및 업데이트

#### 4. 실시간 상태 동기화 개선 (`fleet_manager/fleet_manager.py`)
```python
elif status_text == "waiting_confirm":
    # Robot arrived at user location - update DB to 수령대기 
    with query_db() as cur:
        cur.execute("""
            UPDATE tasks SET status = '수령대기' WHERE task_id = %s
        """, (int(robot_state.current_task_id),))
```

**개선사항:**
- Fleet Manager가 로봇 상태 변화를 실시간으로 DB에 반영
- `waiting_confirm` 상태 시 DB를 `수령대기`로 즉시 업데이트
- 모든 로봇 상태(`이동중`, `집기중`, `수령대기`)가 DB와 동기화

#### 5. 테스트 방법
1. 새 주문 생성 → 로봇이 자동으로 출발하는지 확인
2. 로봇 도착 시 웹에서 확인 버튼이 바로 표시되는지 확인
3. 서버 재시작 후에도 기존 작업 상태가 올바르게 유지되는지 확인