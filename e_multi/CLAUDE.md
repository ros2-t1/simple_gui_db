# CLAUDE.md

이 파일은 이 저장소의 코드와 함께 작업할 때 Claude Code (claude.ai/code)에게 지침을 제공합니다.

## 명령어

### 개발환경

#### 단일 머신 (기존 방식)
```bash
python run_fleet.py  # 모든 구성 요소를 단일 머신에서 실행 (Domain 129)
```

#### 멀티도메인 시스템 (권장)
```bash
python run_multirobot_fleet.py  # Domain Bridge 포함 멀티로봇 시스템
```

### 수동 구성 요소 시작 (디버깅용)
```bash
# 1. Fleet Manager 시작
python fleet_manager/fleet_manager.py

# 2. Robot Node(s) 시작  
python mobile_robot/robot_nodes/robot_node.py robot_1

# 3. Web Server 시작
python -c "from web import create_app; create_app().run(host='0.0.0.0', port=8080)"
```

### 분산 배포 (실제 운영)

#### 멀티도메인 분산 배포
```bash
# 중앙 서버에서 (Domain 129)
python run_multirobot_fleet.py  # Fleet Manager + Domain Bridge + Web Server

# 각 로봇 머신에서
cd mobile_robot
./start_robot.sh robot_1  # Robot 1 (Domain 18)
./start_robot.sh robot_2  # Robot 2 (Domain 19)
```

#### 기존 배포 방식
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
python test_db_sync.py  # DB 동기화 기능 테스트
```

## 아키텍처

이것은 주거용 건물의 배송 로봇을 위한 ROS2 통합 다중 로봇 플릿 관리 시스템입니다.

### 멀티도메인 아키텍처
시스템은 ROS2 Domain Bridge를 사용하여 각 로봇을 독립적인 도메인에서 실행합니다:
- **중앙 서버**: Domain 129 (Fleet Manager + Web Server)
- **Robot 1**: Domain 18 (hana_bot_id: 8)
- **Robot 2**: Domain 19 (hana_bot_id: 9)

Domain Bridge가 도메인 간 통신을 중계하여 네트워크 격리와 확장성을 제공합니다.

### 핵심 구성 요소
- **Web Server** (`web/`): Flask 기반 HTTP API 및 대시보드, 사용자 요청 및 웹 인터페이스 처리
- **Fleet Manager** (`fleet_manager/`): ROS2를 사용한 로봇 할당 및 작업 분배를 위한 중앙 조정자 (Domain Bridge 포함)
- **Robot Nodes** (`mobile_robot/robot_nodes/`): 다중 로봇 지원을 위한 ROS2 네임스페이싱이 있는 개별 로봇 컨트롤러
- **Robot Control** (`mobile_robot/robot/`): 팔 제어 및 내비게이션을 포함한 FSM 기반 로봇 동작
- **Shared** (`shared/`): 공통 메시지 정의 및 데이터 구조
- **Domain Bridge Config** (`config/domain.yaml`): 멀티도메인 통신 설정

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
다양한 유형의 기능을 추가할 위치에 대한 지침:
- Web API 기능 → `web/routes/`
- 로봇 동작 → `mobile_robot/robot/`
- 로봇 노드 → `mobile_robot/robot_nodes/`
- 플릿 관리 → `fleet_manager/`
- 공통 데이터/메시지 → `shared/`
- 데이터베이스 작업 → `web/`
- 도메인 브리지 설정 → `config/domain.yaml`

### 구성
- **중앙 서버**: `config.py`의 데이터베이스 연결 및 전역 설정
- **Fleet 구성**: `fleet_config.yaml`의 로봇별 도메인 ID 및 브리지 설정
  - robot_1: Domain 18, hana_bot_id 8
  - robot_2: Domain 19, hana_bot_id 9
- **Domain Bridge**: `config/domain.yaml`의 도메인 간 토픽 라우팅 설정
- **로봇 개별 설정**: `mobile_robot/robot_config.py`의 로봇별 독립 설정 (DB 접근 없음)
- **배포 도구**: `deploy_robot.py`로 로봇별 배포 패키지 생성

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

## 수정사항 및 문제해결 히스토리 (2025-08-06)

### 해결된 주요 문제들

#### 1. 서버 재시작 후 확인 알림 미표시 문제 ✅
**문제**: 서버 재시작 시 기존 '수령대기' 상태 작업의 확인 버튼/팝업이 웹에서 표시되지 않음
**원인**: 웹 서버 캐시 초기화 시 DB의 '이동중' 상태를 '수령대기'로 올바르게 변환하지 못함
**해결**: 
- 시간 기반 휴리스틱으로 3분 이상 경과한 '이동중' 작업을 '수령대기'로 추정
- 웹 상태 캐시 초기화 로직 개선 (`web/routes/status.py`)

#### 2. 다중 작업 할당 문제 ✅  
**문제**: 단일 로봇에 여러 작업이 동시 할당되는 race condition
**원인**: Fleet Manager 로컬 상태와 DB 상태 불일치
**해결**:
- DB 동기화 함수 `_sync_robot_states_with_db()` 추가
- 작업 할당 전 이중 확인으로 race condition 방지
- 데이터베이스를 Single Source of Truth로 활용

#### 3. 실시간 상태 동기화 부족 ✅
**문제**: 로봇이 도착(`waiting_confirm`)해도 DB가 '수령대기'로 업데이트되지 않음
**원인**: Fleet Manager가 로봇 상태 변화를 DB에 실시간 반영하지 않음
**해결**:
```python
elif status_text == "waiting_confirm":
    # Robot arrived - update DB immediately
    with query_db() as cur:
        cur.execute("UPDATE tasks SET status = '수령대기' WHERE task_id = %s", 
                   (int(robot_state.current_task_id),))
```

#### 4. 할당 상태 묶임 문제 (Stuck Assignments) ✅
**문제**: 시간이 지나면 Task는 'assigned' 상태인데 로봇은 'idle' 상태로 묶이는 현상
**원인**: Fleet Manager가 작업 할당 후 로봇 명령 전송에 실패하거나 누락
**해결**: 
- 자동 복구 시스템 `recover_stuck_assignments()` 추가
- 시작 시 2분 이상 된 '할당' 작업 중 로봇이 idle인 경우 자동으로 '대기'로 리셋

### 핵심 개선사항

#### 실시간 DB-로봇 상태 동기화
```python
# Fleet Manager에서 모든 로봇 상태 변화를 DB에 즉시 반영
"moving_to_arm" → DB: '이동중'
"picking" → DB: '집기중'  
"moving_to_user" → DB: '이동중'
"waiting_confirm" → DB: '수령대기' ← 핵심!
```

#### 자동 복구 시스템
```python
def recover_stuck_assignments(self):
    # Find stuck 'assigned' tasks where robot is idle
    # Reset them to 'pending' for reprocessing
    # Runs once on Fleet Manager startup
```

#### 상태 동기화 원칙
- **DB = 신뢰할 수 있는 단일 정보원**: 모든 상태 결정은 DB 기준
- **실시간 업데이트**: 로봇 상태 변화 즉시 DB 반영
- **자동 복구**: 시스템 재시작 시 불일치 상태 자동 감지 및 복구

### 현재 상태 (안정화 완료)

#### ✅ 정상 작동하는 기능들
- 새 주문 → 로봇 자동 출발
- 로봇 도착 → 웹에서 확인 버튼 즉시 표시  
- 서버 재시작 → 기존 작업 상태 올바르게 복원
- 장시간 대기 → 자동 복구로 묶임 방지

#### 🔧 모니터링 포인트
- Fleet Manager 로그에서 "🔄 Recovering stuck assignment" 메시지 확인
- DB와 로봇 ROS 상태의 실시간 일치 여부
- 웹 API `/robot_status/robot_1?resident_id=X`의 정확성

#### 🚀 성능 최적화
- 작업 할당 체크: 2초마다
- 타임아웃 체크: 30초마다  
- 자동 복구: 시작 시 1회 (5초 후)

### 사용 방법
```bash
# 정상 운영
python run_fleet.py

# 문제 진단 시
python check_db.py  # DB 상태 확인
ros2 topic echo /robot_1/status  # 로봇 실제 상태 확인
curl "http://localhost:8080/robot_status/robot_1?resident_id=9999"  # 웹 상태 확인
```