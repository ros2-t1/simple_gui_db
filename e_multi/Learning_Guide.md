# 🤖 E-Multi 로봇 플릿 관리 시스템 학습 가이드

## 👨‍🎓 대상 독자
- 로봇 공학 및 소프트웨어 개발을 공부하는 교육생
- ROS2와 웹 개발에 관심이 있는 학생
- 실무 프로젝트를 이해하고 싶은 초급 개발자

---

## 📚 목차
1. [시스템 소개 - 무엇을 만드는가?](#1-시스템-소개---무엇을-만드는가)
2. [전체 구조 이해하기](#2-전체-구조-이해하기)
3. [핵심 구성요소 상세 설명](#3-핵심-구성요소-상세-설명)
4. [데이터 흐름 따라가기](#4-데이터-흐름-따라가기)
5. [실제 시나리오로 배우기](#5-실제-시나리오로-배우기)
6. [코드 구조 깊이 들여다보기](#6-코드-구조-깊이-들여다보기)
7. [실습 해보기](#7-실습-해보기)
8. [자주 하는 질문](#8-자주-하는-질문)

---

## 1. 시스템 소개 - 무엇을 만드는가?

### 🏢 배경 이야기
여러분이 사는 아파트에 배송 로봇이 있다고 상상해보세요. 주민이 앱으로 물건을 주문하면, 로봇이 물건을 가져다 주는 시스템입니다.

### 🎯 시스템의 목적
- **배달 서비스**: 주민이 주문한 물건을 로봇이 배달
- **호출 서비스**: 물건 없이 로봇만 호출 (예: 쓰레기 수거)
- **다중 로봇 관리**: 여러 대의 로봇을 효율적으로 운영

### 💡 비유로 이해하기
이 시스템은 **택시 콜센터**와 비슷합니다:
- **주민** = 승객 (서비스 요청)
- **웹 서버** = 콜센터 접수원 (요청 접수)
- **플릿 매니저** = 배차 담당자 (택시 배정)
- **로봇** = 택시 (실제 서비스 수행)
- **데이터베이스** = 운행 기록부 (모든 기록 저장)

---

## 2. 전체 구조 이해하기

### 🏗️ 시스템 아키텍처

```
┌─────────────────────────────────────────────────────────────┐
│                         사용자 영역                           │
├─────────────────────────────────────────────────────────────┤
│  📱 모바일 앱        💻 웹 브라우저        🖥️ 관리자 패널    │
│       ↓                    ↓                      ↓         │
└─────────────────────────────────────────────────────────────┘
                              ↓
┌─────────────────────────────────────────────────────────────┐
│                      중앙 서버 (Domain 129)                  │
├─────────────────────────────────────────────────────────────┤
│  ┌────────────────────────────────────────────────────┐     │
│  │              🌐 Web Server (Flask)                 │     │
│  │  • HTTP API 제공                                   │     │
│  │  • 사용자 인증                                     │     │
│  │  • 주문 접수                                       │     │
│  └────────────────────────────────────────────────────┘     │
│                         ↓ ↑                                  │
│  ┌────────────────────────────────────────────────────┐     │
│  │           🎯 Fleet Manager (ROS2)                  │     │
│  │  • 로봇 상태 관리                                  │     │
│  │  • 작업 할당 결정                                  │     │
│  │  • 실시간 모니터링                                 │     │
│  └────────────────────────────────────────────────────┘     │
│                         ↓ ↑                                  │
│  ┌────────────────────────────────────────────────────┐     │
│  │           🌉 Domain Bridge (Router)                │     │
│  │  • 도메인 간 통신 중계                             │     │
│  │  • 메시지 라우팅                                   │     │
│  └────────────────────────────────────────────────────┘     │
│                         ↓ ↑                                  │
│  ┌────────────────────────────────────────────────────┐     │
│  │           💾 PostgreSQL Database                   │     │
│  │  • 주민 정보 (residents)                           │     │
│  │  • 작업 정보 (tasks)                               │     │
│  │  • 로봇 정보 (hana_bots)                          │     │
│  │  • 주문 내역 (orders)                              │     │
│  └────────────────────────────────────────────────────┘     │
└─────────────────────────────────────────────────────────────┘
                              ↓
         ┌────────────────────┴─────────────────────┐
         ↓                                          ↓
┌──────────────────┐                    ┌──────────────────┐
│   🤖 Robot 1     │                    │   🤖 Robot 2     │
│   (Domain 18)    │                    │   (Domain 19)    │
├──────────────────┤                    ├──────────────────┤
│ • Robot Node     │                    │ • Robot Node     │
│ • FSM Control    │                    │ • FSM Control    │
│ • Navigation     │                    │ • Navigation     │
│ • Arm Control    │                    │ • Arm Control    │
└──────────────────┘                    └──────────────────┘
```

### 🔑 핵심 개념 설명

#### ROS2 Domain이란?
- **도메인 = 독립된 통신 채널**
- 각 도메인은 서로 격리되어 있음
- TV 채널처럼 각자의 주파수를 가짐
- Domain Bridge가 채널 간 통신을 연결

#### 왜 멀티도메인을 사용하나요?
1. **격리**: 로봇들이 서로 간섭하지 않음
2. **보안**: 각 로봇의 통신이 분리됨
3. **확장성**: 새 로봇 추가가 쉬움

---

## 3. 핵심 구성요소 상세 설명

### 📦 1. Web Server (웹 서버)
**역할**: 사용자와 시스템을 연결하는 창구

**주요 기능**:
- 웹 페이지 제공 (HTML/CSS/JavaScript)
- REST API 엔드포인트 제공
- 사용자 인증 및 권한 관리
- 주문 데이터 검증

**핵심 파일들**:
```python
web/
├── __init__.py          # Flask 앱 초기화
├── routes/              # API 엔드포인트들
│   ├── orders.py        # 주문 관련 API
│   ├── status.py        # 상태 조회 API
│   └── auth.py          # 인증 관련 API
├── static/              # 정적 파일 (CSS, JS, 이미지)
├── templates/           # HTML 템플릿
└── task_db.py          # 데이터베이스 작업
```

**코드 예시**:
```python
# 주문 API 엔드포인트 (web/routes/orders.py)
@app.route('/api/order', methods=['POST'])
def create_order():
    data = request.json
    # 1. 데이터 검증
    if not data.get('resident_id'):
        return {'error': 'Resident ID required'}, 400
    
    # 2. DB에 작업 생성
    task_id = create_task_in_db(data)
    
    # 3. Fleet Manager에게 전달
    send_to_fleet_manager(task_id, data)
    
    return {'task_id': task_id, 'status': 'pending'}, 200
```

### 🎯 2. Fleet Manager (플릿 매니저)
**역할**: 로봇들의 중앙 관리자 (배차 담당자)

**주요 기능**:
- 로봇 상태 실시간 추적
- 최적의 로봇 선택 및 작업 할당
- 작업 타임아웃 관리
- 로봇-DB 상태 동기화

**핵심 로직**:
```python
# fleet_manager/fleet_manager.py
class FleetManager(Node):
    def __init__(self):
        # 로봇 상태 관리
        self.robots = {
            'robot_1': RobotState(status='idle'),
            'robot_2': RobotState(status='busy')
        }
        
        # 타이머 설정
        self.create_timer(2.0, self.check_and_assign_tasks)  # 2초마다 작업 확인
        self.create_timer(30.0, self.check_task_timeouts)    # 30초마다 타임아웃 확인
    
    def find_best_robot(self, task):
        """가장 적합한 로봇 찾기"""
        available_robots = [r for r in self.robots if r.status == 'idle']
        if not available_robots:
            return None
        
        # 단순 예: 첫 번째 가용 로봇 선택
        # 실제로는 거리, 배터리, 작업 이력 등 고려
        return available_robots[0]
```

### 🤖 3. Robot Node (로봇 노드)
**역할**: 개별 로봇의 두뇌

**주요 기능**:
- Fleet Manager의 명령 수신
- FSM(유한 상태 기계)으로 행동 제어
- 네비게이션 실행
- 상태 보고

**FSM 상태들**:
```python
class Step(IntEnum):
    IDLE          = 1  # 대기 중
    GO_TO_ARM     = 2  # 팔로 이동 (물건 집기 위해)
    PICK          = 3  # 물건 집기
    GO_TO_USER    = 4  # 사용자에게 이동
    WAIT_CONFIRM  = 5  # 수령 확인 대기
    GO_DOCK       = 6  # 충전소로 복귀
```

**상태 전이 다이어그램**:
```
        [주문 수신]
            ↓
    IDLE ──────→ GO_TO_ARM
                     ↓
                   PICK
                     ↓
               GO_TO_USER
                     ↓
              WAIT_CONFIRM
                     ↓
                 GO_DOCK
                     ↓
                   IDLE
```

### 🌉 4. Domain Bridge (도메인 브리지)
**역할**: 서로 다른 도메인 간의 통역사

**작동 원리**:
```yaml
# Domain Bridge 설정 예시
topics:
  - topic: /robot_1/user_cmd      # 토픽 이름
    type: std_msgs/msg/String     # 메시지 타입
    from_domain: 129               # 출발 도메인 (Fleet Manager)
    to_domain: 18                  # 도착 도메인 (Robot 1)
```

---

## 4. 데이터 흐름 따라가기

### 📊 주요 데이터 타입

#### 1. Task (작업)
```json
{
    "task_id": "12345",
    "task_type": "delivery",  // 배달 or 호출
    "resident_id": "9999",    // 주민 ID
    "items": [                // 배달할 물품들
        {
            "item_id": 1,
            "quantity": 2,
            "item_name": "생수"
        }
    ],
    "status": "pending",      // 대기|할당|진행중|완료
    "assigned_bot_id": null,  // 할당된 로봇 ID
    "created_at": "2025-01-07 10:00:00"
}
```

#### 2. Robot Command (로봇 명령)
```json
{
    "command": "order",        // 명령 타입
    "resident_id": "9999",     // 목적지 주민
    "task_type": "배달",       // 작업 유형
    "target_coordinates": [1.5, 2.3, 0.707],  // 목적지 좌표
    "task_id": "12345"
}
```

#### 3. Robot Status (로봇 상태)
```python
# 단순 문자열로 전송
"idle"              # 대기 중
"moving_to_arm"     # 팔로 이동 중
"picking"           # 물건 집는 중
"moving_to_user"    # 사용자에게 이동 중
"waiting_confirm"   # 수령 확인 대기
"returning"         # 복귀 중
```

### 🔄 ROS2 토픽 통신

#### Fleet 레벨 토픽 (Domain 129)
```bash
/fleet/task_request     # Web → Fleet Manager (새 작업)
/fleet/task_response    # Fleet Manager → Web (응답)
/fleet/robot_status     # Fleet Manager → Web (상태 브로드캐스트)
```

#### Robot 레벨 토픽 (각 로봇 도메인)
```bash
/robot_1/user_cmd       # Fleet Manager → Robot (명령)
/robot_1/status         # Robot → Fleet Manager (상태)
```

---

## 5. 실제 시나리오로 배우기

### 📱 시나리오 1: 주민이 생수를 주문하는 경우

**상황**: 9999호 주민이 웹/앱으로 생수 2병을 주문

#### Step 1: 주문 접수
```python
# 1. 사용자가 웹에서 주문 버튼 클릭
POST /api/order
{
    "resident_id": "9999",
    "items": [{"item_id": 1, "quantity": 2, "name": "생수"}]
}

# 2. Web Server가 DB에 작업 생성
INSERT INTO tasks (task_type, resident_id, status)
VALUES ('delivery', '9999', '대기')

# 3. Fleet Manager에게 알림
publish to /fleet/task_request:
{
    "task_id": "12345",
    "task_type": "delivery",
    "resident_id": "9999",
    "items": [...]
}
```

#### Step 2: 로봇 할당
```python
# Fleet Manager의 할당 로직
def handle_task_request(self, task):
    # 1. 가용 로봇 찾기
    available_robot = self.find_idle_robot()  # robot_1 선택됨
    
    # 2. DB 업데이트
    UPDATE tasks SET status='할당', assigned_bot_id=8
    WHERE task_id='12345'
    
    # 3. 로봇에게 명령 전송
    self.send_command_to_robot('robot_1', {
        'command': 'order',
        'resident_id': '9999',
        'target_coordinates': [1.5, 2.3, 0.707]  # 9999호 좌표
    })
```

#### Step 3: 로봇 동작
```python
# Robot FSM의 상태 변화
1. IDLE → GO_TO_ARM
   - 상태 전송: "moving_to_arm"
   - 동작: 물건 집는 위치로 이동

2. GO_TO_ARM → PICK
   - 상태 전송: "picking"
   - 동작: 로봇 팔로 물건 집기

3. PICK → GO_TO_USER
   - 상태 전송: "moving_to_user"
   - 동작: 9999호로 이동

4. GO_TO_USER → WAIT_CONFIRM
   - 상태 전송: "waiting_confirm"
   - 동작: 도착 알림, 확인 대기
   - DB 업데이트: status='수령대기'
```

#### Step 4: 수령 확인
```python
# 1. 주민이 웹에서 "수령 확인" 버튼 클릭
POST /api/confirm/12345

# 2. Fleet Manager가 로봇에게 전달
publish to /robot_1/user_cmd: "confirm"

# 3. 로봇이 복귀
WAIT_CONFIRM → GO_DOCK → IDLE

# 4. DB 업데이트
UPDATE tasks SET status='완료' WHERE task_id='12345'
```

### 🔔 시나리오 2: 로봇 호출 (물건 없이)

**상황**: 주민이 쓰레기 수거를 위해 로봇만 호출

```python
# 주요 차이점: 팔/집기 단계를 건너뜀
FSM 상태 변화:
IDLE → GO_TO_USER → WAIT_CONFIRM → GO_DOCK → IDLE
       (GO_TO_ARM과 PICK 단계 생략)
```

---

## 6. 코드 구조 깊이 들여다보기

### 📁 프로젝트 디렉토리 구조

```
e_multi/
├── 📄 run_fleet.py              # 시스템 전체 시작 스크립트
├── 📄 run_multirobot_fleet.py   # 멀티도메인 버전 시작 스크립트
├── 📄 fleet_config.yaml         # 플릿 설정 파일
├── 📄 config.py                 # 전역 설정 (DB 접속 정보 등)
│
├── 📁 fleet_manager/            # Fleet Manager 모듈
│   └── fleet_manager.py        # 메인 Fleet Manager 클래스
│
├── 📁 web/                      # 웹 서버 모듈
│   ├── __init__.py             # Flask 앱 생성
│   ├── fleet_client.py         # Fleet Manager와 통신
│   ├── task_db.py              # 작업 DB 관리
│   ├── db.py                   # DB 연결 헬퍼
│   ├── 📁 routes/              # API 엔드포인트
│   │   ├── orders.py           # 주문 API
│   │   ├── status.py           # 상태 API
│   │   └── auth.py             # 인증 API
│   ├── 📁 static/              # 정적 파일
│   │   ├── style.css           # 스타일시트
│   │   └── order.js            # 주문 페이지 JS
│   └── 📁 templates/           # HTML 템플릿
│       └── order.html          # 주문 페이지
│
├── 📁 mobile_robot/             # 로봇 관련 모듈
│   ├── 📁 robot_nodes/         # ROS2 노드
│   │   └── robot_node.py       # 로봇 노드 메인
│   ├── 📁 robot/               # 로봇 제어 로직
│   │   ├── fsm.py              # 유한 상태 기계
│   │   ├── nav2_waypoint_class.py  # 네비게이션
│   │   └── arm.py              # 로봇 팔 제어
│   └── robot_config.py         # 로봇 개별 설정
│
└── 📁 shared/                   # 공유 모듈
    └── fleet_msgs.py           # 메시지 정의
```

### 🔧 핵심 클래스 설계

#### FleetManager 클래스
```python
class FleetManager(Node):
    """중앙 플릿 관리자"""
    
    def __init__(self):
        # 로봇 상태 딕셔너리
        self.robots = {}  # {robot_id: RobotState}
        
        # ROS2 통신 설정
        self.task_request_sub = ...   # 작업 요청 구독
        self.task_response_pub = ...  # 작업 응답 발행
        self.robot_cmd_pubs = {}      # 로봇별 명령 발행자
        self.robot_status_subs = {}   # 로봇별 상태 구독자
        
        # 타이머 설정
        self.create_timer(2.0, self.check_and_assign_tasks)
        self.create_timer(30.0, self.check_task_timeouts)
    
    def handle_task_request(self, msg):
        """새 작업 요청 처리"""
        # 1. 작업 파싱
        # 2. 최적 로봇 선택
        # 3. 작업 할당
        # 4. 응답 전송
    
    def handle_robot_status(self, robot_id, status):
        """로봇 상태 업데이트 처리"""
        # 1. 내부 상태 업데이트
        # 2. DB 동기화
        # 3. 웹 서버에 브로드캐스트
```

#### DeliveryFSM 클래스
```python
class DeliveryFSM(Node):
    """로봇 배송 유한 상태 기계"""
    
    def __init__(self, robot_name, navigator):
        self.step = Step.IDLE
        self.waypoint_nav = navigator
        
        # ROS2 통신
        self.create_subscription(String, 'user_cmd', self.on_cmd, 10)
        self.stat_pub = self.create_publisher(String, 'status', 10)
        
        # 주기적 실행
        self.create_timer(0.5, self.loop)
    
    def on_cmd(self, msg):
        """명령 수신 처리"""
        if msg.data == "order":
            self.set_step(Step.GO_TO_ARM)
    
    def loop(self):
        """상태 기계 메인 루프"""
        match self.step:
            case Step.GO_TO_ARM:
                if self.waypoint_nav.is_arrived():
                    self.set_step(Step.PICK)
            case Step.PICK:
                if self.arm_pick_complete():
                    self.set_step(Step.GO_TO_USER)
            # ... 나머지 상태들
```

### 🔐 데이터베이스 스키마

```sql
-- 로봇 정보 테이블
CREATE TABLE hana_bots (
    hana_bot_id SERIAL PRIMARY KEY,
    name VARCHAR(50),
    status VARCHAR(20),
    current_location VARCHAR(100)
);

-- 작업 테이블
CREATE TABLE tasks (
    task_id SERIAL PRIMARY KEY,
    task_type VARCHAR(20),      -- 'delivery' or 'call'
    resident_id VARCHAR(20),
    status VARCHAR(20),          -- '대기', '할당', '진행중', '완료'
    assigned_bot_id INTEGER REFERENCES hana_bots(hana_bot_id),
    created_at TIMESTAMP DEFAULT CURRENT_TIMESTAMP,
    completed_at TIMESTAMP
);

-- 주민 정보 테이블
CREATE TABLE residents (
    resident_id VARCHAR(20) PRIMARY KEY,
    name VARCHAR(100),
    phone VARCHAR(20),
    address VARCHAR(200),
    service_station_coords FLOAT[]  -- [x, y, z] 좌표
);

-- 주문 상세 테이블
CREATE TABLE order_items (
    order_id SERIAL PRIMARY KEY,
    task_id INTEGER REFERENCES tasks(task_id),
    item_id INTEGER,
    quantity INTEGER,
    item_name VARCHAR(100)
);
```

---

## 7. 실습 해보기

### 🚀 Step 1: 시스템 시작하기

```bash
# 1. 프로젝트 디렉토리로 이동
cd /home/sang/dev/simple_gui_db/e_multi

# 2. 전체 시스템 시작
python run_multirobot_fleet.py

# 출력 예시:
# Starting Fleet Manager on domain 129...
# Starting Domain Bridge...
# Starting Robot 1 on domain 18...
# Starting Robot 2 on domain 19...
# Starting Web Server on port 8080...
# System ready!
```

### 🔍 Step 2: 시스템 모니터링

**터미널 1 - Fleet Manager 모니터링**:
```bash
export ROS_DOMAIN_ID=129
ros2 topic echo /fleet/robot_status
```

**터미널 2 - Robot 1 모니터링**:
```bash
export ROS_DOMAIN_ID=18
ros2 topic echo /robot_1/status
```

**터미널 3 - 데이터베이스 확인**:
```bash
python check_db.py
```

### 🎮 Step 3: 수동 테스트

**작업 생성 (API 호출)**:
```bash
curl -X POST http://localhost:8080/api/order \
  -H "Content-Type: application/json" \
  -d '{
    "resident_id": "9999",
    "items": [{"item_id": 1, "quantity": 2, "name": "생수"}]
  }'
```

**로봇 상태 확인**:
```bash
curl http://localhost:8080/robot_status/robot_1?resident_id=9999
```

**수령 확인**:
```bash
curl -X POST http://localhost:8080/api/confirm/12345
```

### 📝 Step 4: 코드 수정 실습

**실습 1: 새로운 로봇 상태 추가**
```python
# mobile_robot/robot/fsm.py에서
class Step(IntEnum):
    # ... 기존 상태들
    MAINTENANCE = auto()  # 새 상태 추가

# 상태 처리 로직 추가
def loop(self):
    # ...
    elif self.step == Step.MAINTENANCE:
        self.pub_status("maintenance")
        # 유지보수 로직
```

**실습 2: 새 API 엔드포인트 추가**
```python
# web/routes/orders.py에 추가
@app.route('/api/robot/<robot_id>/pause', methods=['POST'])
def pause_robot(robot_id):
    """로봇 일시정지 API"""
    # Fleet Manager에게 일시정지 명령 전송
    fleet_client.send_command(robot_id, "pause")
    return {'status': 'paused', 'robot_id': robot_id}
```

---

## 8. 자주 하는 질문

### ❓ Q1: ROS2와 일반 웹 서버를 왜 함께 사용하나요?

**A**: 각각의 강점을 활용하기 위해서입니다:
- **ROS2**: 실시간 로봇 제어, 센서 데이터 처리에 최적화
- **웹 서버**: 사용자 인터페이스, API 제공, DB 관리에 적합
- 두 시스템을 연결하여 최상의 솔루션 구현

### ❓ Q2: Domain Bridge는 왜 필요한가요?

**A**: 멀티도메인 환경에서 통신을 가능하게 합니다:
- 각 로봇이 독립된 도메인에서 작동 (간섭 방지)
- Fleet Manager가 중앙에서 모든 로봇과 통신
- Bridge가 도메인 간 메시지를 중계

### ❓ Q3: FSM(유한 상태 기계)을 사용하는 이유는?

**A**: 로봇 행동을 체계적으로 관리하기 위해서입니다:
- 명확한 상태 정의로 예측 가능한 동작
- 디버깅과 유지보수가 쉬움
- 복잡한 행동을 단순한 상태들로 분해

### ❓ Q4: 실제 배포시 고려사항은?

**A**: 실제 환경에서는 다음을 추가로 고려해야 합니다:
- **보안**: HTTPS, 인증 토큰, 암호화
- **확장성**: 로드 밸런싱, 캐싱
- **안정성**: 에러 처리, 자동 복구
- **모니터링**: 로그 수집, 성능 측정

### ❓ Q5: 이 시스템을 개선하려면?

**A**: 다음과 같은 기능을 추가할 수 있습니다:
- **스케줄링**: 예약 배송 기능
- **최적화**: 경로 최적화, 배터리 관리
- **AI 통합**: 수요 예측, 이상 감지
- **UI/UX**: 실시간 추적, 푸시 알림

---

## 🎯 다음 단계

### 추천 학습 경로

1. **ROS2 기초 학습**
   - ROS2 공식 튜토리얼
   - 토픽, 서비스, 액션 개념

2. **Flask 웹 개발**
   - REST API 설계
   - 데이터베이스 연동

3. **시스템 통합**
   - Docker 컨테이너화
   - CI/CD 파이프라인

4. **실제 로봇 적용**
   - 시뮬레이션 환경 구축
   - 실제 하드웨어 연동

### 유용한 리소스

- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [Flask Documentation](https://flask.palletsprojects.com/)
- [PostgreSQL Tutorial](https://www.postgresql.org/docs/current/tutorial.html)
- [Python Asyncio](https://docs.python.org/3/library/asyncio.html)

---

## 💬 마무리

이 시스템은 실제 서비스에 사용될 수 있는 수준의 로봇 플릿 관리 시스템입니다. 웹 기술과 로봇 기술을 결합한 좋은 학습 사례이며, 여러분이 이해한 내용을 바탕으로 자신만의 기능을 추가해보세요!

**질문이나 피드백은 언제든 환영합니다! 🚀**

---

*작성일: 2025년 1월 7일*
*작성자: E-Multi 개발팀*