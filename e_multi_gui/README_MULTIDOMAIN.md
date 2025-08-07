# Multi-Domain Fleet Management System

ROS2 `domain_bridge` 패키지를 사용한 진정한 멀티로봇 분산 아키텍처입니다.

## 🏗️ 아키텍처 개요

### 도메인 할당
```
Domain 129: Central Fleet Management
├── Fleet Manager Node
├── Web Server (ROS Interface) 
├── Database Access
└── Domain Bridge Hub

Domain 18: Robot 1
├── Robot Node (robot_1)
├── FSM Controller
├── Navigation Stack
└── ROS2 Topics (isolated)

Domain 19: Robot 2  
├── Robot Node (robot_2)
├── FSM Controller
├── Navigation Stack
└── ROS2 Topics (isolated)
```

### 통신 흐름
```
Web Request → Fleet Manager (Domain 129) → Domain Bridge → Robot Nodes (Domain 18/19)
```

## 🚀 실행 방법

### 1. 멀티도메인 시스템 실행

#### 중앙 서버에서:
```bash
# 새로운 멀티도메인 런처 사용
python run_multirobot_fleet.py

# 또는 기존 방식 (domain_bridge 자동 시작)
python run_fleet.py
```

#### 각 로봇 머신에서:
```bash
# Robot 1 (Domain 18)
cd mobile_robot
./start_robot.sh robot_1

# Robot 2 (Domain 19) 
cd mobile_robot
./start_robot.sh robot_2
```

### 2. 수동 Domain Bridge 실행 (필요시)
```bash
# 별도 터미널에서 domain bridge 실행
./scripts/start_domain_bridge.sh
```

## 🌉 Domain Bridge 설정

### 자동 생성되는 브리지 토픽:
```yaml
# Robot 1 (Domain 18 ↔ Domain 129)
/robot_1/user_cmd      # Central → Robot
/robot_1/status        # Robot → Central
/robot_1/confirm_request   # Central → Robot  
/robot_1/confirm_response  # Robot → Central

# Robot 2 (Domain 19 ↔ Domain 129)
/robot_2/user_cmd      # Central → Robot
/robot_2/status        # Robot → Central
/robot_2/confirm_request   # Central → Robot
/robot_2/confirm_response  # Robot → Central
```

## 📋 필요 패키지

### ROS2 Domain Bridge 설치:
```bash
sudo apt install ros-humble-domain-bridge
```

### Python 의존성:
```bash
pip install pyyaml
```

## ⚙️ 설정 파일

### `fleet_config.yaml`
```yaml
central_domain: 129    # 중앙 서버 도메인
robots:
  robot_1:
    domain_id: 18      # Robot 1 도메인
    hana_bot_id: 8     # DB 매핑 ID
  robot_2: 
    domain_id: 19      # Robot 2 도메인
    hana_bot_id: 9     # DB 매핑 ID
domain_bridge:
  enable: true         # Domain Bridge 활성화
```

## 🔍 디버깅 명령어

### 도메인별 토픽 확인:
```bash
# 중앙 서버 (Domain 129)
export ROS_DOMAIN_ID=129
ros2 topic list

# Robot 1 (Domain 18)
export ROS_DOMAIN_ID=18  
ros2 topic list

# Robot 2 (Domain 19)
export ROS_DOMAIN_ID=19
ros2 topic list
```

### 통신 테스트:
```bash
# Domain Bridge를 통한 통신 테스트
export ROS_DOMAIN_ID=129
ros2 topic pub /robot_1/user_cmd std_msgs/msg/String "data: 'test'"

# Robot에서 상태 확인
export ROS_DOMAIN_ID=18
ros2 topic echo /robot_1/user_cmd
```

## 🎯 멀티도메인 아키텍처의 장점

### 1. **네트워크 격리**
- 각 로봇의 ROS 통신이 완전히 독립적
- 한 로봇의 토픽 플러딩이 다른 로봇에 영향 없음

### 2. **확장성**
- 새 로봇 추가 시 새 도메인만 할당
- Domain Bridge 설정 자동 생성

### 3. **장애 격리**  
- 한 로봇의 ROS 노드 크래시가 전체 시스템에 영향 없음
- 로봇별 독립적인 재시작 가능

### 4. **대역폭 최적화**
- 필요한 토픽만 선택적으로 브리지
- 네트워크 트래픽 최소화

## 🔧 기존 시스템과의 차이점

| 구분 | 기존 (단일 도메인) | 신규 (멀티 도메인) |
|------|-------------------|-------------------|
| 도메인 | 모든 노드 동일 도메인 | 로봇별 독립 도메인 |
| 통신 | 직접 ROS 토픽 통신 | Domain Bridge 경유 |
| 격리 | 네임스페이스만 분리 | 완전한 네트워크 격리 |
| 확장성 | 네트워크 트래픽 증가 | 선택적 브리지로 최적화 |
| 장애 대응 | 전체 영향 가능 | 로봇별 독립적 |

## 🚨 주의사항

### 1. **Domain Bridge 의존성**
- domain_bridge가 중단되면 로봇 통신 불가
- Fleet Manager가 자동으로 bridge를 관리함

### 2. **네트워크 설정**
- 모든 머신이 동일 네트워크에 있어야 함
- 방화벽에서 ROS2 포트(7400-7500) 허용 필요

### 3. **시간 동기화**
- 모든 머신의 시간이 동기화되어야 함
- `chrony` 또는 `ntp` 사용 권장

## 📊 성능 모니터링

### Domain Bridge 상태 확인:
```bash
# Bridge 프로세스 확인
ps aux | grep domain_bridge

# Bridge 로그 확인  
journalctl -f | grep domain_bridge
```

### 로봇 상태 API:
```bash
# 모든 로봇 상태
curl http://localhost:8080/robot_status

# 특정 로봇 상태  
curl http://localhost:8080/robot_status/robot_1
```