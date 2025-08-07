# Domain Bridge 기반 멀티로봇 아키텍처

## 개요
ROS2 domain_bridge를 사용하여 각 로봇을 별도 도메인에서 실행하고, 중앙 Fleet Manager와 통신하는 구조입니다.

## 아키텍처 설계

### 도메인 할당 전략
```
Domain 0: Central Fleet Management
├── Fleet Manager Node
├── Web Server (ROS Interface)
└── Domain Bridge Hub

Domain 10: Robot 1  
├── Robot Node (robot_1)
├── FSM Controller
├── Navigation Stack
└── Domain Bridge Client

Domain 20: Robot 2
├── Robot Node (robot_2) 
├── FSM Controller
├── Navigation Stack
└── Domain Bridge Client
```

### 통신 흐름
```
Web Server (Domain 0) → Fleet Manager (Domain 0) → Domain Bridge → Robot Nodes (Domain 10,20)
```

### 장점
1. **네트워크 격리**: 각 로봇의 ROS 통신이 독립적
2. **확장성**: 새 로봇 추가 시 새 도메인만 할당
3. **장애 격리**: 한 로봇의 문제가 다른 로봇에 영향 없음
4. **대역폭 최적화**: 필요한 토픽만 브리지를 통해 전달

### 필요한 토픽 브리지
```yaml
# Fleet Manager → Robot
/robot_X/user_cmd: String
/robot_X/confirm_request: String

# Robot → Fleet Manager  
/robot_X/status: String
/robot_X/confirm_response: String
```

## 구현 단계

### 1. Domain Bridge 설정
- 각 로봇 도메인과 중앙 도메인 간 브리지 설정
- 필요한 토픽만 선택적으로 브리지

### 2. Fleet Manager 수정
- 멀티 도메인 로봇 인식 및 관리
- 도메인별 토픽 네이밍 처리

### 3. Robot Node 수정  
- 독립적인 도메인 환경 변수 설정
- 브리지된 토픽을 통한 통신

### 4. 배포 스크립트 업데이트
- 로봇별 도메인 ID 자동 할당
- Domain Bridge 자동 시작/중지

## 설정 파일 구조
```
fleet_config.yaml:
  central_domain: 0
  robots:
    robot_1: 
      domain_id: 10
      bridge_topics: [/robot_1/user_cmd, /robot_1/status]
    robot_2:
      domain_id: 20  
      bridge_topics: [/robot_2/user_cmd, /robot_2/status]
```