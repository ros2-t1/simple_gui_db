# Multi-Robot Fleet Management System - Interface Tables

## 1. HTTP REST API Specification Table

### 1.1 Authentication & Session Management

| API명 | HTTP메서드 | URL | 요청 데이터 구조 | 응답 데이터 구조 | 비고 |
|------|-----------|-----|-----------------|-----------------|-----|
| 로그인 | POST | `/login` | `{username: string, password: string}` | `{success: bool, resident_id: int, name: string, message: string}` | bcrypt 해시 검증 |
| 로그인 페이지 | GET | `/login` | - | HTML | 로그인 폼 렌더링 |
| 홈페이지 | GET | `/` | - | HTML | 로그인 리다이렉트 |

### 1.2 Order & Task Management

| API명 | HTTP메서드 | URL | 요청 데이터 구조 | 응답 데이터 구조 | 비고 |
|------|-----------|-----|-----------------|-----------------|-----|
| 주문 페이지 | GET | `/order` | - | HTML | 주문 폼 렌더링 |
| 배달 주문 | POST | `/order` | `{item_type: string, quantity: int}` | `{success: bool, task_id: string, message: string}` | 재고 감소 처리 |
| 로봇 호출 | POST | `/call` | `{reason: string}` | `{success: bool, task_id: string, message: string}` | 호출 작업 생성 |
| 작업 확인 | POST | `/confirm` | `{robot_id: string}` | `{success: bool, message: string}` | 수령 확인 처리 |

### 1.3 Status & Monitoring

| API명 | HTTP메서드 | URL | 요청 데이터 구조 | 응답 데이터 구조 | 비고 |
|------|-----------|-----|-----------------|-----------------|-----|
| 전체 로봇 상태 | GET | `/robot_status` | - | `{robot_1: {status, task_id, battery, last_update}, ...}` | 캐시된 상태 반환 |
| 특정 로봇 상태 | GET | `/robot_status/{robot_id}` | `?resident_id=int` | `{robot_id, status, task_id, is_for_resident, eta_seconds}` | 실시간 상태 조회 |
| 캐시 강제 갱신 | POST | `/force_cache_refresh` | - | `{success: bool, message: string}` | 상태 캐시 재초기화 |
| 재고 목록 | GET | `/items` | - | `[{item_id, item_type, quantity}, ...]` | 현재 재고 상태 |

### 1.4 Fleet Management API

| API명 | HTTP메서드 | URL | 요청 데이터 구조 | 응답 데이터 구조 | 비고 |
|------|-----------|-----|-----------------|-----------------|-----|
| 플릿 상태 | GET | `/api/fleet/status` | - | `{robots: {total, idle, busy}, tasks: {pending, in_progress}, system_health}` | 플릿 개요 |
| 플릿 분석 | GET | `/api/fleet/analytics` | - | `{performance: {avg_delivery_time, success_rate}, utilization: {...}}` | 성능 메트릭 |
| 플릿 제어 | POST | `/api/fleet/control` | `{command: string, target: string}` | `{success: bool, message: string}` | pause/resume/reset |

### 1.5 Admin API

| API명 | HTTP메서드 | URL | 요청 데이터 구조 | 응답 데이터 구조 | 비고 |
|------|-----------|-----|-----------------|-----------------|-----|
| 로봇 관리 | GET | `/api/admin/robots` | - | `[{robot_id, status, battery, location, current_task}, ...]` | 관리자 전용 |
| 작업 관리 | GET | `/api/admin/tasks` | `?status=string&date=string&robot_id=string` | `[{task_id, type, status, created_at, assigned_bot}, ...]` | 필터링 지원 |
| 사용자 관리 | GET | `/api/admin/users` | - | `[{resident_id, name, room_id, task_count, last_order}, ...]` | 통계 포함 |
| 재고 관리 | GET | `/api/admin/items` | - | `[{item_id, type, quantity, last_updated, history}, ...]` | 이력 포함 |

### 1.6 Camera Streaming

| API명 | HTTP메서드 | URL | 요청 데이터 구조 | 응답 데이터 구조 | 비고 |
|------|-----------|-----|-----------------|-----------------|-----|
| 카메라 목록 | GET | `/camera/list` | - | `{cameras: [{id, status, fps, resolution}, ...]}` | 가용 카메라 목록 |
| 비디오 스트림 | GET | `/camera/stream/{camera_id}` | - | MJPEG Stream | Multipart 스트림 |
| 카메라 시작 | POST | `/camera/start/{camera_id}` | - | `{success: bool, message: string}` | 스트림 시작 |
| 카메라 중지 | POST | `/camera/stop/{camera_id}` | - | `{success: bool, message: string}` | 스트림 중지 |
| 카메라 상태 | GET | `/camera/status` | - | `[{camera_id, is_streaming, fps, viewers}, ...]` | 전체 상태 |

## 2. ROS2 Topic Interface Specification Table

### 2.1 Fleet Management Topics

| 토픽명 | 메시지 타입 | 발행자 | 구독자 | 데이터 구조 | 비고 |
|-------|------------|-------|-------|------------|-----|
| `/fleet/task_request` | std_msgs/String | Web Server | Fleet Manager | `{task_id, db_task_id, task_type, requester_resident_id, item_id}` | 작업 요청 |
| `/fleet/task_response` | std_msgs/String | Fleet Manager | Web Server | `{task_id, status, robot_id, message}` | 작업 응답 |
| `/fleet/confirm_request` | std_msgs/String | Web Server | Fleet Manager | `"confirm"` or `"robot_id"` | 확인 요청 |
| `/fleet/robot_status` | std_msgs/String | Fleet Manager | Web Server | `{robot_id, status, current_task_id, timestamp}` | 상태 브로드캐스트 |

### 2.2 Robot Control Topics (Per Robot)

| 토픽명 | 메시지 타입 | 발행자 | 구독자 | 데이터 구조 | 비고 |
|-------|------------|-------|-------|------------|-----|
| `/{robot_id}/user_cmd` | std_msgs/String | Fleet Manager | Robot FSM | `{command, resident_id, task_type, item_id, target_coordinates}` | 로봇 명령 |
| `/{robot_id}/status` | std_msgs/String | Robot FSM | Fleet Manager | `"idle"/"moving_to_arm"/"picking"/"moving_to_user"/"waiting_confirm"/"returning_to_dock"/"navigation_failed"` | 로봇 상태 |

### 2.3 Robot Arm Interface Topics

| 토픽명 | 메시지 타입 | 발행자 | 구독자 | 데이터 구조 | 비고 |
|-------|------------|-------|-------|------------|-----|
| `/robot_arm/user_cmd` | std_msgs/Int32 | Robot FSM | Robot Arm | `item_id (integer)` | 픽업 명령 |
| `/robot_arm/status` | std_msgs/String | Robot Arm | Robot FSM | `"complete"/"error"/"busy"` | 작업 상태 |

## 3. Domain Bridge Configuration Table

### 3.1 Domain Assignment

| 컴포넌트 | Domain ID | 역할 | 네트워크 위치 | 비고 |
|---------|----------|-----|--------------|-----|
| Central Server | 129 | Fleet Manager, Web Server | 중앙 서버 | 메인 도메인 |
| Robot 1 | 18 | Mobile Robot FSM | 로봇 1 머신 | hana_bot_id: 8 |
| Robot 2 | 19 | Mobile Robot FSM | 로봇 2 머신 | hana_bot_id: 9 |
| Robot Arm | 14 | Arm Controller | 픽업 스테이션 | 공유 리소스 |

### 3.2 Bridge Topic Routing

| 토픽 패턴 | Source Domain | Target Domain | 방향 | 비고 |
|----------|--------------|---------------|-----|-----|
| `robot_1/user_cmd` | 129 | 18 | → | 로봇1 명령 |
| `robot_1/status` | 18 | 129 | ← | 로봇1 상태 |
| `robot_2/user_cmd` | 129 | 19 | → | 로봇2 명령 |
| `robot_2/status` | 19 | 129 | ← | 로봇2 상태 |
| `robot_arm/user_cmd` | 18, 19 | 14 | → | 로봇암 명령 |
| `robot_arm/status` | 14 | 18, 19, 129 | ← | 로봇암 상태 |

## 4. System State Tables

### 4.1 Task Status Transitions

| 현재 상태 | 다음 상태 | 전환 조건 | 트리거 | 비고 |
|----------|----------|----------|--------|-----|
| 대기 | 할당 | 로봇 가용 | Fleet Manager | 자동 할당 |
| 할당 | 집기중 | 로봇 도착 | Robot FSM | 픽업 위치 도착 |
| 집기중 | 이동중 | 픽업 완료 | Robot Arm | 아이템 적재 완료 |
| 이동중 | 수령대기 | 목적지 도착 | Robot FSM | 사용자 위치 도착 |
| 수령대기 | 완료 | 확인 버튼 | User/Web | 수령 확인 |
| 수령대기 | 실패 | 타임아웃 | Fleet Manager | 10분 초과 |

### 4.2 Robot FSM States

| 상태명 | 진입 조건 | 종료 조건 | 다음 상태 | 비고 |
|--------|----------|----------|----------|-----|
| IDLE | 초기/작업완료 | 새 명령 수신 | GO_TO_ARM/GO_TO_USER | 대기 상태 |
| GO_TO_ARM | 배달 명령 | 픽업 위치 도착 | PICK | 이동 중 |
| PICK | 픽업 위치 도착 | 로봇암 명령 전송 | WAIT_ARM | 픽업 시작 |
| WAIT_ARM | 로봇암 명령 전송 | 완료 신호/타임아웃 | GO_TO_USER | 로봇암 대기 |
| GO_TO_USER | 픽업 완료 | 사용자 위치 도착 | WAIT_CONFIRM | 배달 중 |
| WAIT_CONFIRM | 사용자 도착 | 확인 수신 | GO_DOCK | 수령 대기 |
| GO_DOCK | 확인 완료 | 충전소 도착 | IDLE | 복귀 중 |

## 5. Error Response Table

| 에러 코드 | HTTP 상태 | 설명 | 발생 조건 | 대응 방법 |
|----------|----------|-----|----------|----------|
| AUTH_REQUIRED | 401 | 인증 필요 | 세션 없음 | 로그인 페이지로 |
| AUTH_FAILED | 401 | 인증 실패 | 잘못된 자격증명 | 재시도 |
| INVALID_REQUEST | 400 | 잘못된 요청 | 파라미터 오류 | 입력값 검증 |
| RESOURCE_NOT_FOUND | 404 | 리소스 없음 | 존재하지 않는 ID | ID 확인 |
| ROBOT_UNAVAILABLE | 503 | 로봇 없음 | 모든 로봇 사용중 | 대기 또는 재시도 |
| TASK_FAILED | 500 | 작업 실패 | 실행 중 오류 | 로그 확인 |
| SYSTEM_ERROR | 500 | 시스템 오류 | 내부 오류 | 관리자 문의 |

## 6. Performance Requirements Table

| 메트릭 | 목표값 | 측정 방법 | 임계값 | 비고 |
|--------|--------|----------|--------|-----|
| API 응답 시간 | < 200ms | 95 percentile | 500ms | 쿼리 최적화 필요 |
| 로봇 상태 업데이트 | < 100ms | End-to-end latency | 200ms | 실시간성 보장 |
| 작업 할당 시간 | < 2s | Request to dispatch | 5s | 큐 처리 포함 |
| 카메라 스트림 | 30 FPS | Frame rate | 15 FPS | 네트워크 의존 |
| 시스템 가용성 | 99.9% | Uptime monitoring | 99% | 월간 측정 |
| 동시 사용자 | 100+ | Concurrent sessions | 50 | 부하 분산 필요 |
| 로봇 대수 | 10+ | Active robots | 5 | 도메인 확장 |