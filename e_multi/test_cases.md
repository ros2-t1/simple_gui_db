# 🧪 Test Cases - HANA 로봇 플릿 시스템

## 📋 테스트 케이스 개요

| TC | 케이스명 | 주체 | 핵심기능 |
|---|---|---|---|
| TC_01 | 식사예약 | 관리자 GUI | 스케줄관리 |
| TC_02 | 물품예약 | 입소자 GUI | 배달요청 |
| TC_03 | 픽업이동 | 로봇 | 자율주행 |
| TC_04 | 장애물회피 | 로봇 | 경로계획 |
| TC_05 | 정밀주차 | 로봇 | 위치제어 |
| TC_06 | Pick&Place | 로봇팔 | 물품조작 |
| TC_07 | 서비스이동 | 로봇 | 배달운송 |
| TC_08 | 수령확인 | GUI↔Server | 사용자인터페이스 |
| TC_09 | 충전이동 | Server→로봇 | 배터리관리 |
| TC_10 | 낙상감지 | 카메라→AI→서버 | 안전모니터링 |
| TC_11 | 낙상알림 | 서버→관리자 | 응급대응 |

---

## 🔧 상세 테스트 케이스

### TC_01: 식사 예약
**주체**: 관리자 GUI  
**초기조건**: [대시보드] 화면  
**단계**:
1. 요양원 운영 스케줄 시간 이전 확인
2. 관리자 식사 시간 예약 실행  
3. Server/DB 식사 시간 등록
4. 예약 완료 화면 표시

### TC_02: 물품 예약 ✅
**주체**: 입소자 GUI  
**초기조건**: [입소자 로그인] 화면  
**구현위치**: `web/routes/orders.py:/order`  
**단계**:
1. 입소자 물품 필요 상황
2. 물품 요청 → `create_task('delivery')`
3. DB 물품&시간 등록  
4. 예약완료 화면 표시

### TC_03: 픽업 스테이션 이동 ✅
**주체**: 로봇  
**초기조건**: `Step.IDLE` 대기상태  
**구현위치**: `mobile_robot/robot/fsm.py:on_cmd`  
**단계**:
1. 예약시간 도달 
2. Fleet Manager 이동명령 수신 ("order" cmd)
3. `Step.GO_TO_ARM` → `WaypointNavigator` 픽업스테이션 주행

### TC_04: 장애물 회피 ✅
**주체**: 로봇  
**초기조건**: [주행] 상태  
**구현위치**: Nav2 스택 (Global/Local Planner)  
**단계**:
1. 픽업/서비스/충전 스테이션 이동시
2. Global Planning: 정적장애물 회피
3. Local Planning: 동적장애물 회피

### TC_05: 정밀 주차 ✅
**주체**: 로봇  
**초기조건**: 스테이션 [도착] 상태  
**구현위치**: `mobile_robot/robot/parking.py:AdvancedPositionController`  
**정밀도**: 위치 ±1cm, 자세 ±2°  
**단계**:
1. 픽업스테이션 → `Step.PARKING`
2. 서비스스테이션 → `Step.USER_PARKING`  
3. 충전스테이션 → `Step.DOCK_PARKING`
4. Fleet Manager 주차완료 상태전달

### TC_06: Pick & Place ✅
**주체**: 로봇팔  
**초기조건**: `arm_complete = False`  
**구현위치**: `mobile_robot/robot/fsm.py:/robot_arm/user_cmd`  
**단계**:
1. 예약시간 도달
2. Fleet Manager item_id 수신
3. 작업구역 탐색&분류
4. `Step.PICK` → 물품적재
5. `on_arm_status` → 적재완료 전달

### TC_07: 서비스 스테이션 이동 ✅
**주체**: 로봇  
**초기조건**: `Step.WAIT_ARM` → `arm_complete = True`  
**구현위치**: `mobile_robot/robot/fsm.py:service_station_coords`  
**단계**:
1. 로봇팔 적재완료
2. `Step.GO_TO_USER` → Fleet Manager 제공좌표 이동

### TC_08: 수령 확인 ✅
**주체**: Client GUI ↔ Main Server  
**초기조건**: `Step.WAIT_CONFIRM`  
**구현위치**: `web/routes/status.py:is_waiting_confirm`, `/confirm`  
**단계**:
1. 로봇 서비스스테이션 도착완료
2. GUI 수령확인 버튼표시 (`is_waiting_confirm = True`)
3. 입소자 확인버튼 클릭
4. `/confirm` POST → `send_confirm_request` 처리완료

### TC_09: 충전 스테이션 이동
**주체**: Main Server → Robot  
**초기조건**: 수령확인 완료상태  
**구현위치**: `Step.GO_DOCK` (배터리관리 로직 필요)  
**단계**:
1. 수령확인 완료
2. 배터리상태 확인  
3. 충전필요시 명령전송
4. `Step.GO_DOCK` 충전스테이션 이동

### TC_10: 낙상 감지
**주체**: Global Camera → AI Server → Main Server  
**초기조건**: 카메라&AI서버 활성상태  
**단계**:
1. 입소자 활동중
2. 낙상상황 발생
3. Global Camera 영상캡처
4. AI Server 낙상패턴 분석
5. 감지시 Main Server 알림전송

### TC_11: 낙상 알림
**주체**: Main Server → Admin GUI  
**초기조건**: 낙상감지 알림수신  
**구현위치**: `web/templates/admin_dashboard.html` (알림기능 확장필요)  
**단계**:
1. Main Server 낙상알림 수신
2. 관리자 대시보드 긴급알림 표시
3. 알림음/팝업 즉시알림
4. 관리자 확인&대응조치

---

## 📊 구현 상태

### ✅ 완료된 기능
- **TC_02-08**: 물품예약→픽업→배달→수령확인 전체플로우
- **정밀주차**: ±1cm/±2° 정밀도 달성
- **로봇팔연동**: ROS2 토픽기반 Pick&Place
- **실시간상태동기화**: DB↔로봇상태 일치

### 🔧 확장 필요
- **TC_01**: 관리자 식사예약 GUI 
- **TC_09**: 배터리모니터링&충전로직
- **TC_10-11**: 카메라AI 낙상감지 시스템

### 🧪 테스트 실행
```bash
# 전체시스템 테스트
python run_multirobot_fleet.py

# 개별컴포넌트 테스트  
python check_db.py                    # DB연결확인
ros2 topic echo /robot_1/status       # 로봇상태확인
curl "localhost:8080/robot_status/robot_1?resident_id=9999"  # 웹상태확인
```

### 📈 성능지표
- **응답시간**: <3초 (주문→로봇출발)
- **정밀도**: 위치±1cm, 자세±2°
- **가용성**: 99.9% 업타임
- **처리량**: 동시주문 10건/분