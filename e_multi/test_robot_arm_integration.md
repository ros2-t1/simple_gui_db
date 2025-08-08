# 로봇암 통합 테스트 가이드

## 구현 완료 사항

### 1. Fleet Manager 수정
- task 할당 시 `item_id`를 cmd_data에 포함하여 전달
- 파일: `fleet_manager/fleet_manager.py`

### 2. Robot FSM 수정  
- 새로운 상태 추가: `Step.WAIT_ARM`
- 로봇암 통신 토픽 추가:
  - Publisher: `/robot_arm/user_cmd` (Int32) - item_id 전송
  - Subscriber: `/robot_arm/status` (String) - 완료 신호 수신
- picking 상태에서 item_id를 로봇암에 전송하고 완료 대기
- 30초 타임아웃 처리
- 파일: `mobile_robot/robot/fsm.py`

### 3. 로봇암 시뮬레이터
- 테스트용 로봇암 시뮬레이터 작성
- 파일: `test_robot_arm_simulator.py`

## 테스트 방법

### 1. 로봇암 시뮬레이터 실행
```bash
# 별도 터미널에서
python test_robot_arm_simulator.py
```

### 2. 시스템 실행
```bash
# 메인 터미널에서
python run_fleet.py
# 또는
python run_multirobot_fleet.py
```

### 3. 주문 생성 및 테스트
1. 웹 브라우저에서 http://localhost:8080 접속
2. 관리자 로그인
3. 새 주문 생성 (item_id가 포함된 배달 작업)
4. 로그 확인:
   - Fleet Manager: `"item_id": X` 포함된 명령 전송
   - Robot FSM: `"Sent item_id X to robot arm"` 메시지
   - Robot Arm Simulator: `"Received command to pick item_id: X"` 메시지
   - Robot Arm Simulator: `"Sent 'complete' status"` 메시지
   - Robot FSM: `"Robot arm task complete, proceeding to user"` 메시지

### 4. 토픽 모니터링 (선택사항)
```bash
# item_id 전송 확인
ros2 topic echo /robot_arm/user_cmd

# 로봇암 상태 확인  
ros2 topic echo /robot_arm/status

# 로봇 상태 확인
ros2 topic echo /robot_1/status
```

## 동작 흐름

1. **주문 생성**: 웹에서 item_id 포함 주문 생성
2. **작업 할당**: Fleet Manager가 robot에 task 할당 (item_id 포함)
3. **로봇 이동**: Robot이 픽업 지점으로 이동 (GO_TO_ARM)
4. **픽업 시작**: 도착 후 PICK 상태로 전환
5. **로봇암 명령**: item_id를 `/robot_arm/user_cmd`로 발행
6. **대기 상태**: WAIT_ARM 상태로 전환하여 완료 대기
7. **완료 신호**: 로봇암이 `/robot_arm/status`에 "complete" 발행
8. **배달 진행**: GO_TO_USER 상태로 전환하여 고객에게 이동

## 주의사항

- item_id가 없는 경우 기존 동작(하드코딩된 "vitamin")으로 폴백
- 로봇암 응답 타임아웃: 30초
- 실제 로봇암이 없을 때는 시뮬레이터 사용

## 향후 개선사항

- 로봇암 에러 처리 추가
- 다양한 로봇암 상태 처리 (busy, error 등)
- 멀티 로봇암 지원
- 로봇암 작업 진행률 표시