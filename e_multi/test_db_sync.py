#!/usr/bin/env python3
"""DB 동기화 기능 테스트 스크립트"""

import sys
import os
sys.path.append(os.path.dirname(__file__))
sys.path.append(os.path.join(os.path.dirname(__file__), 'web'))

from web.db import query_db
import time

def test_db_connection():
    """데이터베이스 연결 테스트"""
    print("=== DB 연결 테스트 ===")
    try:
        with query_db() as cur:
            cur.execute("SELECT version()")
            version = cur.fetchone()[0]
            print(f"✅ DB 연결 성공: {version}")
            return True
    except Exception as e:
        print(f"❌ DB 연결 실패: {e}")
        return False

def test_robot_status_update(robot_id, hana_bot_id, status):
    """로봇 상태 업데이트 테스트"""
    print(f"\n=== 로봇 상태 업데이트 테스트: {robot_id} (ID:{hana_bot_id}) → {status} ===")
    
    # 상태 매핑
    status_map = {
        "idle": "대기중",
        "moving_to_arm": "작업중",
        "picking": "작업중",
        "waiting_confirm": "작업중",
        "returning_to_dock": "복귀중",
        "error": "오프라인"
    }
    db_status = status_map.get(status, "대기중")
    
    try:
        # 업데이트 전 상태 확인
        with query_db() as cur:
            cur.execute("SELECT status FROM hana_bots WHERE hana_bot_id = %s", (hana_bot_id,))
            old_status = cur.fetchone()
            print(f"이전 상태: {old_status[0] if old_status else 'NOT FOUND'}")
            
            # 상태 업데이트
            cur.execute("""
                UPDATE hana_bots 
                SET status = %s 
                WHERE hana_bot_id = %s
            """, (db_status, hana_bot_id))
            
            rows_affected = cur.rowcount
            print(f"영향받은 행: {rows_affected}")
            
            if rows_affected > 0:
                # 업데이트 후 상태 확인
                cur.execute("SELECT status FROM hana_bots WHERE hana_bot_id = %s", (hana_bot_id,))
                new_status = cur.fetchone()
                print(f"✅ 업데이트 성공: {status} → {db_status} (실제 DB: {new_status[0] if new_status else 'NOT FOUND'})")
                return True
            else:
                print(f"❌ 업데이트 실패: robot ID {hana_bot_id}가 hana_bots 테이블에 없음")
                return False
                
    except Exception as e:
        print(f"❌ DB 업데이트 실패: {e}")
        return False

def show_current_robots():
    """현재 로봇 상태 조회"""
    print("\n=== 현재 로봇 상태 ===")
    try:
        with query_db() as cur:
            cur.execute("""
                SELECT hana_bot_id, bot_name, status, battery 
                FROM hana_bots 
                ORDER BY hana_bot_id
            """)
            robots = cur.fetchall()
            
            for robot in robots:
                print(f"ID: {robot[0]}, Name: {robot[1]}, Status: {robot[2]}, Battery: {robot[3]}%")
                
    except Exception as e:
        print(f"❌ 로봇 조회 실패: {e}")

def test_fleet_manager_mapping():
    """Fleet Manager의 로봇 매핑 테스트"""
    print("\n=== Fleet Manager 로봇 매핑 테스트 ===")
    
    # fleet_config.yaml에서 매핑 정보 확인
    robot_name_to_id = {
        'robot_1': 8,
        'robot_2': 9
    }
    
    for robot_name, robot_db_id in robot_name_to_id.items():
        print(f"{robot_name} → DB ID: {robot_db_id}")
        
        # 해당 ID가 DB에 존재하는지 확인
        try:
            with query_db() as cur:
                cur.execute("SELECT bot_name, status FROM hana_bots WHERE hana_bot_id = %s", (robot_db_id,))
                result = cur.fetchone()
                if result:
                    print(f"  ✅ DB에 존재: {result[0]}, 현재 상태: {result[1]}")
                else:
                    print(f"  ❌ DB에 없음: ID {robot_db_id}")
        except Exception as e:
            print(f"  ❌ 조회 실패: {e}")

if __name__ == "__main__":
    print("🔍 DB 동기화 기능 진단 시작...")
    
    # 1. DB 연결 테스트
    if not test_db_connection():
        print("DB 연결 실패로 테스트 중단")
        exit(1)
    
    # 2. 현재 로봇 상태 표시
    show_current_robots()
    
    # 3. Fleet Manager 매핑 테스트
    test_fleet_manager_mapping()
    
    # 4. 로봇 상태 업데이트 테스트
    print("\n=== 상태 업데이트 시뮬레이션 ===")
    
    # robot_1 (ID: 8) 상태 변경 테스트
    test_robot_status_update("robot_1", 8, "moving_to_arm")
    time.sleep(1)
    
    # 다시 idle로 변경
    test_robot_status_update("robot_1", 8, "idle") 
    
    # 5. 최종 상태 확인
    show_current_robots()
    
    print("\n✅ 테스트 완료")