from flask import Blueprint, render_template, jsonify, request, redirect, url_for
import json

bp = Blueprint('main', __name__)

@bp.route('/')
def index():
    """메인 인덱스 - 대시보드로 리다이렉트"""
    return redirect(url_for('main.dashboard'))

@bp.route('/dashboard')
def dashboard():
    """메인 대시보드 페이지"""
    return render_template('main_dashboard.html')

@bp.route('/order')
def order():
    """배달 주문 페이지"""
    return render_template('order.html')

@bp.route('/robot_call')
def robot_call():
    """로봇 호출 페이지"""
    return render_template('robot_call.html')

@bp.route('/call_robot', methods=['POST'])
def call_robot():
    """로봇 호출 API"""
    try:
        data = request.get_json()
        resident_id = data.get('resident_id')
        call_type = data.get('call_type')
        
        if not resident_id or not call_type:
            return jsonify({
                'success': False,
                'message': '필수 정보가 누락되었습니다.'
            })
        
        # TODO: 실제 로봇 호출 로직 구현
        # 현재는 성공 응답만 반환
        
        return jsonify({
            'success': True,
            'message': f'로봇 호출이 요청되었습니다. 호출 유형: {call_type}',
            'call_type': call_type,
            'estimated_arrival': '3-5분'
        })
        
    except Exception as e:
        return jsonify({
            'success': False,
            'message': f'호출 처리 중 오류가 발생했습니다: {str(e)}'
        })

@bp.route('/api/user')
def get_user():
    """사용자 정보 API"""
    try:
        # URL 파라미터나 세션에서 resident_id 가져오기
        from flask import session
        from ..db import query_db
        
        resident_id = request.args.get('resident_id') or session.get('resident_id')
        
        if not resident_id:
            # 로그인하지 않은 경우 기본값
            return jsonify({
                'success': True,
                'user': {
                    'name': '게스트',
                    'room': '-',
                    'phone': '-',
                    'building': '-'
                }
            })
        
        # 데이터베이스에서 실제 사용자 정보 조회
        with query_db() as cur:
            cur.execute("""
                SELECT name, room_number, phone, unit_id 
                FROM residents 
                WHERE resident_id = %s
            """, (int(resident_id),))
            result = cur.fetchone()
            
            if result:
                return jsonify({
                    'success': True,
                    'user': {
                        'name': result[0] or f'주민{resident_id}',
                        'room': result[1] or f'{resident_id}호',
                        'phone': result[2] or '-',
                        'building': f'{result[3]}동' if result[3] else 'A동'
                    }
                })
            else:
                # 사용자가 DB에 없는 경우 기본값으로 생성
                return jsonify({
                    'success': True,
                    'user': {
                        'name': f'주민{resident_id}',
                        'room': f'{resident_id}호',
                        'phone': '-',
                        'building': 'A동'
                    }
                })
                
    except Exception as e:
        print(f"Error fetching user info: {e}")
        return jsonify({
            'success': True,
            'user': {
                'name': '사용자',
                'room': '-',
                'phone': '-',
                'building': '-'
            }
        })