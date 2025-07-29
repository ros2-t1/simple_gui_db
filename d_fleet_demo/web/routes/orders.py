from flask import Blueprint, request, jsonify, render_template
from ..data_access import decrement_stock
from ..fleet_client import get_fleet_client
from ..task_db import create_task
import threading
import time

bp = Blueprint("orders", __name__)

@bp.route("/order", methods=["GET"])
def order_page():
    return render_template("order.html")


@bp.route("/order", methods=["POST"])
def place_order():
    data  = request.get_json()
    rid   = data.get("resident_id")
    items = data.get("items")
    if not rid or not items:
        return jsonify({"error": "resident_id와 items 필요"}), 400

    try:
        # 1) 재고 차감
        decrement_stock(items)

        # 2) 데이터베이스에 작업 생성 (첫 번째 아이템만 사용, 추후 확장 가능)
        item_id = items[0].get('item_id') if items else None
        db_task_id = create_task('delivery', rid, item_id)

        # 3) Fleet Manager에게 작업 요청
        fleet_client = get_fleet_client()
        
        # Response handling
        response_data = {}
        response_event = threading.Event()
        
        def task_callback(response):
            response_data.update(response)
            response_event.set()
        
        # Send task to fleet manager with DB task_id
        task_id = fleet_client.send_delivery_task(rid, items, task_callback, db_task_id)
        
        # Wait for response (with timeout)
        if response_event.wait(timeout=10.0):
            if response_data.get('status') in ['assigned', 'in_progress', 'pending']:
                return jsonify({
                    "message": "주문 접수됨 (순서대로 처리됩니다)", 
                    "resident_id": rid,
                    "task_id": db_task_id,
                    "queue_status": response_data.get('message', '')
                })
            else:
                return jsonify({
                    "error": f"Task failed: {response_data.get('message', 'Unknown error')}"
                }), 500
        else:
            return jsonify({"error": "Fleet manager timeout"}), 500

    except Exception as e:
        return jsonify({"error": str(e)}), 500


@bp.route("/confirm", methods=["POST"])
def confirm():
    try:
        fleet_client = get_fleet_client()
        fleet_client.send_confirm_request()
        return jsonify({"message": "수령 확인됨"})
    except Exception as e:
        return jsonify({"error": str(e)}), 500
