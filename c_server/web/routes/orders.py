from flask import Blueprint, request, jsonify, render_template
from std_msgs.msg import String
from ..data_access import decrement_stock                 # 🔹
import robot                                              # ros_node 공유

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

        # 2) 로봇에게 작업 지시
        robot.ros_node.cmd_pub.publish(String(data="order"))

        # 3) 응답
        return jsonify({"message": "주문 처리됨", "resident_id": rid})

    except Exception as e:
        return jsonify({"error": str(e)}), 500


@bp.route("/confirm", methods=["POST"])
def confirm():
    robot.ros_node.cmd_pub.publish(String(data="confirm"))
    return jsonify({"message": "수령 확인됨"})
