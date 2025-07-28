# flask_ros2_fsm.py

import time
import threading
from flask import Flask, request, jsonify, render_template
import psycopg2
from psycopg2 import pool
from contextlib import contextmanager

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

from nav2_waypoint_class import WaypointNavigator

# ───────── ROS FSM 정의 ─────────
pickup_station = [0.33, -0.33, 0.707]
service_station_1 = [0.1, 0.78, -0.707]
charging_station = [0.0, 0.0, 1.0]

STEP_IDLE = 0
STEP_GO_TO_ARM = 1
STEP_PICK = 2
STEP_GO_TO_USER = 3
STEP_WAIT_CONFIRM = 4
STEP_GO_DOCK = 5

class DeliveryFSM(Node):
    def __init__(self, waypoint_nav):
        super().__init__('delivery_fsm')
        self.step = STEP_IDLE
        self.create_subscription(String, 'user_cmd', self.on_cmd, 10)
        self.publisher = self.create_publisher(String, 'user_cmd', 10)
        self.status_pub = self.create_publisher(String, 'status', 10)
        self.create_timer(0.5, self.loop)
        self.waypoint_nav = waypoint_nav

    def on_cmd(self, msg):
        if msg.data == 'order' and self.step == STEP_IDLE:
            self.set_step(STEP_GO_TO_ARM)
        elif msg.data == 'confirm' and self.step == STEP_WAIT_CONFIRM:
            self.set_step(STEP_GO_DOCK)

    def set_step(self, next_step):
        self.step = next_step
        self.waypoint_nav.pinky_nav2_state = "None"
        if self.step == STEP_IDLE:
            self.pub_status('idle')
        elif self.step == STEP_GO_TO_ARM:
            self.pub_status('moving_to_arm')
            self.nav_to(pickup_station)
        elif self.step == STEP_PICK:
            self.pub_status('picking')
            arm_pick('vitamin')
            arm_place_on_robot()
            self.set_step(STEP_GO_TO_USER)
        elif self.step == STEP_GO_TO_USER:
            self.pub_status('moving_to_user')
            self.nav_to(service_station_1)
        elif self.step == STEP_WAIT_CONFIRM:
            self.pub_status('waiting_confirm')
        elif self.step == STEP_GO_DOCK:
            self.pub_status('returning_to_dock')
            self.nav_to(charging_station)

    def loop(self):
        if self.step == STEP_GO_TO_ARM and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(STEP_PICK)
        elif self.step == STEP_GO_TO_USER and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(STEP_WAIT_CONFIRM)
        elif self.step == STEP_GO_DOCK and self.waypoint_nav.pinky_nav2_state == "Done":
            self.set_step(STEP_IDLE)

    def nav_to(self, where):
        self.waypoint_nav.send_goal(where)

    def pub_status(self, text):
        self.status_pub.publish(String(data=text))
        self.get_logger().info(f'[STATUS] {text}')

def arm_pick(item):
    print(f'[ARM] pick {item}')
    time.sleep(1)

def arm_place_on_robot():
    print('[ARM] place on robot')
    time.sleep(1)

# ───────── Flask + DB 정의 ─────────
app = Flask(__name__, template_folder="./templates", static_folder='./static')
ros_node = None

db_pool = psycopg2.pool.SimpleConnectionPool(
    1, 20,
    dbname="hana_db",
    user="postgres",
    password="qwer1234",
    host="20.249.209.1",
    port="5432"
)

@contextmanager
def get_db_connection():
    conn = db_pool.getconn()
    try:
        yield conn
    finally:
        db_pool.putconn(conn)

@contextmanager
def get_db_cursor(conn):
    cursor = conn.cursor()
    try:
        yield cursor
    finally:
        cursor.close()

def get_client_ip():
    if request.environ.get('HTTP_X_FORWARDED_FOR'):
        return request.environ['HTTP_X_FORWARDED_FOR'].split(',')[0]
    return request.environ['REMOTE_ADDR']

@app.route("/")
@app.route("/login", methods=["GET"])
def index():
    return render_template("login.html")
@app.route("/order", methods=["GET"])
def order_page():
    return render_template("order.html")


@app.route("/login", methods=["POST"])
def login():
    data = request.get_json()
    login_id = data.get("login_id")
    password = data.get("password")

    if not login_id or not password:
        return jsonify({"error": "login_id와 password가 필요합니다"}), 400

    try:
        with get_db_connection() as conn:
            with get_db_cursor(conn) as cursor:
                cursor.execute(
                    """
                    SELECT resident_id, name FROM residents
                    WHERE login_id = %s
                    AND password = crypt(%s, password)
                    """,
                    (login_id, password)
                )
                user = cursor.fetchone()

                success = bool(user)
                resident_id = user[0] if success else None

                cursor.execute(
                    "INSERT INTO login_logs (resident_id, success, client_ip) VALUES (%s, %s, %s)",
                    (resident_id, success, get_client_ip())
                )
                conn.commit()

        if success:
            return jsonify({"resident_id": user[0], "name": user[1] })
        else:
            return jsonify({"error": "로그인 실패"}), 401

    except psycopg2.Error as e:
        return jsonify({"error": f"데이터베이스 오류: {str(e)}"}), 500

@app.route("/order", methods=["POST"])
def place_order():
    data = request.get_json()
    resident_id = data.get("resident_id")
    items = data.get("items")

    if not resident_id or not items:
        return jsonify({"error": "resident_id와 items 필요"}), 400

    try:
        with get_db_connection() as conn:
            with get_db_cursor(conn) as cursor:
                for item in items:
                    cursor.execute(
                        "UPDATE items SET item_quantity = item_quantity - %s WHERE item_id = %s",
                        (item["quantity"], item["id"])
                    )
                conn.commit()

        ros_node.publisher.publish(String(data='order'))
        return jsonify({"message": "주문 처리됨", "resident_id" : resident_id}), 200

    except Exception as e:
        return jsonify({"error": str(e)}), 500
    
@app.route("/items", methods=["GET"])
def get_items():
    try:
        with get_db_connection() as conn:
            with get_db_cursor(conn) as cursor:
                cursor.execute("SELECT item_id, item_type, item_quantity FROM items")
                items = [{"id": row[0], "type": row[1], "quantity": row[2]} for row in cursor.fetchall()]
                return jsonify(items)
    except psycopg2.Error as e:
        return jsonify({"error": f"데이터베이스 오류: {str(e)}"}), 500
    
@app.route("/confirm", methods=["POST"])
def confirm():
    ros_node.publisher.publish(String(data='confirm'))
    return jsonify({"message": "수령 확인됨"})

    
# ───────── 실행 ─────────
def main():
    global ros_node
    rclpy.init()
    waypoint_nav = WaypointNavigator()
    fsm = DeliveryFSM(waypoint_nav)
    ros_node = fsm

    executor = MultiThreadedExecutor()
    executor.add_node(waypoint_nav)
    executor.add_node(fsm)

    threading.Thread(target=executor.spin, daemon=True).start()
    app.run(host="0.0.0.0", port=8080, debug=True)

if __name__ == "__main__":
    main()
