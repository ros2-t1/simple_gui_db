from flask import Flask, request, jsonify, render_template
import psycopg2
from psycopg2 import pool
from contextlib import contextmanager
import os

app = Flask(__name__, template_folder="./templates", static_folder='./static')

# 연결 풀 설정
db_pool = psycopg2.pool.SimpleConnectionPool(
    1, 20,
    dbname="hana_db",
    user="postgres",
    password="qwer1234",
    host="20.249.209.1",
    port="5432"
)

def get_client_ip():
    if request.environ.get('HTTP_X_FORWARDED_FOR'):
        return request.environ['HTTP_X_FORWARDED_FOR'].split(',')[0]
    return request.environ['REMOTE_ADDR']

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

@app.route("/")
@app.route("/login", methods=["GET"])
def index():
    return render_template("login.html")

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

# --- 주문 기능 추가 ---
@app.route("/order", methods=["GET"])
def order_page():
    return render_template("order.html")

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

@app.route("/order", methods=["POST"])
def place_order():
    data = request.get_json()
    resident_id = data.get("resident_id")
    items = data.get("items")

    if not resident_id or not items:
        return jsonify({"error": "주문 정보가 올바르지 않습니다."}), 400

    try:
        with get_db_connection() as conn:
            with get_db_cursor(conn) as cursor:
                for item in items:
                    # 재고 업데이트
                    cursor.execute(
                        "UPDATE items SET item_quantity = item_quantity - %s WHERE item_id = %s",
                        (item["quantity"], item["id"])
                    )
                conn.commit()
        return jsonify({"message": "주문이 성공적으로 처리되었습니다."})
    except psycopg2.Error as e:
        conn.rollback()
        return jsonify({"error": f"데이터베이스 오류: {str(e)}"}), 500


if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080, debug=True)