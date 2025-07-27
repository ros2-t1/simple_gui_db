from flask import Flask, request, jsonify
import psycopg2
from psycopg2 import pool
from contextlib import contextmanager
import os

app = Flask(__name__)

# 연결 풀 설정
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

@app.route("/login", methods=["POST"])
def login():
    data = request.get_json()
    login_id = data.get("login_id")
    password = data.get("password")

    print("입력값:", login_id, password)  # 추가

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
                print("쿼리 결과:", user)  # 추가

                success = bool(user)
                resident_id = user[0] if success else None

                cursor.execute(
                    "INSERT INTO login_logs (resident_id, success, client_ip) VALUES (%s, %s, %s)",
                    (resident_id, success, request.remote_addr)
                )
                # login_log = cursor.fetchall()
                # print("로그인 로그:", login_log)
                conn.commit()

        if success:
            return jsonify({"resident_id": user[0], "name": user[1] })
        else:
            return jsonify({"error": "로그인 실패"}), 401

    except psycopg2.Error as e:
        print("DB 오류:", e)  # 추가
        return jsonify({"error": f"데이터베이스 오류: {str(e)}"}), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=8080)