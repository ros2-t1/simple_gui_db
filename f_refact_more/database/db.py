from contextlib import contextmanager
import psycopg2
from psycopg2 import pool
from psycopg2.extras import DictCursor
import config as cfg

db_pool = psycopg2.pool.SimpleConnectionPool(1, 20, **cfg.DB_DSN)

@contextmanager
def get_conn():
    conn = db_pool.getconn()
    try:
        yield conn
    finally:
        db_pool.putconn(conn)

@contextmanager
def get_cur(conn):
    cur = conn.cursor()
    try:
        yield cur
    finally:
        cur.close()

@contextmanager
def query_db():
    """SELECT 쿼리용 컨텍스트 매니저"""
    with get_conn() as conn, get_cur(conn) as cur:
        yield cur

@contextmanager
def update_db():
    """INSERT, UPDATE 등 데이터 변경용 컨텍스트 매니저 (자동 커밋/롤백)"""
    with get_conn() as conn, get_cur(conn) as cur:
        try:
            yield cur
            conn.commit()
        except Exception:
            conn.rollback()
            raise

@contextmanager
def query_db_dict():
    """SELECT 쿼리용 컨텍스트 매니저 (DictCursor)"""
    with get_conn() as conn:
        cur = conn.cursor(cursor_factory=DictCursor)
        try:
            yield cur
        finally:
            cur.close()

@contextmanager
def update_db_dict():
    """INSERT, UPDATE 등 데이터 변경용 컨텍스트 매니저 (DictCursor, 자동 커밋/롤백)"""
    with get_conn() as conn:
        cur = conn.cursor(cursor_factory=DictCursor)
        try:
            yield cur
            conn.commit()
        except Exception:
            conn.rollback()
            raise
        finally:
            cur.close()
