#!/usr/bin/env python3
# Quick script to check database tables
import psycopg2
from psycopg2.extras import DictCursor
import config as cfg

def check_hana_bots():
    print("=== Checking hana_bots table ===")
    with psycopg2.connect(**cfg.DB_DSN) as conn:
        with conn.cursor(cursor_factory=DictCursor) as cur:
            # Check existing data
            cur.execute("SELECT * FROM hana_bots")
            rows = cur.fetchall()
            print(f"Found {len(rows)} robots:")
            for row in rows:
                print(f"  - {dict(row)}")

def check_tasks():
    print("\n=== Checking recent tasks ===")
    with psycopg2.connect(**cfg.DB_DSN) as conn:
        with conn.cursor(cursor_factory=DictCursor) as cur:
            cur.execute("""
                SELECT task_id, task_type, status, assigned_bot_id, created_at
                FROM tasks 
                ORDER BY created_at DESC 
                LIMIT 5
            """)
            rows = cur.fetchall()
            print(f"Recent tasks:")
            for row in rows:
                print(f"  - {dict(row)}")

if __name__ == "__main__":
    try:
        check_hana_bots()
        check_tasks()
    except Exception as e:
        print(f"Error: {e}")