#!/usr/bin/env python3
"""
Test script to check actual location_name values in database
"""

import psycopg2
import sys
sys.path.append('.')
from config import DB_DSN

def query_db():
    return psycopg2.connect(**DB_DSN).cursor()

print("Checking locations table...")
print("=" * 50)

conn = psycopg2.connect(**DB_DSN)
cur = conn.cursor()
try:
    # Check all locations
    cur.execute("""
        SELECT location_id, location_type, location_name, coordinates 
        FROM locations 
        ORDER BY location_id
    """)
    locations = cur.fetchall()
    
    print("\nAll locations in DB:")
    for loc in locations:
        print(f"  ID: {loc[0]}, Type: {loc[1]}, Name: {loc[2]}, Coords: {loc[3]}")
    
    # Check residents and their service stations
    print("\n" + "=" * 50)
    print("Residents and their service stations:")
    cur.execute("""
        SELECT r.resident_id, r.name, r.service_station_id, l.location_name
        FROM residents r
        LEFT JOIN locations l ON r.service_station_id = l.location_id
        ORDER BY r.resident_id
        LIMIT 10
    """)
    residents = cur.fetchall()
    
    for res in residents:
        print(f"  Resident {res[0]}: {res[1]} -> Station ID: {res[2]}, Name: {res[3]}")
    
    # Check unique location names that are service stations
    print("\n" + "=" * 50)
    print("Unique service station names:")
    cur.execute("""
        SELECT DISTINCT l.location_name 
        FROM locations l
        WHERE l.location_type = '서비스 스테이션'
        ORDER BY l.location_name
    """)
    stations = cur.fetchall()
    
    for station in stations:
        print(f"  - {station[0]}")
finally:
    cur.close()
    conn.close()