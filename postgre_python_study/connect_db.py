import psycopg2

db_name = "hana_dev"
user = "postgres"
password = "qwer1234"

try:
    conn = psycopg2.connect(f"dbname={db_name} user={user} password={password}")
    print("Database connection established successfully.")
except psycopg2.Error as e:
    print(f"Error connecting to the database: {e}")
    exit(1)
else:
    conn.commit()
    conn.close()