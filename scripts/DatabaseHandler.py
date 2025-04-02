import sqlite3
import threading

class DatabaseHandler:
    def __init__(self, db_path):
        self.db_path = db_path
        self.lock = threading.Lock()  # Crear un Lock para evitar accesos concurrentes

    def create_connection(self):
        # Crear una nueva conexión para cada hilo
        return sqlite3.connect(self.db_path)

    def create_table(self):
        with self.lock:  # Asegurarse de que solo un hilo acceda a la base de datos a la vez
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS places (
                    name TEXT PRIMARY KEY,
                    x REAL,
                    y REAL
                )
            ''')
            conn.commit()
            conn.close()

    def insert_place(self, name, x, y):
        with self.lock:  # Asegurarse de que solo un hilo acceda a la base de datos a la vez
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('''
                INSERT OR REPLACE INTO places (name, x, y)
                VALUES (?, ?, ?)
            ''', (name, x, y))
            conn.commit()
            conn.close()


    def get_place(self, name):
        with self.lock:  # Asegurarse de que solo un hilo acceda a la base de datos a la vez
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('''
                SELECT x, y FROM places WHERE name=?
            ''', (name,))
            result = cursor.fetchone()
            conn.close()
            return result
