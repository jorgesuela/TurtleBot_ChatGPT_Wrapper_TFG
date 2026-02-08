import sqlite3
import threading
import json

"IMPORTANTE: Tanto el nodo de chatgpt como turtlebot_actions acceden a la base de datos."
"Esta clase debe usar thread locks para no permitir accesos simultaneos"

class DatabaseHandler:

    def __init__(self, db_path):
        self.db_path = db_path
        self.lock = threading.Lock()  # Crear un Lock para evitar accesos concurrentes

        # Crear las tablas si no existen al inicializar
        self.create_coordinates_table()
        self.create_user_requests_table()

    def create_connection(self):
        # Crear una nueva conexión para cada hilo
        return sqlite3.connect(self.db_path)

    def create_coordinates_table(self):
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS places (
                    name TEXT PRIMARY KEY,
                    x REAL,
                    y REAL,
                    yaw REAL
                )
            ''')
            conn.commit()
            conn.close()

    def create_user_requests_table(self):
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('''
                CREATE TABLE IF NOT EXISTS user_requests (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    user_input TEXT,
                    gpt_response TEXT,
                    pose_json TEXT,
                    timestamp DATETIME DEFAULT CURRENT_TIMESTAMP
                )
            ''')
            conn.commit()
            conn.close()

#### OPERACIONES DE GUARDADO ####
    
    def insert_place(self, name, x, y, yaw):
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('''
                INSERT OR REPLACE INTO places (name, x, y, yaw)
                VALUES (?, ?, ?, ?)
            ''', (name, x, y, yaw))
            conn.commit()
            conn.close()

    # solo se guardan las ultimas 10 interacciones
    def insert_user_request(self, user_input, gpt_response, pose=None):
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()

            pose_json = json.dumps(pose) if pose else None

            # Insertar la nueva interacción con la pose
            cursor.execute('''
                INSERT INTO user_requests (user_input, gpt_response, pose_json)
                VALUES (?, ?, ?)
            ''', (user_input, gpt_response, pose_json))

            # Borrar las entradas más antiguas si hay más de 10
            cursor.execute('''
                DELETE FROM user_requests
                WHERE id NOT IN (
                    SELECT id FROM user_requests
                    ORDER BY timestamp DESC
                    LIMIT 10
                )
            ''')

            conn.commit()
            conn.close()

#### OPERACIONES DE ELIMINACION ####

    def delete_place(self, name):
        """
        Elimina un lugar de la base de datos por su nombre.
        :param name: Nombre del lugar a eliminar.
        :return: True si el lugar fue eliminado, False si no existía.
        """
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('DELETE FROM places WHERE name=?', (name,))
            deleted = cursor.rowcount > 0  # True si alguna fila fue eliminada
            conn.commit()
            conn.close()
            return deleted

#### OPERACIONES DE CONSULTA ####

    def get_place(self, name):
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('''
                SELECT x, y, yaw FROM places WHERE name=?
            ''', (name,))
            result = cursor.fetchone()
            conn.close()
            return result  # Retorna (x, y, yaw) o None
    
    def get_all_places(self):
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('SELECT name FROM places')
            places = cursor.fetchall()
            conn.close()
            return [place[0] for place in places]  # Devuelve una lista con los nombres de los lugares
        
    def get_last_n_user_requests(self, n=3):
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('''
                SELECT user_input, pose_json FROM user_requests
                ORDER BY timestamp DESC LIMIT ?
            ''', (n,))
            result = cursor.fetchall()
            conn.close()
            return result
