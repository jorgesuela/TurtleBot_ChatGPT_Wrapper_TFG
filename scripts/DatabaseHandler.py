import sqlite3
import threading

"IMPORTANTE: Tanto el nodo de chatgpt como el nodo de control del robot acceden a la base de datos."
"Esta clase debe usar thread locks para no permitir accesos simultaneos"

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
    
    def get_all_places(self):
        with self.lock:
            conn = self.create_connection()
            cursor = conn.cursor()
            cursor.execute('SELECT name FROM places')
            places = cursor.fetchall()
            conn.close()
            return [place[0] for place in places]  # Devuelve una lista con los nombres de los lugares
