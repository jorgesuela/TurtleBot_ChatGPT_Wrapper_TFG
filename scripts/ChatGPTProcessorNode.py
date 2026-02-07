#!/usr/bin/env python3

import rospy
import os
from openai import OpenAI, OpenAIError
import subprocess
import tf.transformations
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from RobotSpeaker import RobotSpeaker
from DatabaseHandler import DatabaseHandler

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

class ChatGPTNode:
    def __init__(self):
        rospy.init_node('chatgpt_node', anonymous=True)

        # Configuración de bases de datos
        self.db_paths = {
            "database_1": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_1.db",
            "database_2": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_2.db",
            "database_3": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_3.db",
        }
        self.current_database = "database_1"
        self.db = DatabaseHandler(self.db_paths[self.current_database])

        # Estado del follower
        self.follower_state = "stopped"
        rospy.Subscriber('/follower_state', String, self.follower_state_callback)

        # Estado del corridor follower
        self.corridor_follower_state = "stopped"
        rospy.Subscriber('/wall_follower_state', String, self.corridor_follower_state_callback)

        # Robot Speaker
        self.speaker = RobotSpeaker()

        # Proceso del nodo generado
        self.generated_process = None

        # Cliente OpenAI
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # Estado actual
        self.scan_data = None
        self.pose = None
        self.generated_file = "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/scripts/generated_node.py"

        # Suscripciones
        rospy.Subscriber('/speech_to_text', String, self.handle_user_input)
        rospy.Subscriber('/scan', LaserScan, self.update_scan)
        rospy.Subscriber('/odom', Odometry, self.update_pose)

        rospy.loginfo("Nodo ChatGPT inicializado. Esperando solicitudes del usuario...")

    def update_scan(self, msg):
        self.scan_data = msg

    def update_pose(self, msg):
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        q = [ori.x, ori.y, ori.z, ori.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(q)

        self.pose = {
            "x": round(pos.x, 2),
            "y": round(pos.y, 2),
            "yaw": round(yaw, 2)
        }

    # ----------------- Follower -----------------
    def follower_state_callback(self, msg):
        self.follower_state = msg.data

    # ----------------- Corridor Follower -----------------
    def corridor_follower_state_callback(self, msg):
        self.corridor_follower_state = msg.data

    def handle_user_input(self, msg):
        user_input = msg.data
        rospy.loginfo(f"{YELLOW}Petición recibida: {user_input}{RESET}")
        rospy.loginfo(f"{YELLOW}estado follow me / corridor follower: {self.follower_state} / {self.corridor_follower_state}{RESET}")
        prompt = self.build_prompt(user_input, self.follower_state, self.corridor_follower_state)
        gpt_code = self.query_chatgpt(prompt)

        if gpt_code:
            if self.save_code(gpt_code):
                self.db.insert_user_request(user_input, gpt_code, self.serialize_pose())
                self.run_generated_node()

    def build_prompt(self, user_input, follower_state, corridor_follower_state):
        # Obtener las últimas 5 interacciones para contexto (petición + pose previa)
        last_interactions = self.db.get_last_n_user_requests(5)
        # Construir contexto con especial énfasis en la última interacción
        contexto_interacciones = ""
        for i, (req, pose_json) in enumerate(last_interactions, 1):
            pose_str = pose_json if pose_json else "Posición desconocida"
            marcador = "(última interacción)" if i == 1 else f"(interacción {i})"
            contexto_interacciones += f"{marcador}: Petición: \"{req}\", Posición previa del robot: {pose_str}\n"
        # Obtener lugares almacenados
        stored_places = self.db.get_all_places()
        lugares_str = ", ".join(stored_places) if stored_places else "ninguno"

        return f"""
    Eres un asistente robótico para TurtleBot2 con ROS1 Noetic. Se te dará una solicitud en español y debes generar solo el código completo de un nodo ROS Python3 para que el robot la ejecute.

    === Contexto relevante de interacciones previas ===

    En las últimas peticiones, cada interacción guarda la petición de usuario y la posición previa del robot al ejecutar esa acción.
    Esta información debe usarse para entender referencias implícitas (como "vale", "haz eso") y para posibles reversiones o ajustes basados en la ubicación previa.

    Últimas 5 interacciones (de más antiguas a más recientes):
    {contexto_interacciones}

    === Instrucciones y contexto ===

    - Distingue si la acción está relacionada con el mapa o no:
    * Acciones relacionadas con el mapa incluyen: navegar a un lugar guardado, ir a coordenadas específicas, añadir o eliminar lugares en la base de datos.
    * Acciones no relacionadas con el mapa incluyen: movimientos básicos (avanzar, girar), detección y esquiva de obstáculos, control reactivo, etc.

    - Para navegación con mapa, usa las funciones de TurtleBotActions que gestionan la base de datos y navegación automática.

    - El robot tiene almacenados estos lugares conocidos, debes ser capaz de intuir cuando el usuario se refiere a uno de estos lugares: {lugares_str}.

    - Usa el método `say` para comunicarte con el usuario de forma amable y solo cuando sea importante (una vez al principio y otra al final por ejemplo, como veas necesario).

    === Reglas estrictas ===

    - Solo responde con el código Python completo del nodo ROS, sin explicaciones ni texto adicional.
    - Usa solo comentarios Python si necesitas anotar algo.
    - Importa siempre: `from TurtleBotActions import TurtleBotActions`
    - Hay algunas funciones ya implementadas en `TurtleBotActions` que puedes utilizar si las necesitas,tambien puedes crear tus propias funciones si consideras que es necesario.
    - El robot debe actuar de forma segura y robusta.
    - Debes proporcionar retroalimentación al usuario mediante el método `say` siempre que lo consideres necesario.

    === Funciones disponibles en TurtleBotActions ===

    - stop()
    - get_odom_position() -> (x, y, yaw) or None
    - compute_distance(x0, y0, x1, y1) -> float
    - move_forward(distance, speed=0.2, obstacle_threshold=0.75) -> bool
    - move_backward(distance, speed=0.2, obstacle_threshold=0.75) -> bool
    - rotate(angle_deg, speed=0.5) -> bool
    - get_front_distance() -> float
    - get_left_distance() -> float
    - get_right_distance() -> float
    - is_obstacle_ahead(threshold=0.75) -> bool
    - say(text: str)
    - approach_nearest_obstacle() -> bool

    === Sobre mapas: estas funciones solo sirven cuando el robot tiene un mapa conocido, en caso contrario, comunicar al usuario ===
    - get_map_position() -> (x, y, yaw) or None
    - add_place(lugar: str)
    - delete_place(lugar: str)
    - go_to_place(place_name: str)
    - go_to_coordinates(x, y, yaw) # util para volver a una posición previa

    === MODOS DE MOVIMIENTO AUTÓNOMO ===

    El robot tiene dos modos de movimiento autónomo que **no pueden estar activos al mismo tiempo**. Debes controlar estos modos usando los estados proporcionados y comunicar al usuario cualquier restricción o acción redundante.

    1. FOLLOW ME MODE
    - Propósito: Seguir a una persona automáticamente.
    - Funciones disponibles: follow_me(), stop_follow_me()
    - ACTUA EN BASE A ESTE VALOR : follower_state = {follower_state}, no puedes usar este atributo en tu nodo generado ni acceder a el, genera el nodo fiandote de este valor.
    - MUY IMPORTANTE: Reglas ESTRICTAS de comportamiento:
      * Si follower_state = 'started' y el usuario quiere activar Follow Me, GENERA UN NODO QUE UNICAMENTE HAGA ESTO:
        "say": "Ya estaba en modo Follow Me."
      * Si follower_state = 'stopped' y el usuario quiere desactivar Follow Me, GENERA UN NODO QUE UNICAMENTE HAGA ESTO:
        "say": "No estaba siguiendote."
      * Si corridor_follower_state = 'started' y el usuario quiere activar Follow Me, GENERA UN NODO QUE UNICAMENTE HAGA ESTO:
        "say": "Primero debes desactivar el modo Corridor Follower para poder usar Follow Me."     
      * Jamas uses follow_me() si follower_state = 'started'.
      * Jamas uses stop_follow_me() si follower_state = 'stopped'.

    2. WALL / CORRIDOR FOLLOWER MODE
    - Propósito: Seguir paredes o pasillos automáticamente.
    - Funciones disponibles: start_corridor_follower(), stop_corridor_follower()
    - ACTUA EN BASE A ESTE VALOR : follower_state = {corridor_follower_state}, no puedes usar este atributo en tu nodo generado ni acceder a el, genera el nodo fiandote de este valor.
    - MUY IMPORTANTE: Reglas ESTRICTAS de comportamiento:
      * Si corridor_follower_state = 'started' y el usuario quiere activar Corridor Follower, GENERA UN NODO QUE UNICAMENTE HAGA ESTO:
        "say": "Ya estaba siguiendo la pared/pasillo."
      * Si corridor_follower_state = 'stopped' y el usuario quiere desactivar Corridor Follower, GENERA UN NODO QUE UNICAMENTE HAGA ESTO:
        "say": "No estaba siguiendo la pared/pasillo."
      * Si follower_state = 'started' y el usuario quiere activar Corridor Follower, GENERA UN NODO QUE UNICAMENTE HAGA ESTO:
        "say": "Primero debes desactivar el modo Follow Me para poder usar Corridor Follower."
      * Jamas uses start_corridor_follower() si corridor_follower_state = 'started'.
      * Jamas uses stop_corridor_follower() si corridor_follower_state = 'stopped'.

    IMPORTANTE: tanto en el modo Follow Me como en el modo Corridor Follower, si el usuario te pide parar y alguno de estos modos esta activado, debes interpretar que debes desactivar ese modo.
    
    === Sensores y tópicos relevantes ===

    - /scan: para detectar obstáculos
    - /odom: para posición y orientación odométrica
    - /amcl_pose: para posición y orientación en mapa
    - /cmd_vel_mux/input/navi: para comandos de movimiento
    - /move_base_simple/goal: para navegación automática

    === Parámetros operativos ===

    - Velocidad máxima: 0.35 m/s
    - Distancia mínima segura a obstáculos: 0.75 m
    - Rango del lidar: -30º a +30º frente al robot. Como el rango es muy limitado, es probable que muchas veces necesites comprobar los datos del lidar mientras vas rotando poco a poco para poder hacer algunas tareas.

    === Petición del usuario ===
    "{user_input}"
    """



    def query_chatgpt(self, prompt):
        try:
            response = self.client.chat.completions.create(
                model="gpt-4o",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.4
            )
            return response.choices[0].message.content
        except OpenAIError as e:
            rospy.logerr(f"Error de OpenAI: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"Error inesperado al consultar ChatGPT: {e}")
            return None

    def save_code(self, code):
        try:
            # Eliminar marcas de bloque de código Markdown si existen
            if code.startswith("```"):
                # Remover las tres comillas y la posible palabra "python"
                lines = code.splitlines()
                if lines[0].startswith("```"):
                    lines = lines[1:]  # eliminar primera línea
                if lines[-1].strip() == "```":
                    lines = lines[:-1]  # eliminar última línea
                code = "\n".join(lines)

            with open(self.generated_file, 'w') as f:
                f.write(code)
            os.chmod(self.generated_file, 0o755)
            rospy.loginfo("Código generado guardado correctamente.")
            return True
        except Exception as e:
            rospy.logerr(f"Error al guardar el código: {e}")
            return False

    def run_generated_node(self):
        try:
            terminal_title = "generated_node_terminal"

            # Cerrar terminales anteriores con el mismo título
            self.close_terminals_with_title(terminal_title)

            # Ejecutar nodo generado en nueva terminal
            launch_command = f'gnome-terminal --title="{terminal_title}" -- bash -c "python3 {self.generated_file}; exit"'
            self.generated_process = subprocess.Popen(launch_command, shell=True)
            rospy.loginfo(f"{GREEN}Nodo generado ejecutado en terminal '{terminal_title}'.{RESET}")

        except Exception as e:
            rospy.logerr(f"No se pudo ejecutar el nodo generado: {e}")

    def close_terminals_with_title(self, title):
        try:
            output = subprocess.check_output(['wmctrl', '-l']).decode()
            for line in output.splitlines():
                if title in line:
                    window_id = line.split()[0]
                    subprocess.call(['wmctrl', '-ic', window_id])
        except Exception as e:
            rospy.logwarn(f"No se pudo cerrar terminales con título '{title}': {e}")




    def format_pose(self):
        if not self.pose:
            return "Desconocida"
        p = self.pose.position
        o = self.pose.orientation
        return f"x={p.x:.2f}, y={p.y:.2f}, orientación (quat z={o.z:.2f})"

    def format_scan(self):
        if not self.scan_data:
            return "Sin datos"
        mid = len(self.scan_data.ranges) // 2
        return f"{self.scan_data.ranges[mid]:.2f} metros al frente"

    def serialize_pose(self):
        if not self.pose:
            return None
        return self.pose  # Ya es un diccionario con x, y, yaw


if __name__ == "__main__":
    try:
        ChatGPTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
