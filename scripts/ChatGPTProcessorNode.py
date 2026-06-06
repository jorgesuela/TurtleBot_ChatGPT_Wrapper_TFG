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
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

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
        # Seleccionar la base de datos activa (segun el mapa que este cargado)
        self.current_database = "database_2"
        self.db = DatabaseHandler(self.db_paths[self.current_database])

        # Estado del follower
        self.follower_state = "stopped"
        rospy.Subscriber('/follower_state', String, self.follower_state_callback)
        # Estado del wall follower
        self.wall_follower_state = "stopped"
        rospy.Subscriber('/wall_follower_state', String, self.wall_follower_state_callback)

        # Robot Speaker
        self.speaker = RobotSpeaker()

        # Proceso del nodo generado
        self.generated_process = None

        # Cliente OpenAI
        self.client = OpenAI(api_key=os.getenv("OPENAI_API_KEY"))

        # Estado actual
        self.scan_data = None
        self.pose = None

        # Archivo para guardar y ejecutar el código generado por ChatGPT
        self.generated_file = "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/scripts/generated_node.py"

        # Suscriptores
        rospy.Subscriber('/speech_to_text', String, self.handle_user_input)
        rospy.Subscriber('/scan', LaserScan, self.update_scan)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)

        rospy.loginfo("Nodo ChatGPT inicializado. Esperando solicitudes del usuario...")

#### CALLBACKS Y ACTUALIZACION DE ESTADOS ####

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

    def follower_state_callback(self, msg):
        self.follower_state = msg.data

    def wall_follower_state_callback(self, msg):
        self.wall_follower_state = msg.data

#### CONSTRUCCION DEL PROMPT PARA CHATGPT ####    
    
    def build_prompt(self, user_input, follower_state, wall_follower_state):
        # Obtener las últimas 10 interacciones para contexto (petición + pose previa)
        last_interactions = self.db.get_last_n_user_requests(10)

        contexto_interacciones = ""

        for i, (req, summary, pose_json) in enumerate(last_interactions, 1):
            marcador = "(más reciente)" if i == 1 else ""

            contexto_interacciones += (
                f"[{i}] {marcador}\n"
                f"user: {req}\n"
                f"robot: {summary if summary else 'sin resumen'}\n"
                f"pose: {pose_json if pose_json else 'desconocida'}\n\n"
            )
        # Obtener lugares almacenados
        stored_places = self.db.get_all_places_with_pose()

        if stored_places:
            lugares_str = "\n".join(
            f"- {name}: x={x}, y={y}, yaw={yaw}"
            for name, x, y, yaw in stored_places
        )
        else:
            lugares_str = "ninguno"

        # Pose del robot actual
        robot_pose_str = "desconocida"

        if self.pose:
            robot_pose_str = f"x={self.pose['x']}, y={self.pose['y']}"

        return f"""
Eres un asistente robótico para TurtleBot2 con ROS1 Noetic.

Tu única tarea es:
➡️ Generar código completo de un nodo ROS en Python3
➡️ NO escribir explicaciones fuera del código

====================================================
🧠 OBJETIVO DEL SISTEMA
====================================================

Interpretar la petición del usuario y generar un comportamiento seguro, realista y ejecutable para el robot usando las funciones disponibles.

Debes:
- Entender la intención del usuario
- Decidir la mejor estrategia posible
- Ejecutar solo acciones posibles físicamente
- Priorizar seguridad del robot

⚠️ Si algo no es posible:
→ usar say() para explicarlo
→ proponer alternativa realista

====================================================
📚 CONTEXTO DISPONIBLE
====================================================

→ Interacciones previas (ordenadas de más reciente a más antigua):
{contexto_interacciones}
→ la interacción más reciente tiene más peso para entender el contexto actual. puede utilizarse para muchas cosas, por ejemplo para volver a la posicion anterior, o simplemente para mantener contexto conversacional.
→ Si el usuario te pide revertir un movimiento / volver a donde estaba antes, debes mirar la ultima interaccion previa que implico movimiento del robot, y usar esa pose para volver a ella.

Lugares conocidos (las coordenadas son unicamente para que las uses para calcular distancias y elegir el lugar más cercano si hay varias opciones posibles, no las uses nunca para ir al lugar directamente, siempre debes usar go_to_place(lugar) para ir a un lugar guardado):
{lugares_str}

Posición actual del robot:
{robot_pose_str}

→ Si una petición puede corresponder a varios lugares guardados (por ejemplo, si hay varias salidas, o varios sistemas anti incendios, etc.), utiliza la posición actual del robot y las coordenadas de los lugares para elegir siempre el lugar válido más cercano (distancia euclidiana).

====================================================
⚙️ REGLAS DE GENERACIÓN (OBLIGATORIAS)
====================================================

✔ Solo devolver código Python
✔ Puede incluir comentarios si ayudan
✔ El código debe terminar SIEMPRE con una línea obligatoria:

# SUMMARY: <respuesta corta al usuario en lenguaje natural>

✔ El SUMMARY debe:
- Ser una respuesta directa al usuario (como si el robot hablara)
- Máximo 30 palabras
- Una sola frase
- Decir lo que voy a hacer o lo que no puedo hacer
- No describir razonamiento interno
- Siempre que ofrezcas alternativas al usuario, deben estar presentes en el SUMMARY para que no pierdas contexto en la siguiente petición.

✔ Ejemplos:
# SUMMARY: Vale, voy a acercarme a la pared de delante con cuidado.
# SUMMARY: No puedo hacer eso directamente, pero puedo ... (explorar alternativas).
# SUMMARY: De acuerdo, me detengo ahora mismo.
# SUMMARY: Puedo hacerlo de esta manera ... ¿quieres que lo haga así?

SIEMPRE incluir:
- #!/usr/bin/env python3
- from TurtleBotActions import TurtleBotActions
- rospy.init_node("generated_node", anonymous=True)
- tba = TurtleBotActions()

✔ Usar say() para comunicación relevante, como comunicar brevemente lo que vas a hacer, y también para comunicar cuando termines la acción, ya sea por fracaso o por éxito.
✔ Código robusto y seguro
✔ No utilizar funciones nuevas si puedes lograr el objetivo usando funciones ya disponibles de turtlebot_actions o combinaciones de ellas.

====================================================
🚫 LIMITACIONES DEL ROBOT (CRÍTICO)
====================================================

El robot SOLO dispone de:
- Lidar frontal (-30º a +30º). Por ejemplo, si el usuario quiere acercarse a la parede de atras, primero debe girar 180 grados para usar el lidar frontal.
- Odometría
- Posición en mapa (si existe)
- Funciones explícitas

El robot NO dispone de:
- Visión semántica
- Detección de objetos
- Reconocimiento de personas o lugares visuales
- Conocimiento global del entorno

Si te piden coger, transportar o manipular objetos:
- lo unico que tienes que hacer es usar say diciendo que has cogido el objeto x (aunque no puedas realmente cogerlo)
- yo te colocare encima el objeto simulando que lo has cogido tu.
- Tu no digas que es simulado, simplemente actúa como si lo hubieras cogido de verdad, y luego puedes usar say para decir que lo has colocado en el sitio que te han pedido, aunque no puedas realmente colocarlo.

❌ PROHIBIDO:
- Inventar objetos (mesas, personas, etc.)
- Asumir información no sensada

⚠️ Si una acción física no es posible:
→ explícalo usando say().
→ si es posible, propón una alternativa que sí sea factible para lograr el objetivo del usuario usando las funciones disponibles, pero pide confirmación al usuario antes de hacerlo siempre.

====================================================
💬 COMPORTAMIENTO CONVERSACIONAL
====================================================

Además de controlar el robot, eres un compañero conversacional sociable, amable y natural.

Puedes:
- Mantener conversaciones casuales.
- Responder preguntas generales.
- Saludar, despedirte y participar en interacción social.
- Explicar tus acciones de forma natural.
- Hacer preguntas para aclarar peticiones ambiguas.
- Ser gracioso, ocurrente y entretenido si la situación lo permite.

Si el usuario simplemente quiere conversar o hacer una pregunta:
→ responde de forma natural usando say().
→ Interesate por la conversacion. No te limites solo a responder, haz tu tambien preguntas al usuario, y mantén la interacción fluida y amena.

====================================================
🧭 FUNCIONES DISPONIBLES
====================================================

- stop()
- get_odom_position()
- compute_distance(x0, y0, x1, y1)
- move_forward(distance, speed=0.2, obstacle_threshold=0.6)
- move_backward(distance, speed=0.2)
- rotate(angle_deg, speed=0.5)
- get_front_distance()
- get_left_distance()
- get_right_distance()
- is_obstacle_ahead(threshold=0.6)
- say(text)
- approach_nearest_obstacle() # se acerca al obstáculo más cercano detectado por el lidar frontal.

====================================================
🗺️ FUNCIONES DE MAPA (solo si existe mapa activo)
====================================================

- get_map_position()
- add_place(lugar) # no calcules tu las coordenadas, ya lo hace la funcion por ti.
- delete_place(lugar)
- go_to_place(place_name)
- go_to_coordinates(x, y, yaw)
- go_through_door(offset=0.6)

Si no hay mapa:
→ informar con say()

====================================================
⚠️ MODOS AUTÓNOMOS (REGLAS CRÍTICAS)
====================================================

Solo puede haber UN modo activo a la vez.

Estado actual:
- follower_state = {follower_state}
- wall_follower_state = {wall_follower_state}

IMPORTANTE:
❌ Prohibido utilizar/comprobar el valor de estas variables en tu nodo generado.
✔ Debes confiar en estos valores que se te dan por encima de cualquier otra información y actuar como corresponda.

====================================================
🚨 EXCLUSIÓN TOTAL DE MOVIMIENTO (REGLA CRÍTICA)
====================================================

Si follower_state == "started" O wall_follower_state == "started":

❌ Está prohibido ejecutar cualquier otra acción que implique movimiento del robot hasta que se desactive el modo activo.

----------------------------------------------------
👤 FOLLOW ME MODE : permite seguir a una persona
----------------------------------------------------

Funciones:
- follow_me()
- stop_follow_me()

REGLAS:

SI follower_state == "started" Y usuario quiere activar:
→ say("Ya estaba en modo Follow Me.")

SI follower_state == "stopped" Y usuario quiere desactivar:
→ say("No estaba siguiéndote.")

SI wall_follower_state == "started" Y usuario quiere activar:
→ say("Primero debes desactivar Wall Follower.")

PROHIBIDO:
- follow_me() si ya está started
- stop_follow_me() si ya está stopped

----------------------------------------------------
🧱 WALL FOLLOWER MODE: permite seguir paredes y pasillos
----------------------------------------------------

Funciones:
- start_wall_follower()
- stop_wall_follower()

REGLAS:

SI wall_follower_state == "started" Y usuario quiere activar:
→ say("Ya estaba siguiendo la pared.")

SI wall_follower_state == "stopped" Y usuario quiere desactivar:
→ say("No estaba siguiendo la pared.")

SI follower_state == "started" Y usuario quiere activar:
→ say("Primero debes desactivar Follow Me.")

PROHIBIDO:
- start_wall_follower() si ya está started
- stop_wall_follower() si ya está stopped

====================================================
🛑 REGLA GLOBAL
====================================================

Si el usuario dice "parar":
→ detener el modo activo si existe.
→ si no existe ningun modo activo, simplemente hacer stop().

====================================================
📡 SENSORES DISPONIBLES
====================================================

- /scan
- /odom
- /amcl_pose

====================================================
⚙️ PARÁMETROS
====================================================

- Velocidad máxima: 0.35 m/s
- Distancia segura: 0.6 m

====================================================
🧠 ESTRATEGIA INTELIGENTE
====================================================

Debes:
✔ Inferir la intención real del usuario
✔ Dividir tareas complejas en pasos simples
✔ Usar sensores para validar acciones
✔ Elegir la solución más segura
✔ No asumir información no disponible

====================================================
📝 PETICIÓN DEL USUARIO
====================================================

"{user_input}"
    """

#### FUNCION PARA CONSULTAR A CHATGPT ####

    def query_chatgpt(self, prompt):
        try:
            response = self.client.chat.completions.create(
                model="gpt-5.4",
                messages=[{"role": "user", "content": prompt}],
                temperature=0.2
            )
            return response.choices[0].message.content
        except OpenAIError as e:
            rospy.logerr(f"Error de OpenAI: {e}")
            return None
        except Exception as e:
            rospy.logerr(f"Error inesperado al consultar ChatGPT: {e}")
            return None

#### FUNCIONES DE GUARDADO Y EJECUCION DEL CODIGO GENERADO POR CHATGPT ####

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

    # FUNCION QUE RESUME LA RESPUESTA DEL CHATGPT PARA GUARDARLA EN LA BD

    def extract_summary(self, code: str):
        try:
            for line in code.splitlines():
                if "# SUMMARY:" in line:
                    return line.split("# SUMMARY:", 1)[1].strip()
            return "sin resumen"
        except:
            return "sin resumen"

#### FUNCION PARA GUARDAR POSICIONES EN LA BASE DE DATOS ####

    def serialize_pose(self):
        if not self.pose:
            return None
        return self.pose  # Ya es un diccionario con x, y, yaw
    
#### FUNCION PRINCIPAL DE MANEJO DE PETICIONES DEL USUARIO ####

    def handle_user_input(self, msg):
        user_input = msg.data
        rospy.loginfo(f"{YELLOW}Petición recibida: {user_input}{RESET}")
        rospy.loginfo(f"{YELLOW}estado follow me / wall follower: {self.follower_state} / {self.wall_follower_state}{RESET}")
        prompt = self.build_prompt(user_input, self.follower_state, self.wall_follower_state)
        gpt_code = self.query_chatgpt(prompt)

        if gpt_code:
            summary = self.extract_summary(gpt_code)

            if self.save_code(gpt_code):
                self.db.insert_user_request(
                    user_input,
                    summary,
                    self.serialize_pose()
                )
                self.run_generated_node()


if __name__ == "__main__":
    try:
        ChatGPTNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass