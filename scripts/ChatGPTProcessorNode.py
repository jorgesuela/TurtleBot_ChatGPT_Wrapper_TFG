#!/usr/bin/env python3

import os
import json
import rospy
from std_msgs.msg import String
from openai import OpenAI, OpenAIError
from DatabaseHandler import DatabaseHandler
from nav_msgs.msg import Odometry
import tf.transformations

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

"""
NODO CHATGPT PROCESSOR
Este nodo procesa los mensajes de voz recibidos a travimport tf.transformationses del topic /speech_to_text, los envía a OpenAI y publica las acciones 
resultantes de una en una hacia el nodo controller a traves del topic /turtlebot_single_action' .
"""

class ChatGPTProcessor:
    def __init__(self):
        """
            Topics:
            subscribe: /speech_to_text
            publish: /turtlebot_single_action
            """
        rospy.init_node('chatgpt_processor_node', anonymous=True)
        self._setup_openai()
        self._setup_ros()

        # Crear una instancia de DatabaseHandler en el hilo principal
        self.db_paths = {
            "database_1": "/home/jorge/catkin_ws/src/turtlebot_chatgpt_wrapper/database/turtlebot_database_1.db", # ESTA DB ES DEL MAPA XXXX
            "database_2": "/home/jorge/catkin_ws/src/turtlebot_chatgpt_wrapper/database/turtlebot_database_2.db", # ESTA DB ES DEL MAPA XXXX
            "database_3": "/home/jorge/catkin_ws/src/turtlebot_chatgpt_wrapper/database/turtlebot_database_3.db", # ESTA DB ES DEL MAPA XXXX
        }
        self.current_database = "database_1" # MAPA SELEECCIOANDO, CAMBIAR SEGUN NECESIDAD
        self.db = DatabaseHandler(self.db_paths[self.current_database])
        # Asegúrate de que las tablas existan
        self.db.create_coordinates_table() 
        self.db.create_user_requests_table()

        self.current_pose = None
        self.follower_active = False  # Indica si el robot está siguiendo al usuario

        rospy.loginfo("Nodo ChatGPTProcessor iniciado. Esperando mensajes...")

    def _setup_openai(self):
        """Configura la API de OpenAI."""
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            rospy.logerr("No se encontró la API Key de OpenAI.")
            raise RuntimeError("API Key de OpenAI no encontrada.")
        self.client = OpenAI(api_key=api_key)
        self.model = os.getenv("OPENAI_MODEL", "gpt-4o")

    def _setup_ros(self):
        """Configura la suscripción y publicación de ROS."""
        self.subscription = rospy.Subscriber('/speech_to_text', String, self._process_speech_input)
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self._odom_callback)
        self.publisher = rospy.Publisher('/turtlebot_single_action', String, queue_size=30)

    def _odom_callback(self, msg):
        """Actualiza la pose actual del robot."""
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        yaw = self._quaternion_to_yaw(orientation)

        self.current_pose = {
            "x": round(position.x, 2),
            "y": round(position.y, 2),
            "yaw": round(yaw, 2)
        }

    def _quaternion_to_yaw(self, orientation):
        q = [orientation.x, orientation.y, orientation.z, orientation.w]
        _, _, yaw = tf.transformations.euler_from_quaternion(q)
        return yaw

    def _get_actions_from_gpt(self, user_input, places):
        """Envía la entrada a OpenAI y devuelve la lista de acciones."""
        # Convertir la lista de lugares en un formato de texto para agregar al prompt
        places_text = ", ".join(places)

        # Obtener historial reciente de interacciones (input + respuesta)
        recent_interactions = self.db.get_last_n_user_requests()
        recent_context = []
        for user_input_text, gpt_response_text, pose_json in recent_interactions:
            interaction = {
                "user": user_input_text,
                "gpt": gpt_response_text
            }
            if pose_json:
                try:
                    interaction["pose"] = json.loads(pose_json)
                except json.JSONDecodeError:
                    rospy.logwarn("Error al decodificar pose_json, se ignorará.")
            recent_context.append(interaction)

        # Separar la última interacción del resto
        if recent_context:
            last_interaction = recent_context[0]
            previous_interactions = recent_context[1:]
        else:
            last_interaction = None
            previous_interactions = []

        historial_json = json.dumps(previous_interactions, ensure_ascii=False, indent=2)
        last_json = json.dumps(last_interaction, ensure_ascii=False, indent=2) if last_interaction else "{}"

        # Construir el prompt para OpenAI
        prompt = (
            "Eres un asistente que convierte instrucciones en lenguaje natural en comandos JSON para un TurtleBot.\n"
            "Tu salida debe ser siempre estrictamente un **array JSON válido**, sin ningún formato adicional. Esto significa:\n"
            "- No incluir explicaciones, encabezados como 'Respuesta:', ni bloques de código (como ```json o similares).\n"
            "- El contenido debe ser solo un array JSON bien formado.\n\n"

            "🧷 Reglas estrictas de formato:\n"
            "- Todas las acciones deben tener un campo obligatorio 'say', que contiene lo que el robot dirá.\n"
            "- Si no puedes ejecutar directamente la petición, **debes inferir la intención probable del usuario y actuar con sentido común**."
            "- MUY IMPORTANTE: Si necesitas proponer una alternativa (como seguir al usuario a un lugar desconocido), **debes preguntar primero sin ejecutar ninguna acción todavía**."
            "- En esos casos, tu salida debe ser únicamente un array con un solo objeto con 'say', y sin ningún campo 'action', hasta que el usuario confirme."
            "- Solo si el usuario confirma explícitamente (por ejemplo: 'sí', 'hazlo', 'vale'), entonces puedes devolver las acciones necesarias."
            " Por ejemplo:\n"
            "  - Si el usuario pide que le traigas algo (como 'tráeme agua' o 've a por unas uvas'), puedes suponer que quiere que vayas a un lugar específico como la cocina.\n"
            "  - Si ese lugar no está en la base de datos, propón seguir al usuario y guardar la ubicación como 'cocina' (o pregunta si ese es el lugar correcto).\n"
            "  - No digas simplemente que no sabes hacerlo si puedes hacer algo similar o útil."
            "- Si parece que te hacen una pregunta, responde únicamente con el campo 'say' y una respuesta adecuada.\n"
            "- Debes revisar siempre las interacciones anteriores para evitar responder con las mismas palabras.\n"
            "- Mantén un tono amigable, proactivo e inteligente. Sé creativo en las respuestas de 'say'.\n\n"

            "📌 Ejemplos válidos de salida:\n"
            "[{\"action\": \"move\", \"distance\": 2, \"velocity\": 0.2, \"say\": \"¡vale, voy!\"}]\n"
            "[{\"action\": \"go_to_place\", \"place\": \"cocina\", \"say\": \"Voy a la cocina\"}]\n"
            "[{\"say\": \"No sé cómo ayudarte con eso.\"}]\n"
            "[{\"action\": \"follow_me\", \"say\": \"Puedo seguirte hasta ese lugar y luego guardarlo, ¿te parece bien?\"}]\n\n"

            "🔧 Detalles importantes sobre los comandos:\n"
            "- 'move': requiere 'distance' (marcha atras = distancia negativa). Puede tener 'velocity' (rango permitido: 0.15 a 0.3). Si no se especifica, usa 0.2.\n"
            "- 'turn': requiere 'angle' (positivo = derecha, negativo = izquierda). El ángulo mínimo permitido es 15 grados.\n"
            "- 'add_place' y 'delete_place': requieren 'name'. El nombre debe estar en minúsculas y sin acentos. Se usa para guardar/eliminar ubicaciones en la base de datos.\n"
            "- 'go_to_place': requiere 'place'. Solo se puede usar con lugares registrados en la base de datos. Los lugares actualmente disponibles son: [" + places_text + "]\n"
            "- 'explore': requiere 'time_limit'. Si no se especifica, usa 60 segundos por defecto.\n"
            "- 'follow_me' y 'stop_follow_me': no requieren parámetros. si modo follow = true, en nodo ya esta activo y no hay que hacer nada. si el seguimiento esta activado, no se puede realizar ninguna otra accion, debes avisar al usuario para desactive el seguimiento si quiere hacer otras cosas.\n"
            "- 'go_to_coordinates': necesita la 'x', 'y' y 'yaw', se utiliza solo para mover al robot a las coordenadas donde estaba en la última interacción (revertir accion de movimiento). Debes usar las coordenadas de la ultima interaccion que este relacionada con movimiento.\n"
            "- 'approach_nearest_obstacle': permite al robot acercarse al obstáculo más cercano sin chocar. Acepta opcionalmente 'speed' (por defecto 0.15)\n\n."
            
            "🧠 Contexto conversacional:\n"
            "- Frases como 'claro', 'hazlo', 'adelante', 'vale', 'sí por favor', deben interpretarse como una confirmación de la acción propuesta en la ultima interaccion."
            "- Usa la última interacción como contexto directo. Las anteriores son menos relevantes pero pueden ayudarte a mantener variedad y coherencia.\n"
            "- Usa tu intuición para inferir a qué lugar podrían referirse ciertos términos comunes como 'cocina', 'salón', etc. Si ese lugar no está en la base de datos, propón acciones como seguir al usuario y aprender la ubicación.\n\n"

            f"📎 Última interacción inmediata (muy importante para referencias implícitas):\n{last_json}\n\n"
            f"📚 Interacciones anteriores (menos relevantes):\n{historial_json}\n\n"
            f"📚 Estado del modo follow:\n{self.follower_active}\n\n"
            f"🗣️ Instrucción actual del usuario: '{user_input}'"
        )


        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.7
            )
            rospy.loginfo(f"Respuesta GPT sin procesar: {response.choices[0].message.content}")
            return json.loads(response.choices[0].message.content)
        except (OpenAIError, json.JSONDecodeError) as e:
            rospy.logerr(f"Error al obtener acciones de OpenAI: {e}")
            return []
        except Exception as e:
            rospy.logerr(f"Error inesperado: {e}")
            return []
        
    def _publish_actions(self, actions):
        """Publica cada acción individualmente con una pausa entre ellas."""
        for action in actions:
            output_msg = String()
            output_msg.data = json.dumps(action)
            self.publisher.publish(output_msg)
            rospy.loginfo(f"{GREEN}Acción enviada:{RESET} \n{action}")
            rospy.sleep(1)

    def _process_speech_input(self, msg):
        """Procesa el mensaje recibido, lo traduce y publica las acciones."""
        user_input = msg.data.strip().lower()
        places = self.db.get_all_places()

        rospy.loginfo(f"{YELLOW}Procesando mensaje: {user_input}{RESET}")
        actions = self._get_actions_from_gpt(user_input, places)
        
        if actions:
            gpt_response_json = json.dumps(actions)
            movimiento_detectado = False

            for action in actions:
                action_type = action.get("action")

                # Detecta si es una acción de movimiento
                if action_type in ["move", "turn", "go_to_place", "go_to_coordinates", "explore", "follow_me", "approach_nearest_obstacle"]:
                    movimiento_detectado = True

                # Gestiona el estado del follower
                if action_type == "follow_me":
                    if not self.follower_active:
                        self.follower_active = True
                        rospy.loginfo(f"{GREEN}Follower ACTIVADO{RESET}")
                elif action_type == "stop_follow_me":
                    if self.follower_active:
                        self.follower_active = False
                        rospy.loginfo(f"{YELLOW}Follower DESACTIVADO{RESET}")

            # Guarda interacción en base de datos
            if movimiento_detectado and self.current_pose:
                self.db.insert_user_request(user_input, gpt_response_json, pose=self.current_pose)
            else:
                self.db.insert_user_request(user_input, gpt_response_json, pose=None)

            self._publish_actions(actions)

            

    def spin(self):
        """Mantiene el nodo en funcionamiento."""
        rospy.spin()

if __name__ == "__main__":
    chatgpt_processor = ChatGPTProcessor()
    chatgpt_processor.spin()
