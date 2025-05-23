#!/usr/bin/env python3

import os
import json
import rospy
from std_msgs.msg import String
from openai import OpenAI, OpenAIError
from DatabaseHandler import DatabaseHandler

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

"""
NODO CHATGPT PROCESSOR
Este nodo procesa los mensajes de voz recibidos a traves del topic /speech_to_text, los envía a OpenAI y publica las acciones 
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
        self.db = DatabaseHandler('/home/jorge/catkin_ws/src/turtlebot_chatgpt_wrapper/database/turtlebot_db.db')
        # Asegúrate de que las tablas existan
        self.db.create_coordinates_table() 
        self.db.create_user_requests_table()

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
        self.publisher = rospy.Publisher('/turtlebot_single_action', String, queue_size=30)

    def _get_actions_from_gpt(self, user_input, places):
        """Envía la entrada a OpenAI y devuelve la lista de acciones."""
        # Convertir la lista de lugares en un formato de texto para agregar al prompt
        places_text = ", ".join(places)
        # Últimas acciones del usuario (solo las peticiones) para mayor contexto
        recent_interactions = self.db.get_last_n_user_requests()
        recent_user_inputs = [user for user, _ in recent_interactions]
        recent_context = json.dumps(recent_user_inputs)
        
        
        prompt = (
            "Eres un asistente que convierte instrucciones en lenguaje natural en comandos JSON para un TurtleBot.\n"
            "Tu salida debe ser siempre estrictamente un **array JSON válido** (sin código, sin etiquetas adicionales, sin bloques Markdown).\n\n"
            "Reglas estrictas de formato:\n"
            "- Solo devuelve un array JSON. No expliques, no añadas encabezados como 'Respuesta:' ni bloques ```json.\n"
            "- Todas las acciones deben tener un campo 'say', que contiene lo que el robot dirá.\n"
            "- si no reconoces ninguna accion valida, sino que simplemente te hacen una pregunta, usa el campo say para responder dicha pregunta.\n"
            "- Si no hay ninguna acción reconocible, responde únicamente con: [{\"say\": \"No he entendido lo que quieres que haga.\"}]\n\n"
            "Ejemplos válidos de salida:\n"
            "[{\"action\": \"move\", \"distance\": 2, \"velocity\": 0.35, \"say\": \"¡vale, voy!\"}]\n"
            "[{\"action\": \"go_to_place\", \"place\": \"cocina\", \"say\": \"Voy a la cocina\"}]\n"
            "[{\"say\": \"No sé cómo ayudarte con eso.\"}]\n\n"
            "Detalles importantes:\n"
            "- 'move' requiere: 'distance' y opcionalmente 'velocity' (0.25 a 0.5). Por defecto, 0.35.\n"
            "- 'turn' requiere: 'angle' (negativo para izquierda, positivo para derecha).\n"
            "- 'add_place' y 'delete_place' requieren 'name' (en minúsculas y sin acentos). Sirve para recordar coordenadas de lugares en la bd.\n"
            "- 'go_to_place' requiere 'place'. Solo lugares válidos que esten en la bd, actualmente estan disponibles los siguientes: " + "[" + places_text + "]" + "\n"
            "- 'explore' requiere 'time_limit'. Si no se da, usa 60 por defecto.\n"
            "- 'follow_me' y 'stop_follow_me' no requieren parámetros.\n\n"
            "Sé creativo y amigable en los mensajes de 'say'. Varía el tono. No uses siempre las mismas frases.\n\n"
            "Esta es tu memoria de ultimas acciones solicitadas ordendas de mas reciente a mas antigua:" + recent_context + ".\n"
            f"Instrucción del usuario: '{user_input}'"
        )
        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.2
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

        # Obtener la lista de lugares de la base de datos(sirve de contexto para chatgpt)
        places = self.db.get_all_places()

        rospy.loginfo(f"{YELLOW}Procesando mensaje: {user_input}{RESET}")
        actions = self._get_actions_from_gpt(user_input, places)
        if actions:
            self._publish_actions(actions)
            # guardar la entrada del usuario y la respuesta de GPT en la base de datos
            gpt_response_json = json.dumps(actions)
            self.db.insert_user_request(user_input, gpt_response_json)

    def spin(self):
        """Mantiene el nodo en funcionamiento."""
        rospy.spin()

if __name__ == "__main__":
    chatgpt_processor = ChatGPTProcessor()
    chatgpt_processor.spin()
