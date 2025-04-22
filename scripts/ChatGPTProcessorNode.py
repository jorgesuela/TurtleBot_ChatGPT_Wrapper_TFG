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

"""NODO CHATGPT PROCESSOR"""
"""Este nodo procesa los mensajes de voz recibidos a traves del topic /speech_to_text, los envía a OpenAI y publica las acciones 
resultantes hacia el nodo controller a traves del topic /turtlebot_single_action' ."""

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
        self.db = DatabaseHandler('/home/jorge/catkin_ws/src/turtlebot_chatgpt_wrapper/places.db')
        self.db.create_table()  # Asegúrate de que la tabla exista

        rospy.loginfo("Nodo ChatGPTProcessor iniciado. Esperando mensajes...")

    def _setup_openai(self):
        """Configura la API de OpenAI."""
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            rospy.logerr("No se encontró la API Key de OpenAI.")
            raise RuntimeError("API Key de OpenAI no encontrada.")
        self.client = OpenAI(api_key=api_key)
        self.model = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")

    def _setup_ros(self):
        """Configura la suscripción y publicación de ROS."""
        self.subscription = rospy.Subscriber('/speech_to_text', String, self._process_speech_input)
        self.publisher = rospy.Publisher('/turtlebot_single_action', String, queue_size=30)

    def _get_actions_from_gpt(self, user_input, places):
        """Envía la entrada a OpenAI y devuelve la lista de acciones."""
        # Convertir la lista de lugares en un formato de texto para agregar al prompt
        places_text = ", ".join(places)
        
        prompt = (
            "Eres un asistente que traduce comandos en lenguaje natural en JSON para un TurtleBot. "
            "Devuelve siempre la respuesta en formato JSON válido. "
            "Si no hay ninguna acción, responde con un array vacío []. "
            "Ejemplo de salida esperada:\n"
            "[{\"action\": \"move\", \"distance\": 2, \"velocity\": 0.15}, "
            "{\"action\": \"move\", \"distance\": 1, \"velocity\": 0.25}, "
            "{\"action\": \"turn\", \"angle\": 90}, "
            "{\"action\": \"move\", \"distance\": 1}, "
            "{\"action\": \"explore\", \"time_limit\": 80}, "
            "{\"action\": \"stop\"}, "
            "{\"action\": \"add_place\", \"name\": \"cocina\"}, "
            "{\"action\": \"go_to_place\", \"place\": \"cocina\"}]\n"
            "Notas importantes: - izquierda es ángulo negativo y derecha ángulo positivo\n"
            "                - El formato de salida debe ser un array siempre.\n"
            "                - La funcion move tiene un parametro velocidad, tu rango va desde muy lento 0.15 a muy rapido 0.35. si no te dicen nada, la vel por defecto es 0.25.\n"
            "                - El comando 'add_place' requiere el parámetro 'name' todo en minusculas y sin acentos.\n"
            "                - frases como 'esto es el salon' o 'guarda este sitio como el salon', significan que hagas un add_place.\n"
            "                - El comando 'go_to_place' requiere el parámetro 'place', que es el nombre de un lugar, todo en minusculas y sin acentos.\n"
            "                - si se te pide que vayas a algun lugar(go_to_place), estos son los sitios a los que eres capaz de ir, debes ser capaz de intuir a donde quiere ir el usuario:\n"
            "                - la accion explore necesita el argumento time_limit, si no se especifica por defecto pon 60 segundos. el tiempo siempre sera en segundos\n"
            f"Lugares disponibles: {places_text}\n"
            f"Entrada: '{user_input}'"
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

    def spin(self):
        """Mantiene el nodo en funcionamiento."""
        rospy.spin()

if __name__ == "__main__":
    chatgpt_processor = ChatGPTProcessor()
    chatgpt_processor.spin()
