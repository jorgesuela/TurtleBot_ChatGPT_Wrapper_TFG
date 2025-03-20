#!/usr/bin/env python3

import os
import json
import rospy
from std_msgs.msg import String
from openai import OpenAI, OpenAIError

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

class ChatGPTProcessor:
    def __init__(self):
        """Inicializa el nodo y configura OpenAI y ROS."""
        rospy.init_node('chatgpt_processor_node', anonymous=True)
        self._setup_openai()
        self._setup_ros()
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
        self.publisher = rospy.Publisher('/turtlebot_single_action', String, queue_size=10)

    def _process_speech_input(self, msg):
        """Procesa el mensaje recibido, lo traduce y publica las acciones."""
        user_input = msg.data.strip().lower()
        rospy.loginfo(f"{YELLOW}Procesando mensaje: {user_input}{RESET}")
        actions = self._get_actions_from_gpt(user_input)
        if actions:
            self._publish_actions(actions)

    def _get_actions_from_gpt(self, user_input):
        """Envía la entrada a OpenAI y devuelve la lista de acciones."""
        prompt = (
            "Eres un asistente que traduce comandos en lenguaje natural en JSON para un TurtleBot. "
            "Devuelve siempre la respuesta en formato JSON válido. "
            "Si no hay ninguna acción, responde con un array vacío []. "
            "Ejemplo de salida esperada:\n"
            "[{\"action\": \"move\", \"distance\": 2}, "
            "{\"action\": \"turn\", \"angle\": 90}, "
            "{\"action\": \"move\", \"distance\": 1}, "
            "{\"action\": \"stop\"}]\n"
            "Notas importantes: - izquierda es ángulo negativo y derecha ángulo positivo\n"
            f"Entrada: '{user_input}'"
        )

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.2
            )
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

    def spin(self):
        """Mantiene el nodo en funcionamiento."""
        rospy.spin()

if __name__ == "__main__":
    chatgpt_processor = ChatGPTProcessor()
    chatgpt_processor.spin()
