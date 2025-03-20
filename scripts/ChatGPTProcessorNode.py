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
        """
        Inicializa el nodo de procesamiento de mensajes de voz con ChatGPT.
        """
        rospy.init_node('chatgpt_processor_node', anonymous=True)

        # Obtener la API Key de openai
        api_key = os.getenv("OPENAI_API_KEY")
        if not api_key:
            rospy.logerr("No se encontró la API Key de OpenAI. Asegúrate de definir la variable de entorno.")
            raise RuntimeError("API Key de OpenAI no encontrada.")

        # Configurar OpenAI
        self.client = OpenAI(api_key=api_key)
        self.model = os.getenv("OPENAI_MODEL", "gpt-3.5-turbo")  # Se utiliza el modelo básico por temas de costos

        # Suscripción al topic de voz, por el que recibira los mensajes del usuario
        self.subscription = rospy.Subscriber('/speech_to_text', String, self.process_speech_input)

        # Publicador de comandos JSON, que envia los resultados de la ia al nodo de control del robot
        self.publisher = rospy.Publisher('/turtlebot_commands', String, queue_size=10)

        rospy.loginfo("Nodo ChatGPTProcessor iniciado. Esperando mensajes...")

    def process_speech_input(self, msg):
        """
        Procesa el mensaje de voz del usuario con la ia y envía la respuesta al nodo de control del robot.
        :param msg: Mensaje de voz del usuario.
        """
        user_input = msg.data.strip().lower()

        rospy.loginfo(f"{YELLOW}Procesando mensaje: {user_input}{RESET}")

        # Contexto para la IA + mensaje del usuario
        prompt = (
            "Eres un asistente que traduce comandos en lenguaje natural en JSON para un TurtleBot. "
            "Devuelve siempre la respuesta en formato JSON válido. "
            "Si no hay ninguna acción, responde con un array vacío []. "
            "Ejemplo de salida esperada:\n"
            "[{\"action\": \"move\", \"distance\": 2}, "
            "{\"action\": \"turn\", \"angle\": 90}, "
            "{\"action\": \"move\", \"distance\": 1}, "
            "{\"action\": \"stop\"}]\n"
            "Notas importantes: - izquierda es angulo negativo y derecha angulo positivo\n"
            f"Entrada: '{user_input}'"
        )

        try:
            response = self.client.chat.completions.create(
                model=self.model,
                messages=[{"role": "user", "content": prompt}],
                temperature=0.2  # Respuestas más predecibles
            )
            json_response = response.choices[0].message.content

            # Validar que la salida sea JSON válido
            try:
                parsed_json = json.loads(json_response)
                if not isinstance(parsed_json, list):
                    rospy.logerr("La respuesta de ChatGPT no es un array JSON válido.")
                    return
            except json.JSONDecodeError:
                rospy.logerr("Error: ChatGPT devolvió una respuesta no válida.")
                return

            # Publicar comandos JSON
            output_msg = String()
            output_msg.data = json_response
            self.publisher.publish(output_msg)
            rospy.loginfo(f"{GREEN}Comando enviado al nodo de control del robot:{RESET} \n{json_response}")


        except OpenAIError as e:
            rospy.logerr(f"Error de OpenAI: {e}")
        except Exception as e:
            rospy.logerr(f"Error inesperado: {e}")

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    chatgpt_processor = ChatGPTProcessor()
    chatgpt_processor.spin()
