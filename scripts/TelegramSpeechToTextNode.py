#!/usr/bin/env python3

import rospy
import os
import telebot
import speech_recognition as sr
from io import BytesIO
from std_msgs.msg import String
from pydub import AudioSegment

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

# Configurar el token de Telegram
telegram_api_key = os.getenv("TELEGRAM_BOT_TOKEN")
if not telegram_api_key:
    rospy.logerr("No se encontró el token de Telegram. Asegúrate de definir la variable de entorno.")
    raise RuntimeError("Token de Telegram no encontrado.")

bot = telebot.TeleBot(telegram_api_key)

class TelegramSpeechToTextNode:

    def __init__(self):
        """
        Inicializa el nodo de ROS para procesar mensajes de voz de Telegram.
        """
        rospy.init_node('telegram_speech_to_text_node', anonymous=True)
        self.pub = rospy.Publisher('/speech_to_text', String, queue_size=10)
        self.recognizer = sr.Recognizer()
        rospy.loginfo("Nodo de Telegram Speech-to-Text iniciado.")

#### PUBLICA LOS MENSAJES EN EL TOPIC /speech_to_text ####

    def process_text(self, text, chat_id):
        """
        Procesa el texto recibido y lo publica en el topic /speech_to_text si contiene la palabra clave "robot".
        :param text: Texto recibido del usuario.
        :param chat_id: ID del chat de Telegram.
        """
        text = text.strip().lower()

        msg = String(data=text)
        self.pub.publish(msg)
        bot.send_message(chat_id, "Orden enviada al chatgpt.")
        rospy.loginfo(f"{GREEN}Mensaje publicado en /speech_to_text: {text}{RESET}")

#### HANDLERS DE TELEGRAM PARA MENSAJES DE TEXTO Y VOZ ####

node = TelegramSpeechToTextNode()

@bot.message_handler(content_types=['text'])
def handle_text(message):
    """
    Maneja los mensajes de texto recibidos por el bot de Telegram.
    :param message: Mensaje de texto recibido.
    """
    rospy.loginfo(f"{YELLOW}Mensaje recibido: {message.text}{RESET}")
    node.process_text(message.text, message.chat.id)

@bot.message_handler(content_types=['voice'])
def handle_voice(message):
    """
    Maneja los mensajes de voz recibidos por el bot de Telegram.
    :param message: Mensaje de voz recibido.
    """
    rospy.loginfo(f"{YELLOW}Mensaje de voz recibido. Procesando...{RESET}")

    # Manejo seguro de descarga de archivos
    try:
        file_info = bot.get_file(message.voice.file_id)
        file = bot.download_file(file_info.file_path)
    except telebot.apihelper.ApiException as e:
        bot.send_message(message.chat.id, "Error al descargar el archivo de audio.")
        rospy.logerr(f"Error al descargar el archivo de Telegram: {e}")
        return

    # Convertir y procesar el audio
    audio_file = "voice.wav"
    try:
        audio = AudioSegment.from_file(BytesIO(file), format="ogg")
        audio.export(audio_file, format="wav")

        with sr.AudioFile(audio_file) as source:
            audio_data = node.recognizer.record(source)
            text = node.recognizer.recognize_google(audio_data, language="es-ES")
            rospy.loginfo(f"Texto reconocido: {text}")
            node.process_text(text, message.chat.id)
    except sr.UnknownValueError:
        bot.send_message(message.chat.id, "No se entendió el audio.")
    except sr.RequestError:
        bot.send_message(message.chat.id, "Error en el reconocimiento de voz.")
    finally:
        os.remove(audio_file)  # Eliminar archivo temporal

#### FUNCION PRINCIPAL PARA INICIAR EL BOT DE TELEGRAM ####

def run_bot():
    """
    Inicia el bot de Telegram y espera mensajes.
    """
    rate = rospy.Rate(10)  # 10 Hz para evitar consumo escesivo de CPU
    rospy.loginfo("Esperando mensajes en Telegram...")
    while not rospy.is_shutdown():
        try:
            bot.polling(none_stop=True, timeout=10)
        except Exception as e:
            rospy.logerr(f"Error en el bot de Telegram: {e}")
        rate.sleep()

if __name__ == "__main__":
    try:
        run_bot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido.")
