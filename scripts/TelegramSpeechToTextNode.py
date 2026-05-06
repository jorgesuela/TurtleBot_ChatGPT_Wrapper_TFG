#!/usr/bin/env python3

import rospy
import os
import telebot
import speech_recognition as sr
from io import BytesIO
from DatabaseHandler import DatabaseHandler
from std_msgs.msg import String
from pydub import AudioSegment
import re
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf.transformations
from TurtleBotActions import TurtleBotActions

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

        self.db_paths = {
            "database_1": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_1.db",
            "database_2": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_2.db",
            "database_3": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_3.db",
        }

        self.current_database = "database_1"
        self.db = DatabaseHandler(self.db_paths[self.current_database])

        self.pose = None
        self.follow_me_state = "stopped"
        self.wall_follower_state = "stopped"
        self.stop_commands = {"para", "parate", "detente", "quieto", "quedate ahi", "parate ya", "quedate quieto", "vale para", "vale parate ahi"}
        self.tba = TurtleBotActions()

        self.pub = rospy.Publisher('/speech_to_text', String, queue_size=10)
        rospy.Subscriber('/follower_state', String, self.follow_me_callback)
        rospy.Subscriber('/wall_follower_state', String, self.wall_follower_callback)
        rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, self.update_pose)

        self.recognizer = sr.Recognizer()

        rospy.loginfo("Nodo de Telegram Speech-to-Text iniciado.")

#### CALLBACKS PARA MANEJAR PARADA ANTICIPADA ####

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

    def follow_me_callback(self, msg):
        self.follow_me_state = msg.data

    def wall_follower_callback(self, msg):
        self.wall_follower_state = msg.data

    def serialize_pose(self):
        if not self.pose:
            return None
        return self.pose

#### INTERCEPTOR DE PARADA ####

    def handle_stop_command(self):

        rospy.logwarn(f"{YELLOW}COMANDO DE PARADA ANTICIPADA DETECTADO{RESET}")

        # 🔴 CASO 1: FOLLOW ME
        if self.follow_me_state == "started":
            rospy.loginfo("Parando Follow Me anticipadamente")
            self.tba.stop_follow_me()

            self.db.insert_user_request(
                    "deja de seguirme",
                    "he dejado de seguirte",
                    self.serialize_pose()
                )
            self.tba.say("He dejado de seguirte.")
            return

        # 🔴 CASO 2: WALL FOLLOWER
        if self.wall_follower_state == "started":
            rospy.loginfo("Parando Wall Follower anticipadamente")
            self.tba.stop_wall_follower()

            self.db.insert_user_request(
                    "deja de seguir la pared",
                    "he dejado de seguir la pared",
                    self.serialize_pose()
                )
            self.tba.say("He dejado de seguir la pared.")
            return

        # 🔴 CASO 3: STOP GENERAL
        rospy.loginfo("Parada general anticipadamente")
        self.tba.stop()

        self.db.insert_user_request(
                    "para",
                    "voy a detenerme",
                    self.serialize_pose()
                )
        self.tba.say("Me detengo.")

#### PUBLICA LOS MENSAJES EN EL TOPIC /speech_to_text ####

    def process_text(self, text, chat_id):
        text = text.strip().lower()
        
        # 🔴 Interceptor de parada anticipada
        if text in self.stop_commands:
            self.handle_stop_command()
            return

        # 🟢 Flujo normal
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
