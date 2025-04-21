#!/usr/bin/env python3

import rospy
import asyncio
import websockets
import os
from std_msgs.msg import String
from io import BytesIO
from pydub import AudioSegment
import speech_recognition as sr
import socket

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

PORT = 5005  # Nuevo puerto WebSocket

class WSSpeechToTextNode:
    def __init__(self):
        rospy.init_node('ws_speech_to_text_node', anonymous=True)
        self.pub = rospy.Publisher('/speech_to_text', String, queue_size=10)
        self.recognizer = sr.Recognizer()
        rospy.loginfo("Nodo WebSocket Speech-to-Text iniciado.")

    def get_local_ip(self):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            # No se necesita conexión real, solo se usa para determinar IP local
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
        except Exception:
            ip = "127.0.0.1"
        finally:
            s.close()
        return ip

    def process_text(self, text):
        # Eliminar la cabecera "txt robot " si existe
        if text.lower().startswith("txt robot "):
            text = text[10:].strip()  # Elimina "txt robot " y los posibles espacios al principio
        
        text = text.strip().lower()
        if "robot" in text:
            msg = String(data=text)
            self.pub.publish(msg)
            rospy.loginfo(f"{GREEN}Mensaje publicado en /speech_to_text: {text}{RESET}")
        else:
            rospy.logwarn("Mensaje ignorado, debe contener la palabra clave 'robot'.")

    def process_audio(self, audio_bytes):
        temp_file = "/tmp/input.wav"
        try:
            audio = AudioSegment.from_file(BytesIO(audio_bytes), format="webm")
            audio.export(temp_file, format="wav")

            with sr.AudioFile(temp_file) as source:
                audio_data = self.recognizer.record(source)
                text = self.recognizer.recognize_google(audio_data, language="es-ES")
                rospy.loginfo(f"{YELLOW}Texto reconocido de audio: {text}{RESET}")
                self.process_text(text)
        except sr.UnknownValueError:
            rospy.logwarn("No se pudo entender el audio.")
        except sr.RequestError as e:
            rospy.logerr(f"Error de reconocimiento de voz: {e}")
        except Exception as e:
            rospy.logerr(f"Error al procesar audio: {e}")
        finally:
            if os.path.exists(temp_file):
                os.remove(temp_file)

    async def handle_client(self, websocket, path):
        try:
            async for message in websocket:
                if isinstance(message, bytes):
                    try:
                        # Intentamos decodificar como texto primero
                        decoded_text = message.decode('utf-8')
                        self.process_text(decoded_text)
                    except UnicodeDecodeError:
                        # Si falla la decodificación, asumimos que es audio
                        self.process_audio(message)
                else:
                    rospy.loginfo(f"{YELLOW}Texto recibido por WS: {message}{RESET}")
                    self.process_text(message)
        except Exception as e:
            rospy.logerr(f"Error en la conexión WebSocket: {e}")

    def start_server(self):
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        start_server = websockets.serve(self.handle_client, "0.0.0.0", PORT, max_size=None)
        local_ip = self.get_local_ip()
        rospy.loginfo(f"Servidor WebSocket iniciado en: {GREEN}ws://{local_ip}:{PORT}{RESET}")
        rospy.loginfo("Esperando mensajes desde SpeakLink ...")
        loop.run_until_complete(start_server)
        loop.run_forever()

if __name__ == "__main__":
    try:
        node = WSSpeechToTextNode()
        node.start_server()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo interrumpido.")
