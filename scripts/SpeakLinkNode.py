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

"""
NODO SPEAKLINK
Este nodo se encarga de recibir mensajes de texto y audio a través de un WebSocket.
Los mensajes de texto se publican en el topic /speech_to_text.
Los mensajes de audio se convierten a texto y también se publican en el mismo topic.
El nodo utiliza la librería SpeechRecognition para el reconocimiento de voz y pydub para la manipulación de audio.
"""

class WSSpeechToTextNode:
    def __init__(self):
        rospy.init_node('ws_speech_to_text_node', anonymous=True)
        self.pub = rospy.Publisher('/speech_to_text', String, queue_size=10)
        self.recognizer = sr.Recognizer()
        rospy.loginfo("Nodo WebSocket Speech-to-Text iniciado.")

    def get_local_ip(self):
        """
        Obtiene la IP local del dispositivo para mostrarla por pantalla
        al iniciar el nodo y que sea mas facil configurar la app.
        """
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
        """
        Procesa el texto recibido, y lo publica en el topic /speech_to_text"
        """
        # Eliminar la cabecera "txt robot " si existe
        if text.lower().startswith("txt robot "):
            text = text[10:].strip()  # Elimina "txt robot " y los posibles espacios al principio
        
        text = text.strip().lower()
        msg = String(data=text)
        self.pub.publish(msg)
        rospy.loginfo(f"{GREEN}Mensaje publicado en /speech_to_text: {text}{RESET}")


    def process_audio(self, audio_bytes):
        """
        Procesa el audio recibido, lo convierte a texto y lo publica en el topic /speech_to_text
        """
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
        """
        Maneja la conexión del cliente WebSocket.
        Recibe mensajes de texto o audio y los procesa.
        """
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
        except websockets.exceptions.ConnectionClosed as e:
            rospy.logwarn(f"Conexión cerrada correctamente: {e}")
        except Exception as e:
            rospy.logerr(f"Error en la conexión WebSocket: {e}")
        finally:
            await websocket.close()  # Asegura que se cierra el WebSocket

    def start_server(self):
        """
        Inicia el servidor WebSocket y espera conexiones de clientes.
        """
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
