#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from TurtleBotActions import TurtleBotActions

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

class TurtleBotControlNode:
    def __init__(self):
        """
        Inicializa el nodo de control del TurtleBot.
        """
        # Inicializar el nodo ROS
        rospy.init_node('turtlebot_control_node', anonymous=True)

        self.actions = TurtleBotActions()  # Asegúrate de que esta línea esté presente

        # Suscripción al topic /turtlebot_commands para recibir comandos JSON de la IA
        self.subscription = rospy.Subscriber(
            '/turtlebot_commands',  # Nombre del topic
            String,                 # Tipo de mensaje (String)
            self.execute_commands   # Función que procesa el comando recibido
        )

        rospy.loginfo("Nodo de control del TurtleBot iniciado. Esperando comandos...")

    def execute_commands(self, msg):
        """Recibe un mensaje JSON con una o varias acciones y las ejecuta secuencialmente.
           param msg: Mensaje JSON con una o varias acciones a ejecutar.
        """
        try:
            # Asegúrate de que msg.data (el contenido del mensaje) sea un string
            command_data = json.loads(msg.data)  # Decodificar el string JSON en un diccionario o lista

            rospy.loginfo(f"{YELLOW}Comando recibido: \n{command_data}{RESET}")

            # Para asegurar que command_data no sea un set, convierte a lista si es necesario
            if isinstance(command_data, set):
                command_data = list(command_data)  # Convertir set a lista

            if isinstance(command_data, list):
                actions = command_data  # Lista de acciones
            else:
                actions = [command_data]  # Si solo hay una acción, convertirla en lista

            # Ejecutar cada acción en secuencia
            rospy.loginfo(f"{YELLOW}Ejecutando acciones...{RESET}")
            for action_cmd in actions:
                self.process_single_action(action_cmd)
            rospy.loginfo(f"{GREEN}Ejecución de acciones completada.{RESET}")

        except json.JSONDecodeError:
            rospy.logerr("Error al parsear el JSON recibido.")
        except KeyError as e:
            rospy.logerr(f"Error de clave en el JSON: {e}")
        except Exception as e:
            rospy.logerr(f"Error al ejecutar el comando: {e}")

    def process_single_action(self, command):
        """Procesa una sola acción del JSON.
           param command: acción a ejecutar
        """
            
        """Diccionario de acciones que el robot es capaz de realizar"""
        try:
            action_map = {
                "move": lambda cmd: self.actions.move(cmd.get("distance", 0)),
                "turn": lambda cmd: self.actions.turn(cmd.get("angle", 0)),
                "stop": lambda cmd: self.actions.stop()
            }

            action_type = command.get("action")
            if action_type not in action_map:
                rospy.logwarn(f"Acción '{action_type}' no reconocida.")
                return

            rospy.loginfo(f"Ejecutando acción: {command}")

            # Ejecutar la acción correspondiente
            action_map[action_type](command)

        except KeyError as e:
            rospy.logerr(f"Faltan parámetros en la acción: {e}")
        except Exception as e:
            rospy.logerr(f"Error al procesar la acción: {e}")

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    # Crear instancia del nodo de control y ejecutar
    control = TurtleBotControlNode()
    control.spin()
