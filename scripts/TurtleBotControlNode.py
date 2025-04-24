#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from TurtleBotActions import TurtleBotActions
from DatabaseHandler import DatabaseHandler

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

"""
NODO CONTROL DEL TURTLEBOT
Este nodo se encarga de recibir comandos desde el topic /turtlebot_single_action
y ejecutar las acciones correspondientes en el TurtleBot.
"""

class TurtleBotControlNode:

    def __init__(self):
        """
            Topics:
            subscribe: /turtlebot_single_action
            publish:   none
            """
        rospy.init_node('turtlebot_control_node', anonymous=True)
        
        # Crear una instancia de DatabaseHandler en el hilo principal
        self.db = DatabaseHandler('/home/jorge/catkin_ws/src/turtlebot_chatgpt_wrapper/places.db')
        self.db.create_table()  # Asegúrate de que la tabla exista
        
        # Pasar el objeto de base de datos a TurtleBotActions
        self.actions = TurtleBotActions(self.db)
        
        self.subscription = rospy.Subscriber('/turtlebot_single_action', String, self.execute_command)
        rospy.loginfo("Nodo de control del TurtleBot iniciado. Esperando comandos...")

    def execute_command(self, msg):
        """
        Callback para el topic /turtlebot_single_action.
        Recibe un mensaje JSON con la acción a ejecutar y llama a la función correspondiente.
        """
        try:
            command_list = json.loads(msg.data)
            command = command_list[0] if isinstance(command_list, list) else command_list
            
            rospy.loginfo(f"{YELLOW}Ejecutando acción: {command}{RESET}")
            self.process_single_action(command)
            rospy.loginfo(f"{GREEN}Acción ejecutada.{RESET}")
            rospy.loginfo("Esperando mas comandos...")
        except Exception as e:
            rospy.logerr(f"Error al ejecutar el comando: {e}")
            rospy.loginfo("Esperando mas comandos...")

    def process_single_action(self, command):
        """
        Procesa una acción individual recibida desde el topic /turtlebot_single_action.
        Se encarga de llamar a la función correspondiente en TurtleBotActions.
        """
        try:
            # diccionario de acciones con su clave "nombre" y su valor "funcion asociada"
            action_map = {
                "move": lambda cmd: self.actions.move(cmd.get("distance", 0), cmd.get("velocity", 0)),
                "turn": lambda cmd: self.actions.turn(cmd.get("angle", 0)),
                "stop": lambda cmd: self.actions.stop(),
                "go_to_place": lambda cmd: self.actions.go_to_place(cmd.get("place")),
                "add_place": lambda cmd: self.actions.add_place(cmd),
                "explore": lambda cmd: self.actions.smart_exploration(cmd.get("time_limit"))
            }
            action_type = command.get("action")
            if action_type in action_map:
                action_map[action_type](command)
            else:
                rospy.logwarn(f"Acción '{action_type}' no reconocida.")
        except Exception as e:
            rospy.logerr(f"Error al procesar la acción: {e}")

if __name__ == "__main__":
    control = TurtleBotControlNode()
    rospy.spin()

