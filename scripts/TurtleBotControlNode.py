#!/usr/bin/env python3

import rospy
import json
from std_msgs.msg import String
from TurtleBotActions import TurtleBotActions

GREEN = "\033[92m"
YELLOW = "\033[93m"
RESET = "\033[0m"

class TurtleBotControlNode:
    def __init__(self):
        rospy.init_node('turtlebot_control_node', anonymous=True)
        self.actions = TurtleBotActions()
        self.subscription = rospy.Subscriber('/turtlebot_single_action', String, self.execute_command)
        rospy.loginfo("Nodo de control del TurtleBot iniciado. Esperando comandos...")

    def execute_command(self, msg):
        try:
            command = json.loads(msg.data)
            rospy.loginfo(f"{YELLOW}Ejecutando acción: {command}{RESET}")
            self.process_single_action(command)
            rospy.loginfo(f"{GREEN}Acción ejecutada.{RESET}")
        except Exception as e:
            rospy.logerr(f"Error al ejecutar el comando: {e}")

    def process_single_action(self, command):
        try:
            action_map = {
                "move": lambda cmd: self.actions.move(cmd.get("distance", 0)),
                "turn": lambda cmd: self.actions.turn(cmd.get("angle", 0)),
                "stop": lambda cmd: self.actions.stop()
            }
            action_type = command.get("action")
            if action_type in action_map:
                action_map[action_type](command)
            else:
                rospy.logwarn(f"Acción '{action_type}' no reconocida.")
        except Exception as e:
            rospy.logerr(f"Error al procesar la acción: {e}")

    def spin(self):
        rospy.spin()

if __name__ == "__main__":
    control = TurtleBotControlNode()
    control.spin()