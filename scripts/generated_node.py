#!/usr/bin/env python3

import rospy
from TurtleBotActions import TurtleBotActions

def go_to_office():
    rospy.init_node('go_to_office_node')
    actions = TurtleBotActions()
    
    # Informar al usuario que el robot va a ir al despacho
    actions.say("Voy al despacho.")
    
    # Intentar ir al lugar conocido como "despacho"
    actions.go_to_place("despacho")
    
    # Confirmar llegada al usuario
    actions.say("He llegado al despacho.")

if __name__ == '__main__':
    try:
        go_to_office()
    except rospy.ROSInterruptException:
        pass