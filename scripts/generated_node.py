#!/usr/bin/env python3
from TurtleBotActions import TurtleBotActions
import rospy

rospy.init_node("generated_node", anonymous=True)
tba = TurtleBotActions()

def main():
    # Contexto: la última orden fue "retrocede un poco".
    # Interpretamos "un poco más" como retroceder un poco más.
    distance = 0.25
    speed = 0.12

    try:
        tba.say("Vale, voy a retroceder un poco más.")
        tba.move_backward(distance, speed=speed)
        tba.say("He retrocedido un poco más.")
    except Exception:
        try:
            tba.stop()
        except Exception:
            pass
        try:
            tba.say("No he podido retroceder, me detengo.")
        except Exception:
            pass

if __name__ == "__main__":
    main()

# SUMMARY: Vale, voy a retroceder un poco más.