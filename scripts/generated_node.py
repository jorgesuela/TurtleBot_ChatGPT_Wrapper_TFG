#!/usr/bin/env python3
import rospy
from TurtleBotActions import TurtleBotActions

rospy.init_node("generated_node", anonymous=True)
tba = TurtleBotActions()

def main():
    try:
        tba.say("Hola, estoy bien y listo para ayudarte.")
    except Exception:
        pass

if __name__ == "__main__":
    main()

# SUMMARY: Hola, estoy bien y listo para ayudarte.