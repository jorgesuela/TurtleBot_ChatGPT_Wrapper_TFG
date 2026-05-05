#!/usr/bin/env python3
from TurtleBotActions import TurtleBotActions
import rospy

rospy.init_node("generated_node", anonymous=True)
tba = TurtleBotActions()

try:
    tba.say("Vale, voy a darme la vuelta.")
    tba.stop()
    rospy.sleep(0.5)
    tba.rotate(180, speed=0.5)
    tba.stop()
except Exception as e:
    try:
        tba.stop()
        tba.say("No he podido completar el giro de forma segura.")
    except Exception:
        pass

# SUMMARY: Vale, voy a darme la vuelta.