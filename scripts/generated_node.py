#!/usr/bin/env python3
from TurtleBotActions import TurtleBotActions
import rospy

rospy.init_node("generated_node", anonymous=True)
tba = TurtleBotActions()

try:
    tba.stop()
    tba.say("No tengo emociones reales, pero me gusta ayudarte.")
except Exception:
    pass

# SUMMARY: No tengo emociones, pero estoy aquí para ayudarte.