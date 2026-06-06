#!/usr/bin/env python3
from TurtleBotActions import TurtleBotActions
import rospy

rospy.init_node("generated_node", anonymous=True)
tba = TurtleBotActions()

try:
    tba.say("Qué bien, suena genial que a la tuya le encante nadar. A veces cada perra tiene sus gustos; ¿la llevas más a piscina, río o playa?")
except Exception:
    pass

# SUMMARY: Qué bien que a la tuya le encante nadar; ¿la llevas más a piscina, río o playa?