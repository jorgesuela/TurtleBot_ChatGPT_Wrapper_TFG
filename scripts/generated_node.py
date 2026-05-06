#!/usr/bin/env python3
from TurtleBotActions import TurtleBotActions
import rospy

rospy.init_node("generated_node", anonymous=True)
tba = TurtleBotActions()

def main():
    try:
        tba.say("Entendido, voy a seguir la pared ahora.")
        tba.start_wall_follower()
        rospy.sleep(0.5)
        tba.say("He activado el seguimiento de pared.")
    except Exception:
        try:
            tba.stop()
            tba.say("No he podido activar el seguimiento de pared.")
        except Exception:
            pass

if __name__ == "__main__":
    main()

# SUMMARY: Entendido, voy a seguir la pared ahora.