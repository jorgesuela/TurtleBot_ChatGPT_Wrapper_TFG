#!/usr/bin/env python3
from TurtleBotActions import TurtleBotActions
import rospy

rospy.init_node("generated_node", anonymous=True)
tba = TurtleBotActions()

def safe_say(text):
    try:
        tba.say(text)
    except Exception:
        pass

def main():
    start_pose = {"x": 5.42, "y": -10.53, "yaw": 0.0}

    safe_say("Vale, voy a la papelera y luego vuelvo aquí.")

    success_bin = False
    success_return = False

    try:
        success_bin = bool(tba.go_to_place("papelera"))
    except Exception:
        success_bin = False

    if success_bin:
        safe_say("He llegado a la papelera. Ahora vuelvo al punto de inicio.")
    else:
        safe_say("No he podido llegar a la papelera, pero voy a intentar volver al punto de inicio.")

    try:
        success_return = bool(tba.go_to_coordinates(start_pose["x"], start_pose["y"], start_pose["yaw"]))
    except Exception:
        success_return = False

    if success_return:
        safe_say("Ya he vuelto aquí.")
    else:
        safe_say("No he podido volver exactamente, pero me he detenido de forma segura.")
        try:
            tba.stop()
        except Exception:
            pass

if __name__ == "__main__":
    main()

# SUMMARY: Vale, voy a la papelera y luego vuelvo aquí.