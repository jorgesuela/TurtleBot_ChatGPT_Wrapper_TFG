#!/usr/bin/env python3

import rospy
from TurtleBotActions import TurtleBotActions

def main():
    rospy.init_node('follow_me_node')
    actions = TurtleBotActions()

    # Check if Follow Me mode is already started
    follower_state = 'stopped'  # Assume the state based on the context provided

    if follower_state == 'stopped':
        actions.follow_me()
        actions.say("Activando el modo Follow Me. Ahora te seguiré.")
    else:
        actions.say("Ya estaba en modo Follow Me.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass