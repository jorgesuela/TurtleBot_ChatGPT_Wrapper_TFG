#!/usr/bin/env python3

import rospy
from TurtleBotActions import TurtleBotActions

class WallApproachNode:
    def __init__(self):
        rospy.init_node('wall_approach_node')
        self.tb_actions = TurtleBotActions()
        self.tb_actions.say("Voy a avanzar hasta la pared más cercana.")
        self.approach_wall()

    def approach_wall(self):
        success = self.tb_actions.approach_nearest_obstacle()
        if success:
            self.tb_actions.say("He llegado a la pared.")
        else:
            self.tb_actions.say("No he podido acercarme a la pared de forma segura.")

if __name__ == '__main__':
    try:
        WallApproachNode()
    except rospy.ROSInterruptException:
        pass