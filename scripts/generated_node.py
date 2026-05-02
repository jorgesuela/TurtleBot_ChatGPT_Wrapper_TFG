from TurtleBotActions import TurtleBotActions
import rospy


def main():
    robot = TurtleBotActions()
    robot.say("No los lanzo al mismo tiempo porque compruebo el estado de ambos modos antes de actuar y aplico una exclusión mutua: si uno está activo, no activo el otro.")
    robot.stop()


if __name__ == "__main__":
    rospy.init_node("turtlebot_safe_response_node", anonymous=True)
    main()