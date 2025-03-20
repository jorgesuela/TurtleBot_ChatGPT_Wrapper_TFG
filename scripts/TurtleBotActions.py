#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist

class TurtleBotActions:
    def __init__(self):
        # Para probar en turtlesim el topic es /turtle1/cmd_vel o /cmd_vel para gazebo
        self.publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        rospy.sleep(1)  # Dar tiempo a la conexión con el topic

    def move(self, distance):
        """
        Mueve el robot hacia adelante o hacia atrás.
        :param distance: Distancia en metros para mover el robot (positiva = adelante, negativa = atrás)
        """
        twist = Twist()
        twist.linear.x = 1 if distance > 0 else -1  # Velocidad fija
        duration = abs(distance) / 1  # Tiempo de movimiento

        rospy.loginfo(f"Moviendo {'adelante' if distance > 0 else 'atrás'} {abs(distance)} metros.")
        self.execute_action(twist, duration)

    def turn(self, angle):
        """
        Gira el robot a la izquierda o derecha con una velocidad angular constante.
        :param angle: Ángulo en grados para girar el robot (positivo = derecha, negativo = izquierda).
        """
        twist = Twist()
        
        angular_speed = 1.0  # rad/s
        angle_rad = math.radians(abs(angle))  # Convertir grados a radianes
        duration = angle_rad / angular_speed  # Tiempo necesario para girar

        twist.angular.z = angular_speed if angle < 0 else -angular_speed  
        rospy.loginfo(f"Girando {'izquierda' if angle < 0 else 'derecha'} {abs(angle)} grados en {duration:.2f} segundos.")

        self.execute_action(twist, duration)

    def stop(self):
        """
        Detiene el robot.
        """
        twist = Twist()
        rospy.loginfo("Deteniendo el robot.")
        self.publisher.publish(twist)

    def execute_action(self, twist, duration):
        """
        Publica un mensaje Twist durante un tiempo determinado.
        :param twist: Mensaje Twist con la acción a realizar.
        :param duration: Tiempo en segundos que debe ejecutarse la acción.
        """
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < duration:
            self.publisher.publish(twist)
            rate.sleep()

        self.stop()  # Detener el robot después del tiempo especificado
