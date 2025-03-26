#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist, PoseStamped, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from DatabaseHandler import DatabaseHandler

class TurtleBotActions:
    def __init__(self, db):
        self.db = db  # Usamos la base de datos pasada
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.goal_publisher = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)

        # Suscriptor para obtener las coordenadas actuales del robot
        self.current_pose = None
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        
        rospy.sleep(1)  # Dar tiempo a la conexión con el topic


    def odom_callback(self, msg):
        "permite conocer la posicion actual del robot en todo momento"
        # Extraemos la posición del mensaje de Odometry
        self.current_pose = msg.pose.pose
        
    def move(self, distance):
        twist = Twist()
        twist.linear.x = 1 if distance > 0 else -1
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

    def add_place(self, command):
        """
        Añade un nuevo lugar a la base de datos. 
        Utiliza las coordenadas actuales del robot.
        """
        # este tiempo es para asegurar que el robot ya no esta moviendose antes de guardar la ubicacion 
        rospy.sleep(2)

        name = command.get("name")
        
        # Si la pose actual no está disponible, no podemos añadir el lugar
        if self.current_pose is None:
            rospy.logwarn("No se pudo obtener las coordenadas del robot. Asegúrate de que el robot este publicando odometría.")
            return

        # Usar las coordenadas actuales del robot
        x = round(self.current_pose.position.x, 2)
        y = round(self.current_pose.position.y, 2)
        
        if name:
            try:
                # Usar el objeto db para interactuar con la base de datos
                self.db.insert_place(name, x, y)
                rospy.loginfo(f"Nuevo lugar añadido: {name} en las coordenadas ({x}, {y}).")
            except Exception as e:
                rospy.logerr(f"Error al añadir lugar: {e}")
        else:
            rospy.logwarn("Faltan parámetros para añadir el lugar. Asegúrate de incluir nombre.")

    def go_to_place(self, place_name):
        place = self.db.get_place(place_name)
        if place:
            x, y= place
            rospy.loginfo(f"Moviendo al lugar: {place_name} en las coordenadas {x}, {y}.")
            self.actions.move_to_coordinates(x, y)
        else:
            rospy.logwarn(f"Lugar '{place_name}' no encontrado en la base de datos.")

    def stop(self):
        twist = Twist()
        rospy.loginfo("Deteniendo el robot.")
        self.publisher.publish(twist)

    def execute_action(self, twist, duration):
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < duration:
            self.publisher.publish(twist)
            rate.sleep()

        self.stop()  # Detener el robot después del tiempo especificado
