#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from move_base_msgs.msg import MoveBaseGoal

class TurtleBotActions:
    
    def __init__(self, db):
        """
        Topics:
        subscribe: /scan, /odom
        publish: /cmd_vel
        """
        self.db = db
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_pose = None
        self.min_distance = float('inf')
        
        # Suscripciones
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        rospy.sleep(1)  # dar tiempo a la conexión con el topic de odometría

    def odom_callback(self, msg):
        "permite conocer la posicion actual del robot en todo momento"
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Actualiza la distancia mínima detectada por el LiDAR."""
        self.min_distance = min(msg.ranges)
        
    def move(self, distance, velocity):
        """
        Mueve el robot en línea recta una distancia específica a una velocidad dada.
        :param distance: Distancia en metros a recorrer (positiva hacia adelante, negativa hacia atrás).
        :param velocity: Velocidad lineal en m/s (debe ser positiva, la dirección se maneja con el signo de distance).
        """
        twist = Twist()
        twist.linear.x = velocity if distance > 0 else -velocity  # Ajustar dirección con el signo de la distancia
        duration = abs(distance) / velocity  # Calcular tiempo de movimiento
        
        rospy.loginfo(f"Moviendo {'adelante' if distance > 0 else 'atrás'} {abs(distance)} metros a {velocity} m/s.")
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
        # Este tiempo es para asegurar que el robot ya no está moviéndose antes de guardar la ubicación
        rospy.sleep(2)

        name = command.get("name")
        
        # Si la pose actual no está disponible, no podemos añadir el lugar
        if self.current_pose is None:
            rospy.logwarn("No se pudo obtener las coordenadas del robot. Asegúrate de que el robot esté publicando odometría.")
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
            x, y = place
            rospy.loginfo(f"Moviendo al lugar: {place_name} en las coordenadas {x}, {y}.")
            
            # Crear un mensaje de acción para mover el robot hacia las coordenadas
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"  # Usar el sistema de coordenadas 'map'
            goal.target_pose.header.stamp = rospy.Time.now()

            goal.target_pose.pose.position.x = x
            goal.target_pose.pose.position.y = y
            goal.target_pose.pose.orientation.w = 1.0  # Orientación neutral (sin giro)

            # Enviar la meta a move_base
            self.client.send_goal(goal)
            self.client.wait_for_result()

            # Obtener el resultado de la acción
            result = self.client.get_result()
            if result:
                rospy.loginfo(f"Robot ha llegado al lugar {place_name}.")
            else:
                rospy.logwarn(f"El robot no ha llegado al lugar {place_name}.")
        else:
            rospy.logwarn(f"Lugar '{place_name}' no encontrado en la base de datos.")

    def explore_environment(self, exploration_time=90):
        """
        Explora el entorno evitando obstáculos.
        :param exploration_time: Tiempo total de exploración en segundos.
        """
        start_time = rospy.Time.now().to_sec()
        rospy.loginfo("Iniciando exploración del entorno.")

        while (rospy.Time.now().to_sec() - start_time) < exploration_time:
            if self.min_distance < 0.5:  # Si hay un obstáculo a menos de 0.5m
                rospy.loginfo("Obstáculo detectado a " + str(self.min_distance) + " girando...")
                self.turn(90)  # Girar 90 grados
            else:
                self.move(1, 0.2)  # Moverse 1 metro a 0.2 m/s
            rospy.sleep(1)

        rospy.loginfo("Exploración finalizada.")
        self.stop()

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
        rospy.sleep(1) # Para evitar que se mueva mientras hace otras acciones
