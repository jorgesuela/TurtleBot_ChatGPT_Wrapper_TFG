#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler

class TurtleBotActions:
    
    def __init__(self, db):
        """
        Topics:
        subscribe: /scan, /odom
        publish: /cmd_vel
        param db: Base de datos para almacenar lugares.
        """
        self.db = db
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.current_pose = None

        # para saber las distancias mínimas en cada dirección
        self.min_distance = float('inf')
        self.min_distance_front = float('inf')
        self.min_distance_left = float('inf')
        self.min_distance_right= float('inf')

        # Cliente de acción para la navegación
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
        # Suscripciones
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)

        rospy.sleep(1)  # dar tiempo a la conexión con el topic de odometría

    def odom_callback(self, msg):
        "permite conocer la posicion actual del robot en todo momento"
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Detecta obstáculos en diferentes direcciones."""
        self.min_distance_front = min(min(msg.ranges[:30] + msg.ranges[-30:]), 3.0)  # Frontal
        self.min_distance_left = min(min(msg.ranges[60:120]), 3.0)  # Izquierda
        self.min_distance_right = min(min(msg.ranges[-120:-60]), 3.0)  # Derecha

    def move(self, distance, velocity):
        """
        Mueve el robot en línea recta una distancia específica a una velocidad dada.
        :param distance: Distancia en metros a recorrer (positiva hacia adelante, negativa hacia atrás).
        :param velocity: Velocidad lineal en m/s (debe ser positiva, la dirección se maneja con el signo de distance).
        """
        twist = Twist()
        twist.linear.x = velocity if distance > 0 else -velocity  # Ajustar dirección con el signo de la distancia
        duration = abs(distance) / velocity  # Calcular tiempo de movimiento
        
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
    
        self.execute_action(twist, duration)

    def obstacle_avoidance(self):
        """
        Evita obstáculos utilizando la información del láser.
        Si se detecta un obstáculo, el robot retrocede y gira en la dirección opuesta al obstáculo.
        """
        if self.min_distance_front < 0.35:
                self.move(-0.1, 0.2)  # Retrocede ligeramente
                if self.min_distance_left < self.min_distance_right:
                    self.turn(45)
                else:
                    self.turn(-45)
        else:
            self.move(0.1, 0.2)  # Avanza en línea recta con velocidad optimizada

    def stop(self):
        twist = Twist()
        self.publisher.publish(twist)

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

        x = round(self.current_pose.position.x, 2)
        y = round(self.current_pose.position.y, 2)
        
        if name:
            try:
                self.db.insert_place(name, x, y)
                rospy.loginfo(f"Nuevo lugar añadido: {name} en las coordenadas ({x}, {y}).")
            except Exception as e:
                rospy.logerr(f"Error al añadir lugar: {e}")
        else:
            rospy.logwarn("Faltan parámetros para añadir el lugar. Asegúrate de incluir nombre.")

    def go_to_place(self, place_name):
        """
        Navega a un lugar específico utilizando la base de datos.
        :param place_name: Nombre del lugar al que se desea ir.
        """
        # Obtener las coordenadas del lugar desde la base de datos
        place = self.db.get_place(place_name)
        if place is None:
            rospy.logwarn(f"Lugar '{place_name}' no encontrado en la base de datos.")
            return
        
        # Acceder correctamente a las coordenadas, ya que 'place' es una tupla (x, y)
        x = place[0]
        y = place[1]
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"  # Asegúrate de que el marco de referencia es el adecuado
        goal.target_pose.header.stamp = rospy.Time.now()

        # Establece la posición y orientación del objetivo
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        quaternion = quaternion_from_euler(0, 0, 0)  # Suponiendo que la orientación es cero, puedes cambiarla si lo necesitas
        goal.target_pose.pose.orientation.x = quaternion[0]
        goal.target_pose.pose.orientation.y = quaternion[1]
        goal.target_pose.pose.orientation.z = quaternion[2]
        goal.target_pose.pose.orientation.w = quaternion[3]

        rospy.loginfo(f"Enviando al robot hacia {place_name} ({x}, {y})...")
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot ha llegado a su destino.")
        else:
            rospy.logwarn("No se pudo llegar al destino.")

        
    def explore_environment(self, exploration_time=300):
        """
        Inicia la exploración del entorno durante un tiempo específico utilizando SLAM.
        :param exploration_time: Tiempo en segundos para explorar.
        """
        rospy.loginfo(f"Iniciando exploración durante {exploration_time} segundos.")
        
        # Movimiento continuo para explorar, evitando obstáculos
        start_time = rospy.Time.now().to_sec()
        while rospy.Time.now().to_sec() - start_time < exploration_time:
            self.obstacle_avoidance()

        rospy.loginfo("Exploración completada.")
    

    def execute_action(self, twist, duration):
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < duration:
            self.publisher.publish(twist)
            rate.sleep()

        self.stop()  # Detener el robot después del tiempo especificado
        #rospy.sleep(1) # Para evitar que se mueva mientras hace otras acciones
