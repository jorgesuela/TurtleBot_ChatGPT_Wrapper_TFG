#!/usr/bin/env python3

import subprocess
import time
from tqdm import tqdm
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from RobotSpeaker import RobotSpeaker

"""
CLASE DE ACCIONES DEL TURTLEBOT
Esta clase contiene métodos para controlar el robot TurtleBot, incluyendo
movimiento, giro, adición de lugares a una base de datos y exploración inteligente.
"""

class TurtleBotActions:
    
    def __init__(self, db):
        """
        Topics:
        subscribe: /scan, /odom, /map
        publish: /cmd_vel, /move_base_simple/goal
        param db: Base de datos para almacenar lugares.
        """

        # este es el que mueve el robot real: '/teleop_velocity_smoother/raw_cmd_vel'
        # este es el que mueve el robot gazebo: '/cmd_vel_mux/input/navi'
        self.db = db
        self.publisher = rospy.Publisher('/teleop_velocity_smoother/raw_cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)  # Nuevo publisher

        self.current_pose = None
        self.map_data = None

        # Inicializar el cliente de sonido
        self.speaker = RobotSpeaker()

        # para saber las distancias mínimas en cada dirección
        self.min_distance = float('inf')
        self.min_distance_front = float('inf')
        self.min_distance_left = float('inf')
        self.min_distance_right = float('inf')

        # Suscripciones
        rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        rospy.sleep(1)  # dar tiempo a la conexión con los topics

    def odom_callback(self, msg):
        "permite conocer la posicion actual del robot en todo momento"
        self.current_pose = msg.pose.pose

    def laser_callback(self, msg):
        """Detecta obstáculos en diferentes direcciones."""
        self.min_distance_front = min(min(msg.ranges[:30] + msg.ranges[-30:]), 3.0)  # Frontal
        self.min_distance_left = min(min(msg.ranges[60:120]), 3.0)  # Izquierda
        self.min_distance_right = min(min(msg.ranges[-120:-60]), 3.0)  # Derecha

    def map_callback(self, msg):
        "util para saber que partes del mapa estan exploradas y sin explorar"
        self.map_data = msg

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

    def delete_place(self, command):
        """
        Elimina un lugar de la base de datos si existe.
        :param place_name: Nombre del lugar a eliminar.
        """
        place_name = command.get("name")
        try:
            result = self.db.delete_place(place_name)
            if result:
                rospy.loginfo(self.db.get_all_places())  # debe incluir 'test_place'
                rospy.loginfo(f"Lugar '{place_name}' eliminado de la base de datos.")
            else:
                rospy.logwarn(f"No se encontró el lugar '{place_name}' en la base de datos.")
        except Exception as e:
            rospy.logerr(f"Error al eliminar el lugar: {e}")

    def get_robot_position(self):
        """
        Este metodo se utiliza para saber la posicion actual del robot.
        Es mas fiable que el odom para comprobar si el robot ha llegado a la meta.
        """
        try:
            msg = rospy.wait_for_message('/amcl_pose', PoseWithCovarianceStamped, timeout=5.0)
            pos = msg.pose.pose.position
            return (pos.x, pos.y)
        except rospy.ROSException:
            rospy.logwarn("No se pudo obtener la posición del robot desde /amcl_pose.")
            return None

    def has_arrived(self, goal_x, goal_y, tolerance=0.3):
        """
        Este método comprueba si el robot ha llegado a la meta.
        """
        pos = self.get_robot_position()
        if pos is None:
            return False
        dx = goal_x - pos[0]
        dy = goal_y - pos[1]
        distance = math.hypot(dx, dy)
        rospy.loginfo(f"Distancia actual al objetivo: {distance:.2f} m")
        return distance <= tolerance

    def go_to_place(self, place_name):
        place = self.db.get_place(place_name)
        if place is None:
            rospy.logwarn(f"Lugar '{place_name}' no encontrado en la base de datos.")
            return

        x, y = place
        rospy.loginfo(f"Enviando al robot hacia {place_name} ({x}, {y})...")

        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation.w = 1.0  # Asume orientación hacia adelante

        self.goal_pub.publish(goal_msg)
        rospy.loginfo("Objetivo publicado en /move_base_simple/goal.")

        # Esperar hasta que llegue o se agote el tiempo
        timeout = rospy.Time.now() + rospy.Duration(120)  # 2 minutos máx
        rate = rospy.Rate(1)  # Comprobar cada segundo
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.has_arrived(x, y):
                rospy.loginfo("¡El robot ha llegado a la meta!")
                self.speaker.say(f"he llegado al destino {place_name} con exito.")
                return
            rate.sleep()

        rospy.logwarn("El robot no llegó a la meta dentro del tiempo esperado.")

    
    def custom_recovery(self):          
        if self.min_distance_front < 0.35:
            self.move(-0.1, 0.2)  # Retrocede ligeramente
            if self.min_distance_left < self.min_distance_right:
                self.turn(45)
            else:
                self.turn(-45)
        else:
            self.move(0.1, 0.2)  # Avanza en línea recta con velocidad optimizada     

    def smart_exploration(self, time_limit):
        """
        Explora de forma inteligente usando explore_lite durante un tiempo determinado.
        Termina anticipadamente si no hay metas activas durante al menos 5 segundos seguidos.
        """
        finalizado_antes_de_tiempo = False
        self.last_goal_time = time.time()

        def status_callback(msg):
            # Reiniciamos el temporizador si hay alguna meta activa o pendiente
            if any(status.status in [0, 1] for status in msg.status_list):  # PENDING or ACTIVE
                self.last_goal_time = time.time()

        rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)

        # Lanzar explore_lite
        explore_cmd = f"gnome-terminal -- bash -c \"roslaunch explore_lite explore.launch; exec bash\""
        subprocess.Popen(explore_cmd, shell=True)
        rospy.loginfo("Exploración iniciada. Esperando a que se publiquen fronteras...")

        time.sleep(5)  # Espera inicial para que explore_lite publique su primera meta

        # Bucle con barra de progreso
        for _ in tqdm(range(time_limit), desc="Explorando", ncols=70):
            if time.time() - self.last_goal_time > 5.0:
                finalizado_antes_de_tiempo = True
                break
            time.sleep(1)

        # Terminar exploración
        subprocess.call("pkill -f explore.launch", shell=True)
        if finalizado_antes_de_tiempo:
            rospy.loginfo("No se encontraron mas zonas inexploradas!")
        rospy.loginfo("Exploración terminada.")
    
    def follow_me(self):
        if hasattr(self, 'follower_active') and self.follower_active:
            rospy.logwarn("El modo 'Follow Me' ya está activo internamente.")
            return

        result = subprocess.run(['pgrep', '-f', 'follower.launch'], stdout=subprocess.PIPE)
        if result.stdout:
            rospy.logwarn("Ya hay una instancia del nodo 'follower' ejecutándose externamente.")
            return

        rospy.loginfo("Iniciando el modo 'Follow Me'...")
        follow_cmd = 'gnome-terminal -- bash -c "roslaunch turtlebot_follower follower.launch; exec bash"'
        subprocess.Popen(follow_cmd, shell=True)
        self.follower_active = True
        time.sleep(3)
        rospy.loginfo("'Follow Me' activo.")

    def stop_follow_me(self):
        if not hasattr(self, 'follower_active') or not self.follower_active:
            rospy.logwarn("El modo 'Follow Me' no está activo.")
            return

        # Detener el proceso lanzado con roslaunch follower.launch
        rospy.loginfo("Deteniendo el modo 'Follow Me'...")

        # Esto matará cualquier proceso que contenga 'follower.launch' en su línea de comando
        subprocess.call("pkill -f 'roslaunch turtlebot_follower follower.launch'", shell=True)

        self.follower_active = False
        time.sleep(2)
        rospy.loginfo("'Follow Me' detenido.")

    def execute_action(self, twist, duration):
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < duration:
            self.publisher.publish(twist)
            rate.sleep()

        self.stop()  # Detener el robot después del tiempo especificado