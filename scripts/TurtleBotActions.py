#!/usr/bin/env python3

import subprocess
import time
from tqdm import tqdm
import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from actionlib_msgs.msg import GoalStatusArray

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
        publish: /cmd_vel
        param db: Base de datos para almacenar lugares.
        """
        self.db = db
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)  # Publisher de Markers
        self.current_pose = None
        self.map_data = None

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
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        rospy.sleep(1)  # dar tiempo a la conexión con el topic de odometría

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

    def publish_target_frontier(self, target_frontier):
        """
        Publica la frontera objetivo en RViz como un marcador específico.
        :param target_frontier: Tupla (x, y) representando la frontera objetivo.
        """
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "target_frontier"
        marker.id = 1  # Usamos un ID diferente para diferenciar este marcador
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.scale.x = 0.2  # Tamaño del marcador
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color = ColorRGBA(0.0, 1.0, 0.0, 1.0)  # Color verde para la frontera objetivo
        marker.lifetime = rospy.Duration(0)  # El marker no se elimina automáticamente

        # Definir la posición del marcador
        marker.pose.position.x = target_frontier[0]
        marker.pose.position.y = target_frontier[1]
        marker.pose.position.z = 0
        marker.pose.orientation.w = 1.0

        # Publica el marcador
        self.marker_pub.publish(marker)

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

        rospy.loginfo(f"Enviando al robot hacia {place_name} ({x}, {y})...")
        self.send_goal(x, y, 0)
        
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot ha llegado a su destino.")
        else:
            rospy.logwarn("No se pudo llegar al destino.")
    
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
        self.client.cancel_all_goals()
        if finalizado_antes_de_tiempo:
            rospy.loginfo("No se encontraron mas zonas inexploradas!")
        rospy.loginfo("Exploración terminada.")
    
    def send_goal(self, x, y, yaw):
        "Envia el robot a unas coordenadas especificas con un angulo inicial especifico"
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        q = quaternion_from_euler(0, 0, yaw)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        self.client.cancel_all_goals()  # Cancelar cualquier objetivo anterior
        self.publish_target_frontier((x, y))  # Publicar la frontera objetivo en RViz
        self.client.send_goal(goal)
        
        # Esperamos el resultado durante un máximo de 30 segundos
        self.client.wait_for_result()
        
        # Verificar si el estado es exitoso
        state = self.client.get_state()

        # Si el estado no es un éxito, activamos el comportamiento de recuperación personalizado
        if state != actionlib.GoalStatus.SUCCEEDED:
            self.custom_recovery()  # Llamar al comportamiento de recuperación personalizado

        return state
    
    def execute_action(self, twist, duration):
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < duration:
            self.publisher.publish(twist)
            rate.sleep()

        self.stop()  # Detener el robot después del tiempo especificado
