#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
from nav_msgs.msg import OccupancyGrid
import heapq

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
        # Si hay obstaculo delante
        if self.min_distance_front < 0.35:
                # Retrocede ligeramente si se encuentra un obstaculo
                self.move(-0.1, 0.2)
                # Gira hacia la direccion con el obstaculo mas lejano
                if self.min_distance_left < self.min_distance_right:
                    self.turn(45)
                else:
                    self.turn(-45)
        else:
            # Si no hay obstaculo delante
            self.move(0.1, 0.2)

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

        rospy.loginfo(f"Enviando al robot hacia {place_name} ({x}, {y})...")
        self.send_goal(x, y, 0)
        
        if self.client.get_state() == actionlib.GoalStatus.SUCCEEDED:
            rospy.loginfo("Robot ha llegado a su destino.")
        else:
            rospy.logwarn("No se pudo llegar al destino.")
      
    def find_frontiers(self, max_frontiers=5):
        """
        Devuelve las 5 fronteras mas prometedoras para la exploracion inteligente
        """
        frontiers = []
        if self.map_data is None:
            return frontiers

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin.position
        data = self.map_data.data

        def is_valid(x, y):
            return 0 <= x < width and 0 <= y < height

        processed_cells = set()

        for y in range(1, height - 1):
            for x in range(1, width - 1):
                idx = x + y * width
                if data[idx] == 0 and idx not in processed_cells:
                    unknown_count = 0
                    for dx in [-1, 0, 1]:
                        for dy in [-1, 0, 1]:
                            nx, ny = x + dx, y + dy
                            n_idx = nx + ny * width
                            if is_valid(nx, ny) and data[n_idx] == -1:
                                unknown_count += 1
                    if unknown_count >= 4:  # Frontera válida
                        wx = origin.x + x * resolution
                        wy = origin.y + y * resolution
                        frontiers.append((wx, wy))
                        processed_cells.add(idx)

        # Limitar a un número máximo de fronteras para procesar
        # Ordenar por distancia euclediana a la posición actual
        frontiers = sorted(frontiers, key=lambda p: math.hypot(p[0] - self.current_pose.position.x, p[1] - self.current_pose.position.y))
        return frontiers[:max_frontiers]
    # este metodo hay que modificarlo, tiene que ir por limite de fronteras en vez de limite de tiempo
    # el tiempo no funciona porque el movebase bloquea el while y deja de contar el tiempo
    def smart_exploration(self, exploration_time=60):
        """
        Explora de forma inteligente priorizando zonas inexploradas cercanas
        """
        rospy.loginfo(f"Iniciando exploración inteligente durante {exploration_time} segundos...")
        start_time = rospy.Time.now().to_sec()  # Tiempo de inicio en segundos

        while not rospy.is_shutdown():
            # Calcular el tiempo transcurrido
            elapsed_time = rospy.Time.now().to_sec() - start_time
            if elapsed_time >= exploration_time:
                rospy.loginfo("Se ha alcanzado el tiempo máximo de exploración. Finalizando.")
                break

            # Buscar fronteras a explorar
            frontiers = self.find_frontiers()
            if not frontiers:
                rospy.loginfo("No se encontraron más zonas desconocidas. Exploración finalizada.")
                break

            # Selección de la frontera más cercana y grande
            if self.current_pose:
                heap = []
                for frontier in frontiers:
                    dist = math.hypot(
                        frontier[0] - self.current_pose.position.x,
                        frontier[1] - self.current_pose.position.y
                    )
                    heapq.heappush(heap, (dist, frontier))

                # Extraer la frontera más cercana
                closest_frontier = heapq.heappop(heap)[1]
                (x, y) = self.offset_target(*closest_frontier)

                # esto es para calcular el giro inicial que debe hacer el robot (arctan)
                yaw = math.atan2(y - self.current_pose.position.y, x - self.current_pose.position.x)
                
                self.send_goal(x, y, yaw)

        rospy.loginfo("Exploración finalizada.")

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

        self.client.send_goal(goal)
        self.client.wait_for_result(timeout=rospy.Duration(30))  # esperar 30s máx

        return self.client.get_state()

    def offset_target(self, x, y, extra=1.0):
        """
        Una vez el robot navega a la frontera inexplorada mas cercana, usando este metodo
        el robot se adentra "extra" metros en la zona inexplorada para obtener mas informacion para el mapa
        """
        if self.current_pose is None:
            return x, y

        dx = x - self.current_pose.position.x
        dy = y - self.current_pose.position.y
        dist = math.hypot(dx, dy)
        if dist == 0:
            return x, y

        nx = dx / dist
        ny = dy / dist

        return x + nx * extra, y + ny * extra
    
    def execute_action(self, twist, duration):
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < duration:
            self.publisher.publish(twist)
            rate.sleep()

        self.stop()  # Detener el robot después del tiempo especificado