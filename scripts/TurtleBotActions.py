#!/usr/bin/env python3

# Hay una incompatibilidad entre el amcl node y el follow me, ya que ambos usan la camara y el image to laserscan,
# como hago para que no den conflicto?
# lo mejor seria que el amcl se lanzara desde el principio, y que el follow me no lanzara nada que el amcl ya tenga lanzado
# pero como????

#IMPORTANTEEEEEE!!!!!!!
# he hecho esto ^^^, hay que probarlo:
# en el robot, hay que cambiar el launch del follower_v2 para usar el nuevo que tengo en mi pc guardado
# en el robot, en jorge/scripts/launch_follower.sh, hay que cambiar la linea que lanza el follower.launch por el nuevo, este:
# roslaunch turtlebot_follower follower_v2.launch use_camera:=false


import shlex, subprocess, os, signal, time
from std_msgs.msg import String
import numpy as np
import rospy
import math
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from RobotSpeaker import RobotSpeaker
from DatabaseHandler import DatabaseHandler
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Bool

# necesario para el ssh automatico. el nodo follower debe estar corriendo en el robot
ROBOT_IP   = "10.152.124.172" # ← AQUI HAY QUE PONER LA IP DEL ROBOT, CUIDADO QUE CAMBIA
ROBOT_USER = "turtlebot"  # ← usuario del robot
PASSWORD   = "ros"        # ← contraseña del robot

class TurtleBotActions:

    def __init__(self):
        self.scan_data = None
        self.map_data = None
        self.current_odom_pose = None # coordenadas de odometría
        self.current_map_pose = None # coordenadas del mapa
        self.speaker = RobotSpeaker()

        # Crear una instancia de DatabaseHandler en el hilo principal
        self.db_paths = {
            "database_1": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_1.db", # ESTA DB ES DEL MAPA XXXX
            "database_2": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_2.db", # ESTA DB ES DEL MAPA XXXX
            "database_3": "/home/jorge/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper/database/turtlebot_database_3.db", # ESTA DB ES DEL MAPA XXXX
        }
        self.current_database = "database_1" # Seleccionar la base de datos activa (segun el mapa que este cargado)
        self.db = DatabaseHandler(self.db_paths[self.current_database])

        #SUSCRIPTORES
        rospy.Subscriber('/scan', LaserScan, self.update_scan)
        rospy.Subscriber('/odom', Odometry, self.odom_pose_callback)
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.map_pose_callback, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        #PUBLICADORES
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
        self.follower_state_pub = rospy.Publisher('/follower_state', String, queue_size=10)
        # este es para que el chatgpt sepa si el follower esta activo o no
        self.wall_follower_state_pub = rospy.Publisher('/wall_follower_state', String, queue_size=10)
        # este es para que el nodo wall_follower sepa si tiene que estar activo o no.
        self.wall_follower_enable_pub = rospy.Publisher("/wall_follower/enable", Bool, queue_size=1, latch=True)

        rospy.sleep(1.0)

    def process_laser_data(self, msg):
        """
        Procesa el escaneo láser para obtener distancias mínimas en tres direcciones:
        frente, izquierda y derecha. Equivalente a laser_callback de la versión original.
        """
        num_ranges = len(msg.ranges)
        angle_increment = msg.angle_increment
        center_index = num_ranges // 2
        angle_range_deg = 20
        offset = int((angle_range_deg * math.pi / 180.0) / angle_increment)

        front_indices = msg.ranges[center_index - offset:center_index + offset]
        left_indices = msg.ranges[int(3 * num_ranges / 4):]
        right_indices = msg.ranges[:int(num_ranges / 4)]

        def valid_distances(subset):
            return [d for d in subset if msg.range_min < d < msg.range_max]

        self.min_distance_front = min(valid_distances(front_indices) or [float('inf')])
        self.min_distance_left = min(valid_distances(left_indices) or [float('inf')])
        self.min_distance_right = min(valid_distances(right_indices) or [float('inf')])

    def update_scan(self, msg):
        self.scan_data = msg
        self.process_laser_data(msg)

    def map_callback(self, msg):
        "util para saber que partes del mapa estan exploradas y sin explorar"
        self.map_data = msg

    def map_pose_callback(self, msg: PoseWithCovarianceStamped):
        """Guarda la última estimación de pose de AMCL."""
        self.current_map_pose = msg.pose.pose

    def odom_pose_callback(self, msg):
        self.current_odom_pose = msg.pose.pose

    def stop(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def get_yaw(self, source="odom"):
        """
        Devuelve el yaw (orientación en radianes) desde 'odom' o 'map'.
        :param source: 'odom' o 'map'
        """
        pose = None
        if source == "odom":
            pose = self.current_odom_pose
        elif source == "map":
            pose = self.current_map_pose
        else:
            rospy.logwarn(f"Fuente de yaw desconocida: {source}")
            return 0.0

        if not pose:
            return 0.0

        q = pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        return yaw

    def get_odom_position(self):
        """
        Devuelve la posición (x, y, yaw) usando odometría.
        """
        if not self.current_odom_pose:
            return (0.0, 0.0, 0.0)
        
        x = self.current_odom_pose.position.x
        y = self.current_odom_pose.position.y
        yaw = self.get_yaw(source="odom")
        return (x, y, yaw)
    
    def get_map_position(self):
        if self.current_map_pose is None:
            return None
        x = self.current_map_pose.position.x
        y = self.current_map_pose.position.y
        yaw = self.get_yaw(source="map")
        return (x, y, yaw)

    def compute_distance(self, x0, y0, x1, y1):
        return math.sqrt((x1 - x0)**2 + (y1 - y0)**2)

    def move_forward(self, distance, speed=0.2, obstacle_threshold=0.7):
        if speed > 0.35:
            speed = 0.35
        if not self.current_odom_pose:
            rospy.logwarn("No hay datos de odometría.")
            return False

        x_start, y_start, _ = self.get_odom_position()
        twist = Twist()

        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            front_dist = self.get_front_distance()

            if front_dist < obstacle_threshold:
                self.stop()
                rospy.logwarn(f"Obstáculo detectado a {front_dist:.2f} m, deteniendo avance antes de {distance} m.")
                return False

            x_now, y_now, _ = self.get_odom_position()
            moved = self.compute_distance(x_start, y_start, x_now, y_now)
            if moved >= distance:
                break

            if front_dist < obstacle_threshold + 0.1:
                adjusted_speed = max(0.05, speed * (front_dist / (obstacle_threshold + 0.1)))
                twist.linear.x = adjusted_speed
            else:
                twist.linear.x = speed

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.stop()
        return True
    
    def move_backward(self, distance, speed=0.2):
        """
        Mueve el robot hacia atrás hasta la distancia indicada.
        No hay protección contra obstáculos traseros porque el LIDAR solo cubre 60° frontales.
        """
        if speed > 0.35:
            speed = 0.35

        if not self.current_odom_pose:
            rospy.logwarn("No hay datos de odometría.")
            return False

        x_start, y_start, _ = self.get_odom_position()
        twist = Twist()

        rate = rospy.Rate(10)
        last_distance = None
        no_change_counter = 0
        max_no_change = 10

        while not rospy.is_shutdown():
            # Calcular distancia recorrida
            x_now, y_now, _ = self.get_odom_position()
            moved = self.compute_distance(x_start, y_start, x_now, y_now)

            if moved >= distance:
                break

            # Aplicar velocidad hacia atrás
            twist.linear.x = -abs(speed)
            self.cmd_vel_pub.publish(twist)

            # Protección contra atasco (pequeña distancia recorrida)
            if last_distance is not None and abs(moved - last_distance) < 0.001:
                no_change_counter += 1
                if no_change_counter >= max_no_change:
                    rospy.logwarn("El robot parece atascado durante retroceso. Abortando movimiento.")
                    self.stop()
                    return False
            else:
                no_change_counter = 0

            last_distance = moved
            rate.sleep()

        self.stop()
        return True

    def rotate(self, angle_deg, speed=0.5):
        if speed > 1.0:
            speed = 1.0
        if not self.current_odom_pose:
            rospy.logwarn("No hay datos de odometría.")
            return False

        angle_rad = math.radians(angle_deg)
        direction = 1.0 if angle_rad > 0 else -1.0
        twist = Twist()
        twist.angular.z = direction * abs(speed)

        yaw_prev = self.get_yaw("odom")
        angle_moved = 0.0

        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            current_yaw = self.get_yaw("odom")
            delta = self.normalize_angle(current_yaw - yaw_prev)
            angle_moved += delta
            yaw_prev = current_yaw

            if abs(angle_moved) >= abs(angle_rad):
                break

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.stop()
        return True

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def get_scan_angle_distance(self, target_angle_deg, window=5):
        """Esta función devuelve la distancia promedio medida por el LiDAR hacia un ángulo específico"""
        if not self.scan_data:
            return float('inf')

        angle_rad = math.radians(target_angle_deg)
        index = int(round((angle_rad - self.scan_data.angle_min) / self.scan_data.angle_increment))

        if index < 0 or index >= len(self.scan_data.ranges):
            return float('inf')

        start = max(0, index - window)
        end = min(len(self.scan_data.ranges), index + window + 1)
        values = [r for r in self.scan_data.ranges[start:end] if not math.isinf(r) and not math.isnan(r)]

        if not values:
            return float('inf')

        return sum(values) / len(values)

    def get_front_distance(self):
        return self.get_scan_angle_distance(0)

    def get_right_distance(self):
        if not self.scan_data:
            return float('inf')
        right_deg = math.degrees(self.scan_data.angle_min)
        return self.get_scan_angle_distance(right_deg)

    def get_left_distance(self):
        if not self.scan_data:
            return float('inf')
        left_deg = math.degrees(self.scan_data.angle_max)
        return self.get_scan_angle_distance(left_deg)

    def is_obstacle_ahead(self, threshold=0.7):
        return self.get_front_distance() < threshold
    
    def approach_nearest_obstacle(self, safe_distance=0.50):
        """
        Busca el obstáculo más cercano, se orienta hacia él,
        lo centra finamente y se aproxima hasta una distancia segura.
        Implementado como máquina de estados simple.
        """

        rospy.loginfo("=== APPROACH: buscando obstáculo ===")

        # ---------- STATE: FIND ----------
        try:
            scan = rospy.wait_for_message("/scan", LaserScan, timeout=3.0)
        except rospy.ROSException:
            rospy.logwarn("No se recibió LIDAR.")
            return False

        valid = [(i, d) for i, d in enumerate(scan.ranges)
                if scan.range_min < d < scan.range_max]

        if not valid:
            rospy.logwarn("No hay obstáculos detectables.")
            return False

        min_idx, min_dist = min(valid, key=lambda x: x[1])
        target_angle = scan.angle_min + min_idx * scan.angle_increment
        target_angle_deg = math.degrees(target_angle)

        rospy.loginfo(
            f"Obstáculo a {min_dist:.2f} m, ángulo {target_angle_deg:.1f}°"
        )

        # ---------- STATE: TURN ----------
        if not self.rotate(target_angle_deg):
            rospy.logwarn("Fallo en rotación inicial.")
            return False

        rospy.sleep(0.2)  # dejar estabilizar el scan

        # ---------- STATE: ALIGN ----------
        align_start = rospy.Time.now()
        align_timeout = rospy.Duration(5.0)

        rate = rospy.Rate(10)
        twist = Twist()

        while not rospy.is_shutdown():
            front = self.get_front_distance()
            left = self.get_scan_angle_distance(5)
            right = self.get_scan_angle_distance(-5)

            if not math.isinf(front):
                break  # ya lo tenemos centrado

            # Girar hacia donde esté más cerca
            if left < right:
                twist.angular.z = 0.25
            else:
                twist.angular.z = -0.25

            self.cmd_vel_pub.publish(twist)

            if rospy.Time.now() - align_start > align_timeout:
                rospy.logwarn("Timeout alineando obstáculo.")
                self.stop()
                return False

            rate.sleep()

        self.stop()
        rospy.loginfo("Obstáculo alineado.")

        # ---------- STATE: APPROACH ----------
        approach_start = rospy.Time.now()
        approach_timeout = rospy.Duration(10.0)

        while not rospy.is_shutdown():
            front = self.get_front_distance()

            if math.isnan(front):
                rospy.logwarn("Lectura NaN durante aproximación.")
                self.stop()
                return False

            if front <= safe_distance:
                rospy.loginfo(
                    f"Distancia segura alcanzada ({front:.2f} m)."
                )
                self.stop()
                return True

            if rospy.Time.now() - approach_start > approach_timeout:
                rospy.logwarn("Timeout aproximándose al obstáculo.")
                self.stop()
                return False

            # Velocidad proporcional
            twist.linear.x = 0.2   # velocidad constante
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)

            rate.sleep()

        self.stop()
        return False

    def say(self, text):
        self.speaker.say(text)

#### FUNCIONES DE NAVEGACION CON MAPA (solo con amcl y mapa cargado) ####
    
    def add_place(self, lugar):
        """
        Añade un nuevo lugar a la base de datos. 
        Utiliza las coordenadas actuales del mapa (AMCL).
        """
        rospy.sleep(1.0)

        position = self.get_map_position()
        if position is None:
            rospy.logwarn("No se puede añadir lugar: no hay datos de pose del mapa.")
            return
        x, y, yaw = position
        x = round(x, 2)
        y = round(y, 2)
        yaw = round(yaw, 2)

        self.db.insert_place(lugar, x, y, yaw)
        rospy.loginfo(f"se ha añadido '{lugar}' a la base de datos.")

    def delete_place(self, lugar):
        """
        Elimina un lugar de la base de datos si existe.
        :param lugar: Nombre del lugar a eliminar.
        """
        result = self.db.delete_place(lugar)
        
        if result:
            rospy.loginfo(f"Lugar '{lugar}' eliminado de la base de datos.")
        else:
            rospy.logwarn(f"No se encontró el lugar '{lugar}' en la base de datos.")

    def has_arrived(self, goal_x, goal_y, tolerance=0.3):
        """
        Este método comprueba si el robot ha llegado a la meta.
        """
        pos = self.get_map_position()
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

        x, y, yaw = place
        self._go_to_target(x, y, yaw, destination_name=place_name)

    def go_to_coordinates(self, x, y, yaw=0.0):
        self._go_to_target(x, y, yaw)

    def _go_to_target(self, x, y, yaw=0.0, destination_name=None):
        rospy.loginfo(f"Enviando al robot hacia ({x}, {y}, yaw={yaw})...")

        from tf.transformations import quaternion_from_euler
        from geometry_msgs.msg import Quaternion

        quat = quaternion_from_euler(0, 0, yaw)
        goal_msg = PoseStamped()
        goal_msg.header.stamp = rospy.Time.now()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = x
        goal_msg.pose.position.y = y
        goal_msg.pose.orientation = Quaternion(*quat)

        self.goal_pub.publish(goal_msg)
        rospy.loginfo("Objetivo publicado en /move_base_simple/goal.")

        # Esperar hasta que llegue o se agote el tiempo
        timeout = rospy.Time.now() + rospy.Duration(120)  # 2 minutos máx
        rate = rospy.Rate(1)
        while not rospy.is_shutdown() and rospy.Time.now() < timeout:
            if self.has_arrived(x, y):
                rospy.loginfo("¡El robot ha llegado a la meta!")
                if destination_name:
                    self.speaker.say(f"he llegado al destino {destination_name} con éxito.")
                else:
                    self.speaker.say("he llegado al punto indicado.")
                return
            rate.sleep()

        rospy.logwarn("El robot no llegó a la meta dentro del tiempo esperado.")

#### FOLLOWERS AUTOMATIZADOS: PENDIENTES DE REVISION!!!! ####

    def _ssh_command(self):
        """Comando base ssh con sshpass."""
        return (
            f"sshpass -p {shlex.quote(PASSWORD)} "
            f"ssh -tt -o StrictHostKeyChecking=no "
            f"{ROBOT_USER}@{ROBOT_IP}"
        )

    def _running_remote(self) -> bool:
        """True si follower.launch está corriendo en el robot remoto."""
        chk = subprocess.run(
            f"{self._ssh_command()} pgrep -fa follower",
            shell=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        return bool(chk.stdout.strip())

    def follow_me(self):
        """
        Lanza el modo 'Follow Me' en el robot remoto.
        """
        rospy.loginfo("Activando 'Follow Me'...")

        # Probar SSH
        try:
            result = subprocess.run(
                f"{self._ssh_command()} echo ok",
                shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10
            )
            if result.returncode != 0:
                rospy.logerr("No se pudo conectar vía SSH al robot.")
                self.speaker.say("No puedo conectar con el robot. Verifica que está encendido y conectado.")
                return
        except Exception as e:
            rospy.logerr(f"Error al probar SSH: {e}")
            self.speaker.say("Error inesperado al intentar conectar con el robot.")
            return

        # Comando para abrir terminal y ejecutar el script remoto
        term_cmd = [
            "gnome-terminal", "--", "bash", "-c",
            f"{self._ssh_command()} '~/jorge/scripts/launch_follower.sh'"
        ]

        # Lanzar la terminal
        self.term_proc = subprocess.Popen(term_cmd, preexec_fn=os.setsid)
        rospy.loginfo("Terminal lanzada con el nodo follower. Esperando inicialización...")
        time.sleep(5)  # esperar a que el follower se inicie

        # Comprobar si se está ejecutando el follower en el robot
        if self._running_remote():
            self.follower_state_pub.publish("started")
            rospy.loginfo("'Follow Me' activo.")
            self.speaker.say("Modo seguimiento activado. Ya puedes caminar.")
        else:
            rospy.logerr("El follower no arrancó. Cerrando ventana.")
            self.speaker.say("No se pudo activar 'Follow Me'. Verifica que el robot está listo.")
            if self.term_proc.poll() is None:
                self.term_proc.terminate()

    def stop_follow_me(self):
        """
        Detiene el modo 'Follow Me' en el robot remoto y reinicia la cámara local.
        """
        rospy.loginfo("Deteniendo 'Follow Me'...")

        # Mata el nodo follower remoto
        subprocess.run(
            f"{self._ssh_command()} pkill -f follower.launch",
            shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10
        )

        # Cierra la terminal local si está abierta
        if getattr(self, "term_proc", None) and self.term_proc.poll() is None:
            os.killpg(os.getpgid(self.term_proc.pid), signal.SIGTERM)
            try:
                self.term_proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                self.term_proc.kill()

        # Actualiza estado y reinicia cámara
        self.follower_state_pub.publish("stopped")
        rospy.loginfo("'Follow Me' detenido.")

    def start_corridor_follower(self):
        rospy.loginfo("Activando Wall Follower...")
        self.wall_follower_enable_pub.publish(True)
        self.wall_follower_state_pub.publish("started")

    def stop_corridor_follower(self):
        rospy.loginfo("Deteniendo Wall Follower...")
        self.wall_follower_enable_pub.publish(False)
        self.stop()
        self.wall_follower_state_pub.publish("stopped")




