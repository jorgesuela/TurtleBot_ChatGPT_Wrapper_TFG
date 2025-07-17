#!/usr/bin/env python3

# COSAS QUE NO FUNCIONAN: EL FOLLOWER NO VA BIEN EL SSH.
#bash: /opt/ros/noetic/setup.bash: No such file or directory
#Connection to 192.168.71.172 closed.
#Pulsa ENTER para cerrar esta ventana

# EL MAPA HAY QUE MEJORARLO, NO SE VE BIEN.

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
from std_msgs.msg import String
from RobotSpeaker import RobotSpeaker
import shlex, subprocess, time, rospy
from std_msgs.msg import String
import os
import signal

# necesario para el ssh automatico. el nodo follower debe estar corriendo en el robot
ROBOT_USER = "turtlebot"  # ← usuario del robot
ROBOT_IP   = "192.168.71.172"
PASSWORD   = "ros"        # ← contraseña del robot

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
        self.publisher = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)  # Nuevo publisher
        self.pub_state = rospy.Publisher('/follower_state', String, queue_size=10)

        self.current_pose = None            # guardará la última PoseWithCovarianceStamped
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
        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped,
                         self._amcl_callback, queue_size=1)
        rospy.Subscriber("/map", OccupancyGrid, self.map_callback)

        rospy.sleep(1)  # dar tiempo a la conexión con los topics

    def _amcl_callback(self, msg: PoseWithCovarianceStamped):
        """Guarda la última estimación de pose de AMCL."""
        self.current_pose = msg.pose.pose   

    def laser_callback(self, msg):
        """Detecta obstáculos en direcciones específicas con frontal estrecho (±15° reales)."""
        num_ranges = len(msg.ranges)
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        center_index = num_ranges // 2  # Índice del ángulo 0 rad (frente)
        angle_range_deg = 20
        offset = int((angle_range_deg * math.pi / 180.0) / angle_increment)  # convertir grados a radianes y luego a pasos

        # Frontal ±15° alrededor del índice central
        front_indices = msg.ranges[center_index - offset : center_index + offset]
        left_indices = msg.ranges[int(3*num_ranges/4):]  # 90° a la izquierda
        right_indices = msg.ranges[:int(num_ranges/4)]  # 90° a la derecha

        # Filtrar distancias válidas
        self.min_distance_front = min([d for d in front_indices if msg.range_min < d < msg.range_max] or [float('inf')])
        self.min_distance_left = min([d for d in left_indices if msg.range_min < d < msg.range_max] or [float('inf')])
        self.min_distance_right = min([d for d in right_indices if msg.range_min < d < msg.range_max] or [float('inf')])

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
        """Devuelve (x, y, yaw) de la última pose AMCL, o None si aún no hay dato."""
        if self.amcl_pose is None:
            return None

        x = self.amcl_pose.position.x
        y = self.amcl_pose.position.y

        # convierte el quaternion a yaw
        import tf.transformations as tft
        q = self.amcl_pose.orientation
        _, _, yaw = tft.euler_from_quaternion([q.x, q.y, q.z, q.w])

        return (x, y, yaw)

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
        self._go_to_target(x, y, yaw=0.0, destination_name=place_name)

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

    # ------------- NECESARIO PARA EL FOLLOWER -----------------
    
    def _ssh_command(self):
        """Comando base ssh con sshpass."""
        return (
            f"sshpass -p {shlex.quote(PASSWORD)} "
            f"ssh -tt -o StrictHostKeyChecking=no "
            f"{ROBOT_USER}@{ROBOT_IP}"
        )

    def _running_remote(self) -> bool:
        """True si follower.launch vive en el robot."""
        chk = subprocess.run(
            f"{self._ssh_command()} pgrep -fa follower.launch",
            shell=True, stdout=subprocess.PIPE, stderr=subprocess.DEVNULL
        )
        return bool(chk.stdout.strip())

    def follow_me(self):
        rospy.loginfo("Activando 'Follow Me' (ventana SSH)...")

        try:
            result = subprocess.run(
                f"{self._ssh_command()} echo ok", shell=True,
                stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=10
            )
            if result.returncode != 0:
                rospy.logerr("SSH inaccesible → abortando.")
                self.speaker.say("No puedo conectar con el robot. Asegúrate de que está encendido y conectado a la red.")
                return
        except subprocess.TimeoutExpired:
            rospy.logerr("SSH timeout → abortando.")
            self.speaker.say("No puedo conectar con el robot. La conexión ha expirado.")
            return
        except Exception as e:
            rospy.logerr(f"Error inesperado al probar SSH: {e}")
            self.speaker.say("Error inesperado al intentar conectar con el robot.")
            return

        # Comando remoto con setup y roslaunch, como en tu script funcional
        remote_cmd = (
            "source /opt/ros/noetic/setup.bash && "
            "source ~/catkin_ws/devel/setup.bash && "
            "roslaunch turtlebot_follower follower.launch"
        )
        remote_escaped = shlex.quote(remote_cmd)

        # Construimos el comando completo para abrir gnome-terminal
        full_cmd = f"{self._ssh_command()} {remote_escaped}"

        term_cmd = (
            f"gnome-terminal -- bash -c "
            f"\"{full_cmd}; echo 'Pulsa ENTER para cerrar esta ventana'; read linea\""
        )

        rospy.loginfo("Lanzando terminal con el nodo follower")
        self.term_proc = subprocess.Popen(term_cmd, shell=True, preexec_fn=os.setsid)

        # Esperamos un poco para que el follower se inicie
        time.sleep(5)

        if self._running_remote():
            self.pub_state.publish("started")
            rospy.loginfo("'Follow Me' activo.")
            self.speaker.say("Modo seguimiento activado. Ya puedes caminar.")
        else:
            rospy.logerr("El follower no arrancó. Cerrando ventana.")
            self.speaker.say("No se pudo activar 'Follow Me'. Asegúrate de que el robot está listo.")
            if self.term_proc.poll() is None:
                self.term_proc.terminate()

    def stop_follow_me(self):
        rospy.loginfo("Deteniendo 'Follow Me'...")

        # Mata el nodo follower en el robot
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

        self.pub_state.publish("stopped")
        rospy.loginfo("'Follow Me' detenido.")

    def approach_nearest_obstacle(self, safe_distance=0.8, speed=0.15):
        """
        Gira hacia el obstáculo más cercano y se aproxima hasta mantener una distancia segura.
        Detiene el avance si pierde la lectura o detecta que está demasiado cerca.
        """
        rospy.loginfo("Buscando el obstáculo más cercano...")

        last_distance = None
        no_change_counter = 0
        max_no_change = 5  # Máximo número de ciclos sin progreso


        try:
            scan_msg = rospy.wait_for_message("/scan", LaserScan, timeout=5.0)
        except rospy.ROSException:
            rospy.logwarn("No se pudieron obtener datos del láser.")
            return

        valid_ranges = [
            (i, dist) for i, dist in enumerate(scan_msg.ranges)
            if scan_msg.range_min < dist < scan_msg.range_max
        ]

        if not valid_ranges:
            rospy.logwarn("No se detectaron obstáculos válidos.")
            return

        min_index, min_dist = min(valid_ranges, key=lambda x: x[1])
        angle_to_turn = scan_msg.angle_min + min_index * scan_msg.angle_increment
        angle_degrees = math.degrees(angle_to_turn)

        rospy.loginfo(f"Obstáculo más cercano a {min_dist:.2f} m, ángulo: {angle_degrees:.1f}°")
        #alinear el robot hacia el obstáculo más cercano
        self.turn(-angle_degrees)

        twist = Twist()
        twist.linear.x = speed
        step_duration = 0.15  # avanzar en pequeños pasos
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            # Protección: si la lectura se vuelve inválida
            if not (0.0 < self.min_distance_front < float('inf')):
                rospy.logwarn("Distancia frontal inválida. Deteniendo por seguridad.")
                break

            if self.min_distance_front <= safe_distance:
                rospy.loginfo("Distancia segura alcanzada. Deteniendo.")
                break

            # Detección de atasco si la distancia no cambia significativamente
            avg_front_distance = self.min_distance_front

            if last_distance and abs(avg_front_distance - last_distance) < 0.01:
                no_change_counter += 1
                if no_change_counter >= max_no_change:
                    rospy.logwarn("El robot parece atascado. Abortando acercamiento.")
                    self.speaker.say("Me he quedado atascado.")
                    break
            else:
                no_change_counter = 0

            last_distance = avg_front_distance
            self.execute_action(twist, step_duration)
            rate.sleep()

        self.stop()
        self.speaker.say("Distancia segura alcanzada. He llegado.")
        
    def execute_action(self, twist, duration):
        rate = rospy.Rate(10)  # 10 Hz
        start_time = rospy.Time.now().to_sec()

        while (rospy.Time.now().to_sec() - start_time) < duration:
            self.publisher.publish(twist)
            rate.sleep()

        self.stop()  # Detener el robot después del tiempo especificado