#!/usr/bin/env python3
"""
Wall Follower TurtleBot2 con LiDAR limitado a ±30°
Sigue automáticamente la pared más cercana.
Evita NaN y movimientos bruscos.Wall
Autor: Jorge (modificado para auto-selección de pared)
"""

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from std_msgs.msg import Bool

class PIDController:
    def __init__(self, kp, ki, kd, output_limit=None):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0.0
        self.integral = 0.0
        self.output_limit = output_limit

    def compute(self, setpoint, measured_value, dt):
        error = setpoint - measured_value
        self.integral += error * dt
        derivative = (error - self.prev_error) / max(dt, 1e-3)
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error
        if self.output_limit is not None:
            output = np.clip(output, -self.output_limit, self.output_limit)
        if not np.isfinite(output):
            output = 0.0
        return output

class WallFollowerNode:
    def __init__(self):
        rospy.init_node('wall_follower_node')

        # Parámetros
        self.obstacle_distance = rospy.get_param('~obstacle_distance', 0.5)
        self.forward_speed = rospy.get_param('~forward_speed', 0.2)
        self.max_angular_speed = rospy.get_param('~max_angular_speed', 0.5)
        self.tolerance = rospy.get_param('~tolerance', 0.04)

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel_mux/input/navi', Twist, queue_size=10)
        self.enabled = False
        rospy.Subscriber("/wall_follower/enable", Bool, self.enable_cb)

        self.pid = PIDController(kp=1.5, ki=0.0, kd=0.1, output_limit=self.max_angular_speed)
        self.prev_time = rospy.Time.now()

        rospy.loginfo("Nodo Wall Follower inicializado con auto-selección de pared")
        rospy.on_shutdown(self.shutdown_hook)

    def enable_cb(self, msg):
        self.enabled = msg.data

        # Reset PID
        self.pid.prev_error = 0.0
        self.pid.integral = 0.0

        if not self.enabled:
            # PUBLICAR STOP VARIAS VECES
            for _ in range(3):
                self.publish_stop()
                rospy.sleep(0.05)

            rospy.loginfo("Wall follower desactivado")
        else:
            self.prev_time = rospy.Time.now()
            rospy.loginfo("Wall follower activado")

    def laser_callback(self, msg):
        if not self.enabled:
            return
        cmd = Twist()
        num_ranges = len(msg.ranges)
        angles = [msg.angle_min + i * msg.angle_increment for i in range(num_ranges)]

        # Filtrar rayos frontales (±30°)
        front_mask = [abs(math.degrees(a)) <= 30 for a in angles]
        front_ranges = [r if np.isfinite(r) else float('inf') for r, m in zip(msg.ranges, front_mask) if m]

        if not front_ranges:
            rospy.logwarn("No hay lecturas frontales válidas.")
            self.publish_stop()
            return

        front_dist = min(front_ranges)

        # Estimar distancias laterales izquierda y derecha
        side_angles_deg = [20, 25, 30]
        left_dists, right_dists = [], []

        for deg in side_angles_deg:
            rad_left = math.radians(deg)
            rad_right = math.radians(-deg)

            index_left = int(round((rad_left - msg.angle_min) / msg.angle_increment))
            index_left = max(0, min(index_left, num_ranges - 1))
            if np.isfinite(msg.ranges[index_left]):
                left_dists.append(msg.ranges[index_left] * abs(math.sin(rad_left)))

            index_right = int(round((rad_right - msg.angle_min) / msg.angle_increment))
            index_right = max(0, min(index_right, num_ranges - 1))
            if np.isfinite(msg.ranges[index_right]):
                right_dists.append(msg.ranges[index_right] * abs(math.sin(rad_right)))

        left_dist = np.mean(left_dists) if left_dists else float('inf')
        right_dist = np.mean(right_dists) if right_dists else float('inf')

        # Selección automática de la pared más cercana
        if left_dist <= right_dist:
            side_dist = left_dist
            side_multiplier = 1.0  # izquierda
        else:
            side_dist = right_dist
            side_multiplier = -1.0  # derecha

        # Tiempo delta para PID
        current_time = rospy.Time.now()
        dt = (current_time - self.prev_time).to_sec()
        self.prev_time = current_time

        follow_distance = self.obstacle_distance + 0.25

        # Lógica de control
        if front_dist < self.obstacle_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = side_multiplier * self.max_angular_speed
        else:
            if not np.isfinite(side_dist):
                side_dist = self.obstacle_distance

            if abs(side_dist - self.obstacle_distance) < self.tolerance:
                error = 0.0
            else:
                error = follow_distance - side_dist
                error = np.clip(error, -follow_distance, follow_distance)

            cmd.linear.x = self.forward_speed
            cmd.angular.z = self.pid.compute(0, error, dt) * side_multiplier

        # Prevenir NaN y limitar velocidades
        cmd.linear.x = np.clip(cmd.linear.x, 0.0, self.forward_speed)
        cmd.angular.z = np.clip(cmd.angular.z, -self.max_angular_speed, self.max_angular_speed)

        self.cmd_vel_pub.publish(cmd)

    def publish_stop(self):
        """Publica parada segura"""
        stop = Twist()
        stop.linear.x = 0.0
        stop.angular.z = 0.0
        self.cmd_vel_pub.publish(stop)
    
    def shutdown_hook(self):
        self.publish_stop()
        rospy.loginfo("Wall follower apagado limpiamente")

if __name__ == '__main__':
    try:
        node = WallFollowerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
