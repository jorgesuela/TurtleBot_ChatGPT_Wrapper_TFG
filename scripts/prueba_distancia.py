#!/usr/bin/env python3

import rospy
from TurtleBotActions import TurtleBotActions
import math

def main():
    rospy.init_node('distance_printer_node', anonymous=True)
    tb = TurtleBotActions()
    rate = rospy.Rate(2)  # 2 Hz

    rospy.loginfo("Nodo de prueba iniciado. Mostrando distancias a obstáculos...")

    while not rospy.is_shutdown():
        if tb.scan_data:
            angle_min_deg = math.degrees(tb.scan_data.angle_min)
            angle_max_deg = math.degrees(tb.scan_data.angle_max)
            rospy.loginfo(f"Ángulo LIDAR: min={angle_min_deg:.1f}°, max={angle_max_deg:.1f}°")
        else:
            rospy.loginfo("Esperando datos del escáner...")

        front = tb.get_front_distance()
        left = tb.get_left_distance()
        right = tb.get_right_distance()

        # Evitar imprimir inf o nan
        def safe_dist(d):
            if math.isinf(d) or math.isnan(d):
                return float('nan')
            return d

        rospy.loginfo(f"Distancias - Frente: {safe_dist(front):.2f} m, Izquierda: {safe_dist(left):.2f} m, Derecha: {safe_dist(right):.2f} m")
        rate.sleep()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
