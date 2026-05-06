#!/usr/bin/env python3
from TurtleBotActions import TurtleBotActions
import rospy
import math

rospy.init_node("generated_node", anonymous=True)
tba = TurtleBotActions()

def safe_get_distance(getter):
    try:
        d = getter()
        if d is None:
            return None
        d = float(d)
        if math.isinf(d) or math.isnan(d) or d <= 0.0:
            return None
        return d
    except Exception:
        return None

def main():
    tba.say("Voy a mirar alrededor para encontrar la dirección más despejada y avanzar hacia lo más lejano que pueda detectar con seguridad.")

    # Escaneo 360º por sectores usando el lidar frontal.
    # Guardamos la mejor dirección según la mayor distancia frontal detectada.
    scan_angles = [0, 45, 90, 135, 180, 225, 270, 315]
    current_heading = 0
    best_heading = 0
    best_distance = -1.0

    # Medición inicial
    d0 = safe_get_distance(tba.get_front_distance)
    if d0 is not None:
        best_distance = d0
        best_heading = 0

    for target in scan_angles[1:]:
        delta = target - current_heading
        if delta > 180:
            delta -= 360
        elif delta < -180:
            delta += 360

        try:
            tba.rotate(delta, speed=0.4)
        except Exception:
            tba.say("He tenido un problema al girar. Me detengo por seguridad.")
            tba.stop()
            return

        current_heading = target
        rospy.sleep(0.3)

        d = safe_get_distance(tba.get_front_distance)
        if d is not None and d > best_distance:
            best_distance = d
            best_heading = current_heading

    # Volver a la mejor orientación encontrada
    delta_to_best = best_heading - current_heading
    if delta_to_best > 180:
        delta_to_best -= 360
    elif delta_to_best < -180:
        delta_to_best += 360

    try:
        tba.rotate(delta_to_best, speed=0.4)
    except Exception:
        tba.say("No he podido orientarme hacia la dirección más despejada. Me detengo por seguridad.")
        tba.stop()
        return

    rospy.sleep(0.3)
    final_front = safe_get_distance(tba.get_front_distance)

    if final_front is None:
        tba.say("No puedo estimar una distancia fiable ahora mismo. Puedo acercarme al obstáculo más cercano si quieres.")
        tba.stop()
        return

    # Avanzar hacia lo más lejano detectable, pero sin comprometer seguridad.
    # Dejamos margen de seguridad de 0.8 m y limitamos el avance máximo.
    travel_distance = max(0.0, min(final_front - 0.8, 3.0))

    if travel_distance < 0.15:
        tba.say("No detecto espacio suficiente para avanzar con seguridad en esa dirección. Puedo buscar otra maniobra si quieres.")
        tba.stop()
        return

    try:
        tba.move_forward(travel_distance, speed=0.25, obstacle_threshold=0.6)
        tba.say("He mirado alrededor y he avanzado hacia la zona más despejada que he podido detectar.")
    except Exception:
        tba.say("He encontrado un problema al avanzar y me he detenido por seguridad.")
        tba.stop()

if __name__ == "__main__":
    try:
        main()
    except Exception:
        try:
            tba.stop()
            tba.say("Ha ocurrido un error y me detengo por seguridad.")
        except Exception:
            pass

# SUMMARY: Voy a mirar alrededor y avanzar hacia la dirección más despejada que pueda detectar con seguridad.