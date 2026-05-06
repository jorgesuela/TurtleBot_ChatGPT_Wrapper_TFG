#!/bin/bash

# Iniciar Gazebo (Simulador)
gnome-terminal -- bash -c "roslaunch turtlebot_gazebo turtlebot_world.launch; exec bash"

# Esperar un poco para asegurar que roscore arranque
sleep 5

# tu ip: ifconfig

# hay que comprobar si las ip han cambiado (primero conectar a la misma wifi) para probar : ping 192.168.71.172

# para conectar con el robot desde el portatil:

# ssh turtlebot@192.168.71.172 (la ip puede cambiar)

# para ver la camara : rosrun image_view image_view image:=/camera/rgb/image_raw

# para lanzar la camara: roslaunch astra_launch astra.launch

# para controlar el robot con el teclado: roslaunch turtlebot_teleop keyboard_teleop.launch 

# PARA HACER MAPAS

# roslaunch turtlebot_navigation gmapping_demo.launch

# PARA GUARDAR MAPAS

# rosrun map_server map_saver -f /tmp/my_map

# PARA USAR MAPAS

# primero cargar mapa

# export TURTLEBOT_MAP_FILE=/tmp/my_map.yaml 

# lanzar amcl

# roslaunch turtlebot_navigation amcl_demo.launch

# roslaunch turtlebot_rviz_launchers view_navigation.launch --screen
# FUNCIONA Iniciar nodo 1: TelegramSpeechToTextNode
gnome-terminal -- bash -c "cd ~/catkin_ws/src/cisc_turtlebot_chatgpt_wrapper && rosrun cisc_turtlebot_chatgpt_wrapper TelegramSpeechToTextNode.py; exec bash"

sleep 1

# FUNCIONA Iniciar nodo 2: ChatGPTProcessorNode
gnome-terminal -- bash -c "rosrun cisc_turtlebot_chatgpt_wrapper ChatGPTProcessorNode.py; exec bash"

sleep 1

# Nodo Wall Follower (activo pero sin hacer nada)
gnome-terminal -- bash -c "rosrun cisc_turtlebot_chatgpt_wrapper wall_follower_node.py; exec bash"
