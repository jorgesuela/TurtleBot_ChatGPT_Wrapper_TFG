#!/bin/bash

# Iniciar Gazebo (Simulador)
#gnome-terminal -- bash -c "roslaunch turtlebot_gazebo turtlebot_world.launch; exec bash"

# Esperar un poco para asegurar que roscore arranque
#sleep 8

# tu ip: ifconfig

# hay que comprobar si las ip han cambiado (primero conectar a la misma wifi) para probar : ping 192.168.71.172

# para conectar con el robot desde el portatil:

# ssh turtlebot@192.168.71.172 (la ip puede cambiar)

# para ver la camara : rosrun image_view image_view image:=/camera/rgb/image_raw

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
gnome-terminal -- bash -c "cd ~/catkin_ws/src/turtlebot_chatgpt_wrapper && rosrun turtlebot_chatgpt_wrapper TelegramSpeechToTextNode.py; exec bash"

# FUNCIONA Iniciar nodo 2: ChatGPTProcessorNode
gnome-terminal -- bash -c "rosrun turtlebot_chatgpt_wrapper ChatGPTProcessorNode.py; exec bash"

# FUNCIONA Iniciar nodo 3: TurtleBotControlNode (Tu nodo que contiene el código TurtleBotActions)
gnome-terminal -- bash -c "rosrun turtlebot_chatgpt_wrapper TurtleBotControlNode.py; exec bash"





