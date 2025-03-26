#!/bin/bash

# Iniciar ROS master (si no está corriendo)
gnome-terminal -- bash -c "roscore; exec bash"

# Esperar un poco para asegurar que roscore arranque
sleep 3

# Iniciar nodo 1: TelegramSpeechToTextNode
gnome-terminal -- bash -c "cd ~/catkin_ws/src/turtlebot_chatgpt_wrapper && rosrun turtlebot_chatgpt_wrapper TelegramSpeechToTextNode.py; exec bash"

# Iniciar nodo 2: ChatGPTProcessorNode
gnome-terminal -- bash -c "rosrun turtlebot_chatgpt_wrapper ChatGPTProcessorNode.py; exec bash"

# Iniciar nodo 3: TurtleBotControlNode
gnome-terminal -- bash -c "rosrun turtlebot_chatgpt_wrapper TurtleBotControlNode.py; exec bash"

# Iniciar Gazebo (Simulador)
gnome-terminal -- bash -c "roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch; exec bash"

# Iniciar nodo 5: GMapping SLAM (Creación del mapa)
#gnome-terminal -- bash -c "roslaunch turtlebot3_slam turtlebot3_slam.launch; exec bash"

# Esperar a que SLAM se inicie y cree el mapa
#sleep 10

# Iniciar nodo 6: Navegación (move_base, amcl, etc.)
#gnome-terminal -- bash -c "roslaunch turtlebot3_navigation turtlebot3_navigation.launch; exec #bash"

