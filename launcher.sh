#!/bin/bash

# Iniciar ROS master (si no está corriendo)
gnome-terminal -- bash -c "roscore; exec bash"

# Esperar un poco para asegurar que roscore arranque
sleep 3

# Iniciar nodo 1
gnome-terminal -- bash -c "cd ~/catkin_ws/src/turtlebot_chatgpt_wrapper && rosrun turtlebot_chatgpt_wrapper TelegramSpeechToTextNode.py; exec bash"

# Iniciar nodo 2
gnome-terminal -- bash -c "rosrun turtlebot_chatgpt_wrapper ChatGPTProcessorNode.py; exec bash"

# Iniciar nodo 3
gnome-terminal -- bash -c "rosrun turtlebot_chatgpt_wrapper TurtleBotControlNode.py; exec bash"

# Iniciar nodo 4
gnome-terminal -- bash -c "rosrun turtlesim turtlesim_node; exec bash"
