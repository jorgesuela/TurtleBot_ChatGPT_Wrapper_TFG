# TurtleBot_ChatGPT_Wrapper_TFG

# 🚀 Módulo de Control del Robot con ChatGPT y Telegram

Este módulo permite controlar un robot mediante comandos de voz enviados por Telegram, procesados por ChatGPT y ejecutados en un TurtleBot en **TurtleSim** o **Gazebo**.

## 📌 Configuración Inicial
Se debe descargar el modelo de reconomiento de voz deseado. Para las simulaciones, se ha usado el modelo de precision maxima de vosk, el cual se puede descargar del siguiente enlace:

[🔗 Vosk models (todos los idiomas)](https://alphacephei.com/vosk/models)

Para que funcione, simplemente crea una carpeta en el paquete que se llame Model y mete el reconocedor que hayas descargado dentro.

Antes de usar el módulo, debes definir dos variables de entorno:

### 🔑 1. Configurar la API Key de Telegram

Esta clave permite que el bot reciba mensajes:

```bash
export TELEGRAM_BOT_TOKEN="AQUI_LA_CLAVE"
```

### 🧠 2. Configurar la API Key de ChatGPT

Se requiere una cuenta de OpenAI con saldo disponible.\
💰 **Costo estimado**: \~0.002 USD por 300 tokens de prompt.

```bash
export OPENAI_API_KEY="AQUI_LA_CLAVE"
```

---

## ▶️ Ejecución del Módulo

Para ejecutar el paquete, simplemente ejecuta el script `launcher.sh` dentro del paquete:

```bash
./launcher.sh
```

---

## 🎮 Simulación en **Gazebo** en vez de **TurtleSim**

Si prefieres usar **Gazebo**, edita la línea correspondiente en el script `launcher.sh` y cambia el **topic** en la clase `turtlebotActions` de `/turtle1/cmd_vel` a `/cmd_vel`.

### 🔧 Pasos para cambiar a Gazebo:

1️⃣ Añade estas líneas en el script:

```bash
export TURTLEBOT3_MODEL=burger  # Define el modelo del robot
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch  # Inicia el mundo en Gazebo
```

2️⃣ Modifica la publicación de velocidades en `turtlebotActions`:

📌 **Antes:**

```python
self.publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
```

📌 **Después:**

```python
self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
```

---

## 📚 Más Información

🔗 Consulta la documentación oficial de TurtleBot3 para más detalles sobre la simulación en Gazebo:\
[🔗 TurtleBot3 Simulation - Robotis eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

