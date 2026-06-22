# TurtleBot_ChatGPT_Wrapper_TFG

# 🚀 Módulo de Control del Robot con ChatGPT y SpeakLink

Este módulo permite controlar un robot mediante comandos de voz/texto enviados a traves de la app SpeakLink (en desarrollo), procesados por ChatGPT y ejecutados en un TurtleBot en **Gazebo**.

<img width="1086" height="1448" alt="aaaa" src="https://github.com/user-attachments/assets/1d77d166-cb7c-4cb4-9dcf-6b55efacfc2a" />
ets/42fba6f4-04b7-45d9-8cf7-fc8f2264e872)






## 📌 1. Configuración Inicial
Se debe descargar/añadir el modelo de reconomiento de voz deseado. Para las simulaciones, se ha usado el modelo de google, aunque también se utilizo el modelo de precision maxima de vosk con muy buenos resultados y completamente offline, el cual se puede descargar del siguiente enlace:

[🔗 Vosk models (todos los idiomas)](https://alphacephei.com/vosk/models)

Para que funcione, simplemente crea una carpeta en el paquete que se llame Model y mete el reconocedor que hayas descargado dentro.

Antes de usar el módulo, debes definir una variables de entorno:

### 🧠 2. Configurar la API Key de ChatGPT

Se requiere una cuenta de OpenAI con saldo disponible.\
💰 **Costo estimado de gpt-3.5**: \~0.002 USD por 1000 tokens de prompt.

```bash
export OPENAI_API_KEY="AQUI_LA_CLAVE"
```

---

### :calling: SpeakLink: app para controlar el robot
Para poder controlar el robot, ambos dispositivos deberan conectarse a la misma red y se deberá especificar tanto la ip como el puerto por el que se enviarán los mensajes al nodo de ros. Aunque no es necesario y se puede hacer de forma manual, se ha desarrollado una app que facilita este proceso.

Enlace de la app: [(en desarrollo🔧)](https://github.com/jorgesuela/robo-whisper-android)

## 🎮 3. Simulación en **Gazebo**

Para simular en **Gazebo**:
1️⃣ Añade estas líneas en el launcher.sh, para modificar el tipo de robot que quieres simular, y el mundo que quieres cargar en las simulacion:

```bash
export TURTLEBOT3_MODEL=burger  # Define el modelo del robot
roslaunch turtlebot3_gazebo turtlebot3_empty_world.launch  # Inicia el mundo en Gazebo
```

## ▶️ 4. Ejecución del Módulo

Para ejecutar el paquete, simplemente ejecuta el script `launcher.sh` dentro del paquete:

```bash
./launcher.sh
```

---

## 📚 Más Información

🔗 Consulta la documentación oficial de TurtleBot3 para más detalles sobre la simulación en Gazebo:\
[🔗 TurtleBot3 Simulation - Robotis eManual](https://emanual.robotis.com/docs/en/platform/turtlebot3/simulation/#gazebo-simulation)

