# Realizado por:
# Sofia Estrada Hernandez - A01666608
# Emmanuel Lopez Paredes - A01666331
# Jose Miguel Rodriguez Coronel - A01666969

import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import time
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                       
# Configuración de pines GPIO para los motores
EnableM1, IN1_M1, IN2_M1 = 40, 38, 36
EnableM2, IN4_M2, IN3_M2 = 33, 35, 37

# Configuración del broker MQTT
broker = "mqtt-dashboard.com"
port = 1883
# Comunicacion con el topico movimiento
topic_movimiento = "bts/rover/movimiento"

# Configuración de GPIO
GPIO.setmode(GPIO.BOARD)
GPIO.setup([EnableM1, IN1_M1, IN2_M1, EnableM2, IN4_M2, IN3_M2], GPIO.OUT)
GPIO.output([EnableM1, EnableM2], GPIO.HIGH)  # Activar motores

# Funciones de control de motores
def detener_motores():
    GPIO.output([IN1_M1, IN2_M1, IN4_M2, IN3_M2], GPIO.LOW)
    print("Motores detenidos")

def avanza_adelante():
    GPIO.output(IN1_M1, GPIO.HIGH)
    GPIO.output(IN2_M1, GPIO.LOW)
    GPIO.output(IN4_M2, GPIO.HIGH)
    GPIO.output(IN3_M2, GPIO.LOW)
    print("Moviendo adelante")

def avanza_atras():
    GPIO.output(IN1_M1, GPIO.LOW)
    GPIO.output(IN2_M1, GPIO.HIGH)
    GPIO.output(IN4_M2, GPIO.LOW)
    GPIO.output(IN3_M2, GPIO.HIGH)
    print("Moviendo atrás")

def avanza_izquierda():
    GPIO.output(IN1_M1, GPIO.LOW)
    GPIO.output(IN2_M1, GPIO.LOW)
    GPIO.output(IN4_M2, GPIO.HIGH)
    GPIO.output(IN3_M2, GPIO.LOW)
    print("Girando izquierda")

def avanza_derecha():
    GPIO.output(IN1_M1, GPIO.HIGH)
    GPIO.output(IN2_M1, GPIO.LOW)
    GPIO.output(IN4_M2, GPIO.LOW)
    GPIO.output(IN3_M2, GPIO.LOW)
    print("Girando derecha")

# Callback de conexión
def on_connect(client, userdata, flags, rc):
    print(f"Conectado al broker MQTT con código: {rc}")
    if rc == 0:
        client.subscribe(topic_movimiento)
        print(f"Subscrito al tema: {topic_movimiento}")
    else:
        print(f"Error al conectar con el broker. Código: {rc}")

# Metodo que recibe los mensajes del MQTT para que el motor avance
def on_message(client, userdata, msg):
    comando = msg.payload.decode("utf-8")
    print(f"Comando recibido: {comando}")
    if comando == "adelante":
        avanza_adelante()
    elif comando == "atras":
        avanza_atras()
    elif comando == "izquierda":
        avanza_izquierda()
    elif comando == "derecha":
        avanza_derecha()
    elif comando == "parar":
        detener_motores()
    else:
        print("Comando desconocido")

# Configuración del cliente MQTT
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect  # Asignar callback de conexión
mqtt_client.on_message = on_message  # Asignar callback de mensajes

try:
    print("Conectando al broker MQTT...")
    mqtt_client.connect(broker, port, keepalive=60)
    print("Esperando comandos MQTT...")
    mqtt_client.loop_forever()
except KeyboardInterrupt:
    print("Interrumpido manualmente")
finally:
    detener_motores()
    GPIO.cleanup()
    mqtt_client.disconnect()


