# Realizado por:
# Sofia Estrada Hernandez - A01666608
# Emmanuel Lopez Paredes - A01666331
# Jose Miguel Rodriguez Coronel - A01666969

import time
import board
import busio
import smbus
import adafruit_bmp280
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt
import json

# Configuración del cliente MQTT
mqttc = mqtt.Client()
mqttc.username_pw_set("Sofia7123", "71231106")
mqttc.connect("broker.hivemq.com", 1883)
mqttc.loop_start()  # Inicia el bucle MQTT

# Configuración de sensores I2C
i2c = busio.I2C(board.SCL, board.SDA)

# Sensor BMP280
bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c, 0x76)

# Sensor ADXL345 (acelerómetro)
ADXL345_ADDR = 0x53
bus = smbus.SMBus(1)
bus.write_byte_data(ADXL345_ADDR, 0x2D, 0x08)

# Sensor ADS1115 (fotoresistencia)
ads = ADS.ADS1115(i2c)
channel_ads1115 = AnalogIn(ads, ADS.P0)

# Sensor de ultrasonido
GPIO.setmode(GPIO.BCM)
TRIG = 23
ECHO = 24
GPIO.setup(TRIG, GPIO.OUT)
GPIO.setup(ECHO, GPIO.IN)

# Guarda los valores de temperatura, presion y altitud y los devuelve
def leer_bmp280():
    temperatura = bmp280.temperature
    presion = bmp280.pressure
    altitud = bmp280.altitude
    return temperatura, presion, altitud

# Guarda la aceleracion en x,y,z y los devuelve
def leer_adxl345():
    data = bus.read_i2c_block_data(ADXL345_ADDR, 0x32, 6)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]

    # Ajuste de valores negativos
    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536

    # Ajuste de escala para aceleración (en g)
    return {
            "x": x * 0.03924,
            "y": y * 0.03924,
            "z": z * 0.03924
        }
# Registra distancia del ultrasonico y la devuelve
def leer_ultrasonido():
    GPIO.output(TRIG, False)
    time.sleep(2)

    GPIO.output(TRIG, True)
    time.sleep(0.00001)
    GPIO.output(TRIG, False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()

    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()

    duracion_pulso = pulse_end - pulse_start
    distancia = duracion_pulso * 17150
    return round(distancia, 2)

# Lee el valor de la fotoresistencia y lo devuelve
def leer_ads1115():
    return channel_ads1115.voltage, channel_ads1115.value

try:
    while True:
        # Leer datos de BMP280 y publicar en MQTT
        temperatura, presion, altitud = leer_bmp280()
        mensaje_bmp = f"{temperatura}_{presion}_{altitud}"
        mqttc.publish("bts/sensor1", mensaje_bmp, qos=2)
        print(f"BMP280 - Datos enviados: {mensaje_bmp}")

        # Leer datos de ADXL345 y publicar en MQTT
        datos_adxl = leer_adxl345()
        mqttc.publish("bts/acelerometro", json.dumps(datos_adxl), qos =2)
        print(f"ADXL345 - Datos enviados: {datos_adxl}")
        print(f"Datos serializados acelerometro: {json.dumps(datos_adxl)}")

        # Leer datos de ultrasonido y publicar en MQTT
        distancia = leer_ultrasonido()
        mensaje_ultrasonido = f"{distancia}"
        mqttc.publish("bts/ultrasonico", mensaje_ultrasonido, qos=2)
        print(f"Ultrasonido - Datos enviados: {mensaje_ultrasonido}")

        # Leer datos de ADS1115 y publicar en MQTT
        voltage, analog_value = leer_ads1115()
        mensaje_ads = f"{voltage}"
        mqttc.publish("bts/fotoresistencia", mensaje_ads, qos=2)
        print(f"ADS1115 - Datos enviados: {mensaje_ads}")

        # Intervalo de publicación
        time.sleep(10)

except KeyboardInterrupt:
    print("Deteniendo la medición de datos...")

finally:
    # Limpieza de GPIO y desconexión MQTT
    GPIO.cleanup()
    mqttc.loop_stop()
    mqttc.disconnect()
  