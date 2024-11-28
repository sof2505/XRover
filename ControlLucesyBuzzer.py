# Realizado por:
# Sofia Estrada Hernandez - A01666608
# Emmanuel Lopez Paredes - A01666331
# Jose Miguel Rodriguez Coronel - A01666969

import smbus
import time
import board
import adafruit_bmp280
import busio
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
import RPi.GPIO as GPIO
import paho.mqtt.client as mqtt

# Dirección I2C del ADXL345
ADXL345_ADDR = 0x53

# Pines GPIO
BUZZER_PIN = 17
LED_ALERT_PIN = 27  # LED rojo para alertas
LED_STABLE_PIN = 22  # LED verde siempre encendido
TRIG = 23
ECHO = 24

# Configuración del broker MQTT
broker = "broker.hivemq.com"
port = 1883
topic_movimiento = "bts/rover/movimiento"

# Se establece la comunicación con el bus I2C
bus = smbus.SMBus(1) 

# Inicialización del ADXL345
def init_adxl345():
    # Poner el sensor en modo de medida
    bus.write_byte_data(ADXL345_ADDR, 0x2D, 0x08)  
# Lectura del ADXL345
def read_adxl345():
    # Leer los datos de aceleracion en el eje X, Y, Z
    data = bus.read_i2c_block_data(ADXL345_ADDR, 0x32, 6)
    x = (data[1] << 8) | data[0]
    y = (data[3] << 8) | data[2]
    z = (data[5] << 8) | data[4]

    # Ajuste de valores negativos
    if x > 32767: x -= 65536
    if y > 32767: y -= 65536
    if z > 32767: z -= 65536

    # Ajuste de valores de las aceleraciones
    x = x * 0.03924
    y = y * 0.03924
    z = z * 0.03924

    return x, y, z

# Obtención de valores del GY-BMP280
def get_bmp280_values():
    GPIO.cleanup()
    i2c = board.I2C()
    # Se establece conexión con el bus I2C en la dirección 0x76
    bmp280 = adafruit_bmp280.Adafruit_BMP280_I2C(i2c,address=0x76)
    bmp280.sea_level_pressure = 1013.25
    # Devuelve temperatura, presión y altitud
    return bmp280.temperature, bmp280.pressure, bmp280.altitude

# Obtención de valores del convertidor
def get_ADS1115_values():
    #Se genera la conexión con el bus I2C
    i2c = busio.I2C(board.SCL, board.SDA)
    ads = ADS.ADS1115(i2c)
    #Se guarda el valor analógico de la fotoresistencia
    channel = AnalogIn(ads, ADS.P0)
    #Devuelve valor analógico y voltaje
    return channel.value, channel.voltage

# Obtiene una distancia
def medicion_ultrasonico():
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    TRIG = 23
    ECHO = 24
    # Se declara TRIG como salida y ECHO como entrada
    GPIO.setup(TRIG,GPIO.OUT)
    GPIO.setup(ECHO,GPIO.IN)
    GPIO.output(TRIG,False)
    time.sleep(2)
    GPIO.output(TRIG,True)
    time.sleep(0.00001)
    GPIO.output(TRIG,False)

    while GPIO.input(ECHO) == 0:
        pulse_start = time.time()
        
    while GPIO.input(ECHO) == 1:
        pulse_end = time.time()
        
    pulso_dura = pulse_end - pulse_start
    # Calcula la distancia en base a la velocidad de las ondas
    dist = pulso_dura * 17150
    dist = round (dist,2)
    return dist

def configurar_motores():
    GPIO.setmode(GPIO.BCM)
    EnableM1, IN1_M1, IN2_M2 = 17, 27, 22
    EnableM2, IN4_M2, IN3_M2 = 23, 24, 25
    GPIO.setup([EnableM1, IN1_M1, IN2_M2, EnableM2, IN4_M2, IN3_M2], GPIO.OUT)
    GPIO.output([EnableM1, EnableM2], GPIO.HIGH)
    return EnableM1, IN1_M1, IN2_M2, EnableM2, IN4_M2, IN3_M2

# Cambia el estado del coche para que avance adelante
def avanza_adelante(IN1_M1, IN2_M2, IN4_M2, IN3_M2):
    print("Adelante")
    GPIO.output(IN1_M1, GPIO.HIGH)
    GPIO.output(IN2_M2, GPIO.LOW)
    GPIO.output(IN4_M2, GPIO.HIGH)
    GPIO.output(IN3_M2, GPIO.LOW)

# Cambia el estado del coche para que avance atrás
def avanza_atras(IN1_M1, IN2_M2, IN4_M2, IN3_M2):
    print("Atras")
    GPIO.output(IN1_M1, GPIO.LOW)
    GPIO.output(IN2_M2, GPIO.HIGH)
    GPIO.output(IN4_M2, GPIO.LOW)
    GPIO.output(IN3_M2, GPIO.HIGH)

# Cambia el estado del coche para que avance a la izquierda
def avanza_izq(IN1_M1, IN2_M2, IN4_M2, IN3_M2):
    print("izquierda")
    GPIO.output(IN1_M1, GPIO.LOW)
    GPIO.output(IN2_M2, GPIO.LOW)
    GPIO.output(IN4_M2, GPIO.HIGH)
    GPIO.output(IN3_M2, GPIO.LOW)

# Cambia el estado del coche para que avance a la derecha
def avanza_der(IN1_M1, IN2_M2, IN4_M2, IN3_M2):
    print("derecha")
    GPIO.output(IN1_M1, GPIO.HIGH)
    GPIO.output(IN2_M2, GPIO.LOW)
    GPIO.output(IN4_M2, GPIO.LOW)
    GPIO.output(IN3_M2, GPIO.LOW)

# Detiene todos los motores
def detener_motores(EnableM1, IN1_M1, IN2_M2, EnableM2, IN4_M2, IN3_M2):
    print("Detener")
    GPIO.output([EnableM1, IN1_M1, IN2_M2, EnableM2, IN4_M2, IN3_M2], GPIO.LOW)
    
def alerta_sos():
    BUZZER_PIN = 17
    LED_ALERT_PIN = 27  # LED rojo para alertas
    LED_STABLE_PIN = 22  # LED verde siempre encendido
    TRIG = 23
    ECHO = 24
    # Configuración GPIO
    GPIO.cleanup()
    GPIO.setmode(GPIO.BCM)
    GPIO.setup(BUZZER_PIN, GPIO.OUT)
    GPIO.setup(LED_ALERT_PIN, GPIO.OUT)
    GPIO.setup(LED_STABLE_PIN, GPIO.OUT)
    GPIO.setup(TRIG, GPIO.OUT)
    GPIO.setup(ECHO, GPIO.IN)

    # Inicializa LEDs y buzzer
    GPIO.output(LED_STABLE_PIN, GPIO.HIGH)  # LED verde encendido al inicio
    GPIO.output(LED_ALERT_PIN, GPIO.LOW)
    GPIO.output(BUZZER_PIN, GPIO.LOW)
    for _ in range(3):  # Tres ciclos cortos, largos, cortos
        for _ in range(3):  # Cortos
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            GPIO.output(LED_ALERT_PIN, GPIO.HIGH)
            GPIO.output(LED_STABLE_PIN, GPIO.LOW)
            time.sleep(0.2)
            GPIO.output(BUZZER_PIN, GPIO.LOW)
            GPIO.output(LED_ALERT_PIN, GPIO.LOW)
            time.sleep(0.2)
        for _ in range(3):  # Largos
            GPIO.output(BUZZER_PIN, GPIO.HIGH)
            GPIO.output(LED_ALERT_PIN, GPIO.HIGH)
            GPIO.output(LED_STABLE_PIN, GPIO.LOW)
            time.sleep(0.6)
            GPIO.output(BUZZER_PIN, GPIO.LOW)
            GPIO.output(LED_ALERT_PIN, GPIO.LOW)
            time.sleep(0.2)
    GPIO.output(LED_STABLE_PIN, GPIO.HIGH)  # Reactivar LED estable
    
def on_connect(client, userdata, flags, rc):
    print(f"Conectado al broker MQTT con código: {rc}")
    if rc == 0:
        client.subscribe(topic_movimiento)
        print(f"Subscrito al tema: {topic_movimiento}")
    else:
        print(f"Error al conectar con el broker. Código: {rc}")

# Callback de recepción de mensajes
def on_message(client, userdata, msg):
    comando = msg.payload.decode("utf-8")
    print(f"Comando recibido: {comando}")
    if comando == "adelante":
        avanza_adelante(M1,M2,M3,M4)
    elif comando == "atras":
        avanza_atras(M1,M2,M3,M4)
    elif comando == "izquierda":
        avanza_izq(M1,M2,M3,M4)
    elif comando == "derecha":
        avanza_der(M1,M2,M3,M4)
    elif comando == "parar":
        detener_motores(EN1, M1, M2, EN2, M3, M4)
    else:
        print("Comando desconocido")

    
# Inicializar sensor
init_adxl345()
mqtt_client = mqtt.Client()
mqtt_client.on_connect = on_connect  # Asignar callback de conexión
mqtt_client.on_message = on_message  # Asignar callback de mensajes
# Entra en un ciclo, hasta que el usuario quiera salir
while True:
    # Leer y mostrar valores del ADXL345
    print("Valores del acelerómetro ADXL345")
    print("")
    x, y, z = read_adxl345()
    print("Aceleración en X: {:.2f}, Y: {:.2f}, Z: {:.2f}".format(x, y, z))
    print("")
    time.sleep(0.1)

    # Leer y mostrar valores del GY-BMP280
    temperature, pressure, altitude = get_bmp280_values()
    print("Valores del sensor GY-BMP280")
    print("")
    print("Temperatura: {:.2f} C".format(temperature))
    print("Presión: {:.2f} hPa".format(pressure))
    print("Altitud: {:.2f} metros".format(altitude))
    print("")
    time.sleep(0.1)
    
    # Leer y mostrar valores del convertidor ADS1115
    value, voltage = get_ADS1115_values()
    print("Valores del convertidor ADS1115")
    print("")
    print("Valor analógico: {} ".format(value))
    print("Voltaje: {:.2f} ".format(voltage))
    print("")
    time.sleep(0.1)
    
    # Leer y mostrar valores del sensor ultrasónico
    print("Valor del sensor ultrasónico")
    print("")
    distancia = medicion_ultrasonico()
    print("Distancia: {} cm".format(distancia))
    print("")
    time.sleep(0.1)
    # Se guardan los 6 valores devueltos por configurar_motores en las variables
    EN1, M1, M2, EN2, M3, M4 = configurar_motores()
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
        
    
    
