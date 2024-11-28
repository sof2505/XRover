# Creando un archivo README.md con el contenido solicitado
readme_content = """
# XRover - Mini Mars Rover 🚀

**XRover** (también conocido como *Mart Robert*) es un proyecto innovador que integra IoT, bases de datos relacionales, sistemas digitales y diseño de interfaces. Su propósito es simular un rover de exploración marciana capaz de recopilar datos de sensores, controlarse a través de una interfaz web y visualizarlos en tiempo real.

---

## **Características principales**
- **Control remoto:** El XRover se opera mediante una interfaz web utilizando el protocolo MQTT.
- **Sensores integrados:**
  - BMP280 (presión, temperatura y altitud)
  - ADXL345 (acelerómetro)
  - ADS1115 (conversor analógico-digital)
  - Sensor ultrasónico
- **Interfaz de visualización:** Dashboard interactivo para monitorear los datos en tiempo real.
- **Integración con bases de datos:** Almacena lecturas de los sensores en una base de datos relacional para análisis posterior.
- **Diseño modular:** Permite la expansión y modificación sencilla.

---

## **Requisitos del sistema**
- **Hardware:**
  - Raspberry Pi
  - Sensores mencionados
  - Componentes adicionales para integración (cables, protoboard, etc.)
- **Software:**
  - Python 3.10+
  - HiveMQ MQTT Broker
  - Base de datos MySQL
  - Librerías requeridas (ver sección de instalación)
- **Otros:**
  - Conexión a Internet para el intercambio de datos vía MQTT.

---

## **Instalación**
### **Clonación del repositorio**
```bash
git clone https://github.com/sof2505/XRover.git
cd XRover
