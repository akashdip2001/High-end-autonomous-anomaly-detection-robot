# using Py
# High-end-autonomous-anomaly-detection-robot
Building a high-end autonomous anomaly detection robot that maps your home, detects anomalies, and sends alerts. With NRF24L01 (for wireless communication) and 4-channel motor controlles.

---

## ⚙️ Why this Project?

To create a system who can collect data behafe of humans.

---

✅ **Wireless Remote Control** (ESP32 ↔ Raspberry Pi Pico W using NRF24L01)  
✅ **Autonomous Navigation + Anomaly Detection**  
✅ **Real-Time Alerts to Phone**  
✅ **2D Room Mapping**  
✅ **Manual Control Mode** (via joystick)  

---

## **🚀 Step 1: Components & Roles**  

| Component | Role |
|-----------|------|
| **ESP32** | Main controller (WiFi alerts, navigation, anomaly detection) |
| **Raspberry Pi Pico W** | Handles motor control & sensors |
| **NRF24L01 PA/LNA** | Wireless communication between ESP32 and Pico W |
| **4-Channel Motor Driver** | Controls four motors for movement |
| **Ultrasonic Sensors (x2)** | Obstacle avoidance & mapping |
| **MPU6050 Gyroscope** | Tilt & movement tracking |
| **Gas Sensor (MQ-2/MQ-135)** | Detects gas leaks |
| **Motion Detector (PIR)** | Detects unauthorized movement |
| **Joystick Controller (HW-504)** | Manual control of the robot |
| **GPS Module (GY-GPS6MV2)** | Pinpoints location of detected anomalies |
| **LCD Display (JHD162A)** | Displays system status |
| **Micro SD Card Module** | Stores mapping data |

---

## **🔌 Step 2: Wiring and Connections**  
We'll set up **ESP32 as the master** and **Pico W as the motor controller** via NRF24L01.

### **ESP32 Wiring** (Master)  
| Component | ESP32 Pins |
|-----------|------------|
| **NRF24L01** | MISO → GPIO 19, MOSI → GPIO 23, SCK → GPIO 18, CSN → GPIO 5, CE → GPIO 4 |
| **Ultrasonic Sensor (Front)** | TRIG → GPIO 13, ECHO → GPIO 12 |
| **Ultrasonic Sensor (Side)** | TRIG → GPIO 14, ECHO → GPIO 27 |
| **Gas Sensor (MQ-2)** | A0 → GPIO 34 |
| **Motion Detector (PIR)** | OUT → GPIO 36 |
| **GPS Module** | TX → GPIO 17, RX → GPIO 16 |
| **LCD Display (I2C)** | SDA → GPIO 21, SCL → GPIO 22 |

### **Raspberry Pi Pico W Wiring** (Motor Controller)  
| Component | Raspberry Pi Pico W Pins |
|-----------|------------------|
| **NRF24L01** | MISO → GP4, MOSI → GP3, SCK → GP2, CSN → GP0, CE → GP1 |
| **4-Channel Motor Driver** | IN1 → GP5, IN2 → GP6, IN3 → GP7, IN4 → GP8 |

---

## **🔥 Additional Features**
✅ **Wireless Communication (NRF24L01)**  
✅ **Autonomous Navigation with Mapping**  
✅ **Manual Control (Joystick + NRF24L01)**  
✅ **Real-Time Alerts to Phone**  
✅ **GPS Tracking for Anomalies** 

---

## **📜 Step 3: Code**
### **ESP32 Code (Master)**
```python
from machine import Pin, I2C
import network
import time
import urequests
import nrf24l01
from hcsr04 import HCSR04
from mpu6050 import MPU6050
from gps import GPS

# WiFi & Alert Setup
WIFI_SSID = "YourWiFiSSID"
WIFI_PASS = "YourWiFiPassword"
WEBHOOK_URL = "https://maker.ifttt.com/trigger/anomaly_detected/with/key/YOUR_IFTTT_KEY"

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(WIFI_SSID, WIFI_PASS)
    while not wlan.isconnected():
        pass
    print("Connected to WiFi:", wlan.ifconfig())

connect_wifi()

# Setup NRF24L01 (SPI)
spi = machine.SPI(1, baudrate=10000000, polarity=0, phase=0, sck=Pin(18), mosi=Pin(23), miso=Pin(19))
csn = Pin(5, machine.Pin.OUT)
ce = Pin(4, machine.Pin.OUT)
radio = nrf24l01.NRF24L01(spi, csn, ce, payload_size=32)
radio.open_tx_pipe(b"00001")  
radio.open_rx_pipe(1, b"00002")  
radio.start_listening()

# Sensors
front_sensor = HCSR04(trigger_pin=13, echo_pin=12)
side_sensor = HCSR04(trigger_pin=14, echo_pin=27)
mpu = MPU6050(21, 22)
gps = GPS(tx=17, rx=16)

# Mapping & Movement
map_data = {}
for i in range(0, 360, 10):  
    radio.stop_listening()
    radio.send(b"TURN_RIGHT")  
    time.sleep(0.2)
    distance = front_sensor.distance_cm()
    map_data[i] = distance

print("2D Map Generated:", map_data)

# Anomaly Detection
def detect_anomalies():
    gas_level = Pin(34, Pin.IN).value()
    motion = Pin(36, Pin.IN).value()
    tilt_x, tilt_y, tilt_z = mpu.get_values()

    if gas_level > 300:
        send_alert("Gas Leak Detected!", gps.get_location())

    if motion:
        send_alert("Unauthorized Movement Detected!", gps.get_location())

    if abs(tilt_x) > 10 or abs(tilt_y) > 10:
        send_alert("Structural Shift Detected!", gps.get_location())

def send_alert(message, location):
    data = {"value1": message, "value2": location}
    headers = {"Content-Type": "application/json"}
    urequests.post(WEBHOOK_URL, data=json.dumps(data), headers=headers)
    print("Alert Sent:", message, location)

# Main Loop
while True:
    radio.stop_listening()
    radio.send(b"MOVE_FORWARD")
    time.sleep(1)
    detect_anomalies()
    radio.send(b"STOP")
    time.sleep(2)
```

---

### **Raspberry Pi Pico W Code (Motor Controller)**
```python
import machine
import time
import nrf24l01

# NRF24L01 Setup
spi = machine.SPI(1, baudrate=10000000, polarity=0, phase=0, sck=machine.Pin(2), mosi=machine.Pin(3), miso=machine.Pin(4))
csn = machine.Pin(0, machine.Pin.OUT)
ce = machine.Pin(1, machine.Pin.OUT)
radio = nrf24l01.NRF24L01(spi, csn, ce, payload_size=32)
radio.open_tx_pipe(b"00002")  
radio.open_rx_pipe(1, b"00001")  
radio.start_listening()

# Motor Setup
motorA1 = machine.Pin(5, machine.Pin.OUT)
motorA2 = machine.Pin(6, machine.Pin.OUT)
motorB1 = machine.Pin(7, machine.Pin.OUT)
motorB2 = machine.Pin(8, machine.Pin.OUT)

def move_forward():
    motorA1.on()
    motorA2.off()
    motorB1.on()
    motorB2.off()

def move_backward():
    motorA1.off()
    motorA2.on()
    motorB1.off()
    motorB2.on()

def turn_left():
    motorA1.off()
    motorA2.on()
    motorB1.on()
    motorB2.off()

def turn_right():
    motorA1.on()
    motorA2.off()
    motorB1.off()
    motorB2.on()

def stop():
    motorA1.off()
    motorA2.off()
    motorB1.off()
    motorB2.off()

# Main Loop
while True:
    if radio.any():
        message = radio.recv()
        if message == b"MOVE_FORWARD":
            move_forward()
        elif message == b"TURN_RIGHT":
            turn_right()
        elif message == b"STOP":
            stop()
``` 

