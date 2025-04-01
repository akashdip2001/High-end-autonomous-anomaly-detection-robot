# High-end-autonomous-anomaly-detection-robot
Building a high-end autonomous anomaly detection robot that maps your home, detects anomalies, and sends alerts. With NRF24L01 (for wireless communication) and 4-channel motor controlles.

---

## ⚙️ Why this Project?

To create a system who can collect data behafe of humans. </br> first when start it's move randomly (slowly) for 2D mapping the floor, and if I run joystick this time it can follow my instructions. after mapping done it can store in a SD card, not only store 2d map but also the GPS location with floor angle using gyroscope, then slowly but randomly travel the room for dictation any anomaly using ultrasonic sensor, motion diction and gas sensor and LDR. </br> In our Raspberry Pi Pico W & Esp32 both has their own Wi-Fi & Bluetooth so they can communicate each other using this. And its batter to Raspberry Pi Pico W can control all important sensors because it has more I2C pin. and using servo motor to rotate ultrasonic sensor to create the map (may it's enough with one ultrasonic sensor) and another ultrasonic sensor fix for correction (if needed). also the code for a joystick using two HW-504 & Arduino nano. And send anomaly data using Gmail-smtp.

---

## **📌 Key Features**  
1. **2D Mapping**: Servo rotates ultrasonic sensor, stores data + GPS + gyro angles to SD card.  
2. **Joystick Control**: Override autonomous mode via Arduino Nano.  
3. **Anomaly Detection**: Gas, light (LDR), motion.  
4. **Gmail Alerts**: Uses ESP32 Mail Client.  
5. **Sensor Fusion**: MPU6050 for floor angle correction.
   
---

### **📌 Simplified Hardware Connections**
#### **ESP32 (Master)**
| **Component**       | **ESP32 Pin** |
|----------------------|---------------|
| **Servo Motor**      | GPIO27        |
| **Ultrasonic Sensor**| Trig: GPIO13, Echo: GPIO12 |
| **GPS Module**       | TX: GPIO16, RX: GPIO17 |
| **MPU6050 (Gyro)**   | SDA: GPIO21, SCL: GPIO22 |
| **SD Card (CS)**     | GPIO15        |

#### **Raspberry Pi Pico W (Sensor Hub)**
| **Component**       | **Pico W Pin** |
|----------------------|----------------|
| **Ultrasonic Sensor**| Trig: GP9, Echo: GP10 |
| **Gas Sensor (MQ-2)**| GP26 (ADC)     |
| **LDR**              | GP27 (ADC)     |
| **PIR Motion**       | GP28           |

---

### **📌 Communication Architecture**
1. **ESP32**:
   - Hosts a **Wi-Fi Access Point** (AP) for direct local network control.
   - Runs a **web server** to receive sensor data from the Pico W.
   - Sends **Gmail alerts** with GPS/map data.

2. **Pico W**:
   - Connects to the ESP32's Wi-Fi AP.
   - Sends sensor data (**gas**, **LDR**, **PIR**) via HTTP POST requests.

---

### **📌 Required Libraries**
1. **ESP32**:
   - `WiFi` (built-in)
   - `WebServer` (built-in)
   - `ESP32_Mail_Client` (Gmail SMTP)
   - `TinyGPS++`
   - `SD`
   - `MPU6050_tockn`
   - `Servo`

2. **Pico W**:
   - `WiFi` (MicroPython)
   - `urequests` (HTTP)

---

### **1. ESP32 Code (Wi-Fi Server + Mapping)**
```cpp
#include <WiFi.h>
#include <WebServer.h>
#include <ESP32_Mail_Client.h>
#include <TinyGPS++.h>
#include <SD.h>
#include <MPU6050_tockn.h>
#include <Servo.h>

// Wi-Fi AP Settings
const char* ssid = "RobotAP";
const char* password = "12345678";

// Web Server
WebServer server(80);
String picoData = ""; // Stores sensor data from Pico W

// Gmail SMTP
#define SMTP_HOST "smtp.gmail.com"
#define SMTP_PORT 465
SMTP_Data smtp;

// Sensors
TinyGPSPlus gps;
HardwareSerial SerialGPS(2); // GPS on UART2
MPU6050 mpu6050(Wire);
Servo ultrasonicServo;

// SD Card
File mapFile;

void setup() {
  Serial.begin(115200);
  
  // Wi-Fi AP
  WiFi.softAP(ssid, password);
  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());

  // Web Server Routes
  server.on("/data", HTTP_POST, handleData); // Pico W sends data here
  server.begin();

  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  // MPU6050
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);

  // Servo & SD Card
  ultrasonicServo.attach(27);
  SD.begin(15);
}

void loop() {
  server.handleClient();

  // Phase 1: 2D Mapping
  if (!SD.exists("/map.csv")) {
    mapFile = SD.open("/map.csv", FILE_WRITE);
    for (int angle = 0; angle <= 180; angle += 10) {
      ultrasonicServo.write(angle);
      delay(500);
      long distance = readUltrasonic();
      logMapData(distance, angle);
    }
    mapFile.close();
  }

  // Phase 2: Anomaly Detection
  else {
    if (picoData != "") {
      checkAnomalies();
      picoData = "";
    }
  }
}

void handleData() {
  picoData = server.arg("plain");
  server.send(200, "text/plain", "OK");
}

void logMapData(long distance, int angle) {
  mpu6050.update();
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        mapFile.print(gps.location.lat(), 6);
        mapFile.print(",");
        mapFile.print(gps.location.lng(), 6);
        mapFile.print(",");
        mapFile.print(mpu6050.getAngleZ());
        mapFile.print(",");
        mapFile.print(angle);
        mapFile.print(",");
        mapFile.println(distance);
      }
    }
  }
}

void checkAnomalies() {
  int gas = picoData.substring(0, picoData.indexOf(',')).toInt();
  int ldr = picoData.substring(picoData.indexOf(',')+1).toInt();
  bool motion = picoData.endsWith("1");

  if (gas > 1000 || ldr < 500 || motion) {
    String alertMsg = String(gps.location.lat(), 6) + "," + 
                      String(gps.location.lng(), 6) + "," +
                      String(mpu6050.getAngleZ()) + "," +
                      "Gas:" + gas + ",Light:" + ldr + ",Motion:" + motion;
    sendEmailAlert(alertMsg);
  }
}

void sendEmailAlert(String message) {
  smtp.setLogin(SMTP_HOST, SMTP_PORT, "your_email@gmail.com", "your_app_password");
  smtp.setSender("Robot", "robot@example.com");
  smtp.setSubject("ANOMALY ALERT");
  smtp.setMessage(message, false);
  smtp.addRecipient("you@example.com");
  MailClient.sendMail(smtp);
}

long readUltrasonic() {
  digitalWrite(13, HIGH);
  delayMicroseconds(10);
  digitalWrite(13, LOW);
  return pulseIn(12, HIGH) * 0.034 / 2;
}
```

---

### **2. Pico W Code (Sensor Hub)**
```python
import network
import urequests
import ujson
from machine import ADC, Pin
import time

# Wi-Fi Settings
SSID = "RobotAP"
PASSWORD = "12345678"
ESP32_IP = "192.168.4.1"  # ESP32's AP IP

# Sensors
gas_sensor = ADC(26)
ldr = ADC(27)
pir = Pin(28, Pin.IN)

def connect_wifi():
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(SSID, PASSWORD)
    while not wlan.isconnected():
        pass

connect_wifi()

while True:
    data = {
        "gas": gas_sensor.read_u16(),
        "ldr": ldr.read_u16(),
        "motion": pir.value()
    }
    response = urequests.post(
        f"http://{ESP32_IP}/data",
        headers={"Content-Type": "application/json"},
        data=ujson.dumps(data)
    )
    time.sleep(2)
```

---

### **3. Joystick Code (Arduino Nano)**
```cpp
#include <WiFi.h>
#include <HTTPClient.h>

const char* ssid = "RobotAP";
const char* password = "12345678";
const char* serverURL = "http://192.168.4.1/control";

void setup() {
  Serial.begin(115200);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) delay(500);
}

void loop() {
  int joyX = analogRead(A0);
  int joyY = analogRead(A1);
  
  String payload = "x=" + String(joyX) + "&y=" + String(joyY);
  
  HTTPClient http;
  http.begin(serverURL);
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");
  http.POST(payload);
  http.end();
  
  delay(100);
}
```

---

### **Key Features & Workflow**
1. **2D Mapping**:
   - Servo rotates ultrasonic sensor (0°-180°).
   - Data stored as `map.csv` (GPS, gyro angle, distance).
   
2. **Anomaly Detection**:
   - Pico W sends gas/LDR/motion data to ESP32 every 2s.
   - ESP32 cross-references with GPS/gyro data and triggers email alerts.

3. **Manual Control**:
   - Joystick sends `x/y` values via Wi-Fi to override autonomous mode.

---

### **Setup Instructions**
1. **ESP32**:
   - Create a **Gmail App Password** (Google Account → Security).
   - Insert SD card formatted as FAT32.

2. **Pico W**:
   - Upload MicroPython code (use Thonny IDE).

3. **Testing**:
   - Power up ESP32 first → Connect Pico W/Joystick to `RobotAP`.
   - Visit `http://192.168.4.1` to debug.
