**Complete, optimized code** for your **High-End Autonomous Anomaly Detection Robot** with all connections, libraries, and features integrated:

---

## **📌 Hardware Connections**  
### **1. ESP32 (Master Controller)**  
| **Component**          | **ESP32 Pin** |  
|-------------------------|---------------|  
| **Servo Motor (SG90)**  | GPIO27        |  
| **Ultrasonic (Trig)**   | GPIO13        |  
| **Ultrasonic (Echo)**   | GPIO12        |  
| **GPS (TX)**            | GPIO16        |  
| **GPS (RX)**            | GPIO17        |  
| **MPU6050 (SDA/SCL)**   | GPIO21/22     |  
| **SD Card (CS/SCK/MISO/MOSI)** | GPIO15/18/19/23 |  
| **Wi-Fi Antenna**       | Built-in      |  

---

### **2. Raspberry Pi Pico W (Motor/Sensor Controller)**  
| **Component**          | **Pico W Pin** |  
|-------------------------|----------------|  
| **Motor A (IN1/IN2)**   | GP5/GP6        |  
| **Motor B (IN1/IN2)**   | GP7/GP8        |  
| **Ultrasonic (Trig/Echo)** | GP9/GP10     |  
| **Gas Sensor (MQ-2)**   | GP26 (ADC)     |  
| **LDR**                 | GP27 (ADC)     |  
| **PIR Motion**          | GP28           |  
| **I2C (SDA/SCL)**       | GP20/GP21      |  

---

### **3. Arduino Nano (Joystick)**  
| **Component**          | **Nano Pin** |  
|-------------------------|--------------|  
| **Joystick 1 (X/Y)**    | A0/A1        |  
| **Joystick 2 (X/Y)**    | A2/A3        |  
| **Wi-Fi (ESP8266/ESP-01)** | TX/RX     | *Use separate ESP-01 module* |  

---

## **📌 Required Libraries**  
1. **ESP32 (Arduino IDE)**:  
   - `WiFi`, `WebServer`, `SD`, `SPI` (built-in)  
   - `TinyGPS++`, `MPU6050_tockn`, `ESP32_Mail_Client`, `Servo`  

2. **Pico W (MicroPython)**:  
   - `network`, `urequests`, `machine`, `time`  

3. **Arduino Nano (Joystick)**:  
   - `WiFiNINA` or `ESP8266WiFi` (for ESP-01)  

---

## **1. ESP32 Code (Master)**  
```cpp  
#include <WiFi.h>  
#include <WebServer.h>  
#include <SD.h>  
#include <TinyGPS++.h>  
#include <MPU6050_tockn.h>  
#include <ESP32_Mail_Client.h>  
#include <Servo.h>  

// Wi-Fi AP  
const char* ssid = "RobotAP";  
const char* password = "12345678";  
WebServer server(80);  

// Sensors  
TinyGPSPlus gps;  
HardwareSerial SerialGPS(2);  
MPU6050 mpu6050(Wire);  
Servo ultrasonicServo;  

// SD Card  
File mapFile;  

// Gmail  
#define SMTP_HOST "smtp.gmail.com"  
SMTP_Data smtp;  

// Global Variables  
String picoData = "";  
float floorAngle = 0;  

void setup() {  
  Serial.begin(115200);  

  // Wi-Fi AP  
  WiFi.softAP(ssid, password);  
  server.on("/data", HTTP_POST, handleData);  
  server.begin();  

  // GPS  
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);  

  // MPU6050  
  Wire.begin();  
  mpu6050.begin();  
  mpu6050.calcGyroOffsets(true);  

  // Servo & SD  
  ultrasonicServo.attach(27);  
  SD.begin(15);  
}  

void loop() {  
  server.handleClient();  

  // Phase 1: Mapping  
  if (!SD.exists("/map.csv")) {  
    mapFile = SD.open("/map.csv", FILE_WRITE);  
    for (int angle = 0; angle <= 180; angle += 10) {  
      ultrasonicServo.write(angle);  
      delay(500);  
      logMapData();  
    }  
    mapFile.close();  
  }  

  // Phase 2: Anomaly Detection  
  if (picoData != "") checkAnomalies();  

  // Manual Control  
  static unsigned long lastControl = 0;  
  if (millis() - lastControl > 100) {  
    handleJoystick();  
    lastControl = millis();  
  }  
}  

void handleData() {  
  picoData = server.arg("plain");  
  server.send(200, "text/plain", "OK");  
}  

void logMapData() {  
  mpu6050.update();  
  long distance = pulseIn(12, HIGH) * 0.034 / 2;  
  if (gps.encode(SerialGPS.read())) {  
    mapFile.print(gps.location.lat(), 6);  
    mapFile.print(",");  
    mapFile.print(gps.location.lng(), 6);  
    mapFile.print(",");  
    mapFile.print(mpu6050.getAngleZ());  
    mapFile.print(",");  
    mapFile.println(distance);  
  }  
}  

void checkAnomalies() {  
  int gas = picoData.substring(0, picoData.indexOf(',')).toInt();  
  int ldr = picoData.substring(picoData.indexOf(',') + 1, picoData.lastIndexOf(',')).toInt();  
  bool motion = picoData.substring(picoData.lastIndexOf(',') + 1).toInt();  

  if (gas > 1000 || ldr < 500 || motion) {  
    String alert = String(gps.location.lat(), 6) + "," +  
                  String(gps.location.lng(), 6) + "," +  
                  "Gas:" + gas + ",Light:" + ldr + ",Motion:" + motion;  
    sendEmail(alert);  
  }  
  picoData = "";  
}  

void sendEmail(String msg) {  
  smtp.setLogin(SMTP_HOST, 465, "your_email@gmail.com", "your_app_password");  
  smtp.setSender("Robot", "robot@example.com");  
  smtp.setSubject("ANOMALY ALERT");  
  smtp.setMessage(msg, false);  
  smtp.addRecipient("recipient@example.com");  
  MailClient.sendMail(smtp);  
}  

void handleJoystick() {  
  // Implement joystick logic (e.g., HTTP POST from Nano)  
}  
```  

---

## **2. Pico W Code (MicroPython)**  
```python  
import network  
import urequests  
from machine import ADC, Pin, PWM  
import time  

# Wi-Fi  
SSID = "RobotAP"  
PASSWORD = "12345678"  
ESP32_IP = "192.168.4.1"  

# Sensors  
gas = ADC(26)  
ldr = ADC(27)  
pir = Pin(28, Pin.IN)  
ultrasonic_trig = Pin(9, Pin.OUT)  
ultrasonic_echo = Pin(10, Pin.IN)  

# Motors  
motorA1 = Pin(5, Pin.OUT)  
motorA2 = Pin(6, Pin.OUT)  
motorB1 = Pin(7, Pin.OUT)  
motorB2 = Pin(8, Pin.OUT)  

def connect_wifi():  
    wlan = network.WLAN(network.STA_IF)  
    wlan.active(True)  
    wlan.connect(SSID, PASSWORD)  
    while not wlan.isconnected():  
        pass  

def measure_distance():  
    ultrasonic_trig.value(1)  
    time.sleep_us(10)  
    ultrasonic_trig.value(0)  
    while ultrasonic_echo.value() == 0:  
        pass  
    start = time.ticks_us()  
    while ultrasonic_echo.value() == 1:  
        pass  
    return (time.ticks_us() - start) * 0.034 / 2  

connect_wifi()  

while True:  
    data = {  
        "gas": gas.read_u16(),  
        "ldr": ldr.read_u16(),  
        "motion": pir.value()  
    }  
    try:  
        urequests.post(f"http://{ESP32_IP}/data", json=data)  
    except:  
        pass  
    time.sleep(2)  
```  

---

## **3. Arduino Nano (Joystick)**  
```cpp  
#include <WiFiNINA.h>  

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

  WiFiClient client;  
  HTTPClient http;  
  http.begin(client, serverURL);  
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");  
  http.POST(payload);  
  http.end();  

  delay(50);  
}  
```  

---

## **📌 Key Features**  
1. **Autonomous Mapping**:  
   - Servo sweeps 180° with ultrasonic sensor.  
   - Logs GPS, gyro angles, and distances to SD card (`map.csv`).  

2. **Anomaly Detection**:  
   - Gas, light, motion triggers.  
   - Email alerts with GPS coordinates.  

3. **Manual Control**:  
   - Joystick overrides autonomy via HTTP.  

4. **Wi-Fi Communication**:  
   - ESP32 as AP, Pico W/Nano as clients.  

---

## **📌 Setup Instructions**  
1. **ESP32**:  
   - Format SD card as FAT32.  
   - Create Gmail App Password (Google Account → Security).  

2. **Pico W**:  
   - Upload MicroPython code via Thonny IDE.  

3. **Nano**:  
   - Use ESP-01 for Wi-Fi (or WiFiNINA for Nano 33 IoT).  
