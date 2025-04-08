# High-end-autonomous-anomaly-detection-robot
Building a high-end autonomous anomaly detection robot that maps your home, detects anomalies, and sends alerts. With NRF24L01 (for wireless communication) and 4-channel motor controlles.

---

## ⚙️ Why this Project?

To create a system who can collect data behafe of humans. </br> first when start it's move randomly (slowly) for 2D mapping the floor, and if I run joystick this time it can follow my instructions. after mapping done it can store in a SD card, not only store 2d map but also the GPS location with floor angle using gyroscope, then slowly but randomly travel the room for dictation any anomaly using ultrasonic sensor, motion diction and gas sensor and LDR. </br> In our Raspberry Pi Pico W & Esp32 both has their own Wi-Fi & Bluetooth so they can communicate each other using this. And its batter to Raspberry Pi Pico W can control all important sensors because it has more I2C pin. and using servo motor to rotate ultrasonic sensor to create the map (may it's enough with one ultrasonic sensor) and another ultrasonic sensor fix for correction (if needed). also the code for a joystick using two HW-504 & Arduino nano. And send anomaly data using Gmail-smtp.

---

## **📌 Key Features & Workflow**  
1. **2D Mapping**:  
   - ESP32 rotates servo (0°-180°) while logging ultrasonic distances.  
   - Data saved as `map.csv` (GPS, gyro angle, servo angle, distance).  

2. **Anomaly Detection**:  
   - Pico W sends gas/light/motion data to ESP32 every 2s.  
   - ESP32 cross-checks with GPS/gyro data and triggers Gmail alerts.  

3. **Manual Control**:  
   - Joystick sends X/Y values to ESP32 via HTTP.  
   - Add motor control logic in Pico W code to respond to `/control` endpoint.

---

## **📌 Hardware Connections**  
### **1. ESP32 (Master)**  
| **Component**       | **ESP32 Pin** | **Purpose**  
|----------------------|--------------|-------------  
| **Servo Motor**      | GPIO27       | Rotate ultrasonic sensor  
| **Ultrasonic Sensor**| Trig: GPIO13, Echo: GPIO12 | Distance measurement  
| **GPS Module**       | TX: GPIO16, RX: GPIO17 | Location tracking  
| **MPU6050 (Gyro)**   | SDA: GPIO21, SCL: GPIO22 | Floor angle calculation  
| **SD Card (CS)**     | GPIO15       | Map/GPS data storage  

### **2. Raspberry Pi Pico W (Motor/Sensor Hub)**  
| **Component**       | **Pico W Pin** | **Purpose**  
|----------------------|---------------|-------------  
| **Motor A (IN1/IN2)**| GP5/GP6       | Left wheel control  
| **Motor B (IN1/IN2)**| GP7/GP8       | Right wheel control  
| **Gas Sensor (MQ-2)**| GP26 (ADC0)   | Gas leak detection  
| **LDR**              | GP27 (ADC1)   | Light sensing  
| **PIR Motion**       | GP28          | Intruder detection  

### **3. Arduino Nano (Joystick)**  
| **Component**       | **Nano Pin** | **Purpose**  
|----------------------|-------------|-------------  
| **Joystick X/Y**     | A0/A1       | Manual control  
| **WiFi Module (ESP-01)**| TX/RX: D2/D3 | Send commands  

---

## **📌 Required Libraries**  
1. **ESP32** (Arduino IDE):  
   - `WiFi`, `WebServer`, `SD`, `TinyGPS++`, `MPU6050_tockn`, `Servo`, `ESP_Mail_Client`  
2. **Pico W** (Arduino IDE):  
   - `WiFi`, `HTTPClient`  
3. **Arduino Nano**:  
   - `SoftwareSerial`, `WiFiEsp`  

---

### Check the Wireless modules are working properly :: NRF module check --> [code](https://github.com/akashdip2001/Wireless-LED-Control-Code-with-NRF)


<p align="center">
  <img src="img/modules (4).jpg" alt="Image 1" width="46%" style="margin-right: 10px;"/>
  <img src="img/modules (5).jpg" alt="Image 2" width="46%" style="margin-right: 10px;"/>
</p>

---

## **📌 Final Code**  

### **1. ESP32 (Master Controller)**  
```cpp  
#include <WiFi.h>  
#include <WebServer.h>  
#include <SD.h>  
#include <TinyGPS++.h>  
#include <MPU6050_tockn.h>  
#include <Servo.h>  
#include <ESP_Mail_Client.h>  

#define SERVO_PIN 27  
#define TRIG_PIN 13  
#define ECHO_PIN 12  
#define SD_CS 15  

// Wi-Fi AP  
const char* ssid = "RobotAP";  
const char* password = "12345678";  
WebServer server(80);  
String picoData = "";  

// Sensors  
TinyGPSPlus gps;  
HardwareSerial SerialGPS(2);  
MPU6050 mpu(Wire);  
Servo ultrasonicServo;  

// Email  
SMTPSession smtp;  
void sendAlert(String msg);  

void setup() {  
  Serial.begin(115200);  
  WiFi.softAP(ssid, password);  
  Serial.print("AP IP: ");  
  Serial.println(WiFi.softAPIP());  

  // Web Server  
  server.on("/data", HTTP_POST, [](){  
    picoData = server.arg("plain");  
    server.send(200, "text/plain", "OK");  
  });  
  server.begin();  

  // Sensors  
  ultrasonicServo.attach(SERVO_PIN);  
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);  
  SD.begin(SD_CS);  
  Wire.begin();  
  mpu.begin();  
  mpu.calcGyroOffsets();  
}  

void loop() {  
  server.handleClient();  

  // Phase 1: Mapping  
  if (!SD.exists("/map.csv")) {  
    File mapFile = SD.open("/map.csv", FILE_WRITE);  
    for(int angle=0; angle<=180; angle+=10) {  
      ultrasonicServo.write(angle);  
      delay(500);  
      long dist = pulseIn(ECHO_PIN, HIGH) * 0.034 / 2;  
      mpu.update();  
      while(SerialGPS.available() > 0) {  
        if(gps.encode(SerialGPS.read())) {  
          mapFile.print(gps.location.lat(), 6);  
          mapFile.print(",");  
          mapFile.print(gps.location.lng(), 6);  
          mapFile.print(",");  
          mapFile.print(mpu.getAngleZ());  
          mapFile.print(",");  
          mapFile.print(angle);  
          mapFile.print(",");  
          mapFile.println(dist);  
        }  
      }  
    }  
    mapFile.close();  
  }  

  // Phase 2: Anomaly Detection  
  else if(picoData != "") {  
    int gas = picoData.substring(0, picoData.indexOf(',')).toInt();  
    int light = picoData.substring(picoData.indexOf(',')+1, picoData.lastIndexOf(',')).toInt();  
    bool motion = picoData.endsWith("1");  

    if(gas > 1000 || light < 500 || motion) {  
      String alertMsg = "Gas:" + String(gas) + ",Light:" + String(light) + ",Motion:" + motion;  
      sendAlert(alertMsg);  
    }  
    picoData = "";  
  }  
}  

void sendAlert(String msg) {  
  smtp.debug(1);  
  smtp.callback(smtpCallback);  
  ESP_Mail_Session session;  
  session.server.host_name = "smtp.gmail.com";  
  session.server.port = 465;  
  session.login.email = "your_email@gmail.com";  
  session.login.password = "your_app_password";  

  SMTP_Message message;  
  message.sender.name = "Robot";  
  message.sender.email = "robot@example.com";  
  message.subject = "ANOMALY DETECTED";  
  message.addRecipient("You", "you@example.com");  
  message.text.content = msg.c_str();  

  if(!smtp.connect(&session)) return;  
  if(!MailClient.sendMail(&smtp, &message)) Serial.println("Error sending email");  
}  
```  

---

### **2. Raspberry Pi Pico W (Sensor/Motor Hub)**  
```cpp  
#include <WiFi.h>  
#include <HTTPClient.h>  

const char* ssid = "RobotAP";  
const char* password = "12345678";  
const char* serverURL = "http://192.168.4.1/data";  

#define GAS_PIN 26  
#define LDR_PIN 27  
#define PIR_PIN 28  
#define MOTOR_A1 5  
#define MOTOR_A2 6  
#define MOTOR_B1 7  
#define MOTOR_B2 8  

void setup() {  
  Serial.begin(115200);  
  pinMode(MOTOR_A1, OUTPUT);  
  pinMode(MOTOR_A2, OUTPUT);  
  pinMode(MOTOR_B1, OUTPUT);  
  pinMode(MOTOR_B2, OUTPUT);  

  WiFi.begin(ssid, password);  
  while(WiFi.status() != WL_CONNECTED) delay(500);  
}  

void loop() {  
  // Read sensors  
  int gas = analogRead(GAS_PIN);  
  int light = analogRead(LDR_PIN);  
  bool motion = digitalRead(PIR_PIN);  

  // Send to ESP32  
  HTTPClient http;  
  http.begin(serverURL);  
  http.addHeader("Content-Type", "application/json");  
  String json = "{\"gas\":" + String(gas) + ",\"light\":" + String(light) + ",\"motion\":" + motion + "}";  
  http.POST(json);  
  http.end();  

  // Motor control logic (add your commands)  
  delay(2000);  
}  
```  

---

### **3. Arduino Nano (Joystick Controller)**  
```cpp  
#include <SoftwareSerial.h>  
#include <WiFiEsp.h>  

SoftwareSerial espSerial(2, 3); // RX, TX  
const char* ssid = "RobotAP";  
const char* password = "12345678";  
const char* serverURL = "http://192.168.4.1/control";  

void setup() {  
  Serial.begin(115200);  
  espSerial.begin(9600);  
  WiFi.init(&espSerial);  

  if(WiFi.status() == WL_NO_SHIELD) {  
    Serial.println("WiFi shield not present");  
    while(true);  
  }  

  while(WiFi.begin(ssid, password) != WL_CONNECTED) delay(500);  
}  

void loop() {  
  int x = analogRead(A0);  
  int y = analogRead(A1);  

  WiFiEspClient client;  
  if(client.connect("192.168.4.1", 80)) {  
    String postData = "x=" + String(x) + "&y=" + String(y);  
    client.print("POST /control HTTP/1.1\r\nHost: 192.168.4.1\r\nContent-Length: " + String(postData.length()) + "\r\n\r\n" + postData);  
    client.stop();  
  }  
  delay(100);  
}  
```  

---

## **📌 Setup Instructions**  
1. **ESP32**:  
   - Format SD card as FAT32.  
   - Create **Gmail App Password** (Google Account → Security → App Passwords).  

2. **Pico W**:  
   - Use Arduino IDE with **Raspberry Pi Pico W** board support.  

3. **Testing**:  
   - Power ESP32 first → Connect Pico W/Joystick to `RobotAP` network.  
   - Visit `http://192.168.4.1` to monitor status.

---

Here’s the **simplified, clear hardware connections** for **ESP32**, **Raspberry Pi Pico W**, and **Arduino Nano Joystick** setup. I’ll break it down

---

### **1. ESP32 (Master Controller)**  
| **Component**        | **ESP32 Pin** | **Purpose**  
|-----------------------|---------------|-------------  
| **NRF24L01 (CE)**     | GPIO4         | Radio enable  
| **NRF24L01 (CSN)**    | GPIO5         | Chip select  
| **NRF24L01 (SCK)**    | GPIO18        | SPI clock  
| **NRF24L01 (MOSI)**   | GPIO23        | SPI data out  
| **NRF24L01 (MISO)**   | GPIO19        | SPI data in  
| **Ultrasonic (Trig)** | GPIO13        | Trigger pin  
| **Ultrasonic (Echo)** | GPIO12        | Echo pin  
| **Servo Motor**       | GPIO27        | Rotate sensor for mapping  
| **GPS (TX)**          | GPIO16        | Receive GPS data  
| **GPS (RX)**          | GPIO17        | Unused (just connect TX)  
| **MPU6050 (SDA)**     | GPIO21        | I2C data  
| **MPU6050 (SCL)**     | GPIO22        | I2C clock  
| **SD Card (CS)**      | GPIO15        | SD card select  

---

### **2. Raspberry Pi Pico W (Motor/Sensor Hub)**  
| **Component**         | **Pico W Pin** | **Purpose**  
|-----------------------|----------------|-------------  
| **NRF24L01 (CE)**     | GP1            | Radio enable  
| **NRF24L01 (CSN)**    | GP0            | Chip select  
| **NRF24L01 (SCK)**    | GP2            | SPI clock  
| **NRF24L01 (MOSI)**   | GP3            | SPI data out  
| **NRF24L01 (MISO)**   | GP4            | SPI data in  
| **Motor A (IN1/IN2)** | GP5/GP6        | Left motor control  
| **Motor B (IN1/IN2)** | GP7/GP8        | Right motor control  
| **Gas Sensor (MQ-2)** | GP26 (ADC0)    | Gas detection (analog)  
| **LDR**               | GP27 (ADC1)    | Light sensor (analog)  
| **PIR Motion**        | GP28           | Motion detection (digital)  

---

### **3. Arduino Nano (Joystick Controller)**  
| **Component**         | **Nano Pin** | **Purpose**  
|-----------------------|--------------|-------------  
| **Joystick 1 (X/Y)**  | A0/A1        | X/Y axis analog input  
| **Joystick 2 (X/Y)**  | A2/A3        | X/Y axis analog input  
| **NRF24L01 (CE)**     | D7           | Radio enable  
| **NRF24L01 (CSN)**    | D8           | Chip select  
| **NRF24L01 (SCK)**    | D13          | SPI clock  
| **NRF24L01 (MOSI)**   | D11          | SPI data out  
| **NRF24L01 (MISO)**   | D12          | SPI data in  

---

### **4. Power Connections**  
- **ESP32**: Power via USB or 5V pin.  
- **Pico W**: Power via USB or VSYS (5V).  
- **Motors**: Use a **separate 5V battery** connected to the L298N motor driver.  
- **Sensors**: Use the **3.3V pin** for NRF24L01, MPU6050, and PIR.  

---

### **5. Key Notes**  
1. **NRF24L01 Wiring**:  
   - **VCC** → 3.3V (NOT 5V!)  
   - **GND** → Common ground.  
   - **Capacitor**: Add a **10µF capacitor** between VCC and GND for stability.  

2. **Motor Driver (L298N)**:  
   - **ENA/ENB** → Connect to PWM pins (not needed for basic control).  
   - **IN1/IN2/IN3/IN4** → Connected to Pico W’s GP5-GP8.  

3. **Ultrasonic Sensor**:  
   - Use **HC-SR04** (5V tolerant). Connect Echo to ESP32’s GPIO12.  

4. **GPS Module**:  
   - Only **TX** pin is needed (connects to ESP32’s GPIO16).  

---

### **6. Sensor Flow**  
- **ESP32**: Handles mapping, GPS, gyroscope, and email alerts.  
- **Pico W**: Controls motors, gas/LDR/PIR sensors.  
- **Arduino Nano**: Sends joystick commands via NRF24L01.  

---
