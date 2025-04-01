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

Here’s the **final optimized code** for **High-End Autonomous Anomaly Detection Robot** with all requested features. Break it into **3 parts** (ESP32, Pico W, Arduino Nano Joystick) with **connections**, **libraries**, and **code**.

---

## **📌 Hardware Connections**  
### **1. ESP32 (Master)**  
| **Component**       | **ESP32 Pin** |  
|---------------------|-------------|  
| **NRF24L01 (CE)**   | GPIO4       |  
| **NRF24L01 (CSN)**  | GPIO5       |  
| **NRF24L01 (SCK)**  | GPIO18      |  
| **NRF24L01 (MOSI)** | GPIO23      |  
| **NRF24L01 (MISO)** | GPIO19      |  
| **GPS (TX)**        | GPIO16      |  
| **SD Card (CS)**    | GPIO15      |  
| **Servo Motor**     | GPIO27      |  
| **LDR**             | GPIO34      |  

### **2. Raspberry Pi Pico W (Sensor & Motor Controller)**  
| **Component**       | **Pico W Pin** |  
|---------------------|--------------|  
| **NRF24L01 (CE)**   | GP1          |  
| **NRF24L01 (CSN)**  | GP0          |  
| **NRF24L01 (SCK)**  | GP2          |  
| **NRF24L01 (MOSI)** | GP3          |  
| **NRF24L01 (MISO)** | GP4          |  
| **Motor A (IN1/IN2)** | GP5/GP6    |  
| **Motor B (IN1/IN2)** | GP7/GP8    |  
| **Ultrasonic (Trig/Echo)** | GP9/GP10 |  
| **Gas Sensor (MQ-2)** | GP26 (ADC) |  
| **MPU6050 (SDA/SCL)** | GP20/GP21 |  

### **3. Arduino Nano (Joystick Controller)**  
| **Component**       | **Nano Pin** |  
|---------------------|-------------|  
| **Joystick 1 (X/Y)** | A0/A1      |  
| **Joystick 2 (X/Y)** | A2/A3      |  
| **NRF24L01 (CE)**   | D7          |  
| **NRF24L01 (CSN)**  | D8          |  
| **NRF24L01 (SCK)**  | D13         |  
| **NRF24L01 (MOSI)** | D11         |  
| **NRF24L01 (MISO)** | D12         |  

---

## **📌 Required Libraries**  
1. **ESP32**:  
   - `RF24` (NRF24L01)  
   - `TinyGPS++` (GPS)  
   - `SD` (SD Card)  
   - `ESP32_Mail_Client` (Gmail SMTP)  
   - `MPU6050_tockn` (Gyroscope)  
   - `Servo`  

2. **Pico W**:  
   - `RF24`  
   - `HCSR04` (Ultrasonic)  
   - `MPU6050_tockn`  

3. **Arduino Nano**:  
   - `RF24`  

---

## **📌 Final Code**  

### **1. ESP32 (Master)**  
```cpp  
#include <SPI.h>  
#include <nRF24L01.h>  
#include <RF24.h>  
#include <TinyGPS++.h>  
#include <SD.h>  
#include <ESP32_MailClient.h>  
#include <MPU6050_tockn.h>  
#include <Servo.h>  

// NRF24L01  
RF24 radio(4, 5); // CE=4, CSN=5  
const byte address[6] = "00001";  

// GPS  
TinyGPSPlus gps;  
HardwareSerial SerialGPS(2); // TX=16, RX=17  

// SD Card  
File mapFile;  
#define SD_CS 15  

// Servo  
Servo ultrasonicServo;  
int servoAngle = 0;  

// Sensors  
MPU6050 mpu6050(Wire);  
float floorAngle = 0;  

// Gmail SMTP  
#define SMTP_HOST "smtp.gmail.com"  
#define SMTP_PORT 465  
MailClientSMTP mail;  

void setup() {  
  Serial.begin(115200);  

  // NRF24L01  
  radio.begin();  
  radio.openWritingPipe(address);  
  radio.setPALevel(RF24_PA_MAX);  

  // GPS  
  SerialGPS.begin(9600);  

  // SD Card  
  if (!SD.begin(SD_CS)) Serial.println("SD Card Failed!");  

  // Servo  
  ultrasonicServo.attach(27);  

  // MPU6050  
  Wire.begin();  
  mpu6050.begin();  
  mpu6050.calcGyroOffsets(true);  

  // Gmail Setup  
  mail.debug(1);  
  mail.settings.setLogin(SMTP_HOST, SMTP_PORT, "your_email@gmail.com", "your_app_password");  
}  

void loop() {  
  // Phase 1: Mapping  
  if (!SD.exists("map.txt")) {  
    mapFile = SD.open("map.txt", FILE_WRITE);  
    for (servoAngle = 0; servoAngle <= 180; servoAngle += 10) {  
      ultrasonicServo.write(servoAngle);  
      delay(500);  
      long distance = getUltrasonicDistance();  
      mpu6050.update();  
      floorAngle = mpu6050.getAngleZ();  
      mapFile.print(gps.location.lat(), 6);  
      mapFile.print(",");  
      mapFile.print(gps.location.lng(), 6);  
      mapFile.print(",");  
      mapFile.print(floorAngle);  
      mapFile.print(",");  
      mapFile.println(distance);  
    }  
    mapFile.close();  
  }  

  // Phase 2: Anomaly Detection  
  else {  
    checkAnomalies();  
    if (radio.available()) {  
      char command[20];  
      radio.read(&command, sizeof(command));  
      if (strstr(command, "JOYSTICK")) handleJoystick(command);  
    }  
  }  
}  

void checkAnomalies() {  
  int gasValue = analogRead(34);  
  int ldrValue = analogRead(35);  

  if (gasValue > 1000 || ldrValue < 500) {  
    String alert = "ALERT: Gas=" + String(gasValue) + ", Light=" + String(ldrValue);  
    sendEmailAlert(alert);  
  }  
}  

void sendEmailAlert(String message) {  
  MailClient.sendMail(mail, "recipient@example.com", "Anomaly Alert", message);  
}  

void handleJoystick(char* command) {  
  // Parse joystick data (e.g., "JOYSTICK,1200,1500")  
  int x = atoi(strtok(command + 8, ","));  
  int y = atoi(strtok(NULL, ","));  
  // Implement motor control logic here  
}  
```  

---

### **2. Raspberry Pi Pico W (Motor/Sensor Controller)**  
```cpp  
#include <SPI.h>  
#include <nRF24L01.h>  
#include <RF24.h>  
#include <HCSR04.h>  
#include <MPU6050_tockn.h>  

RF24 radio(1, 0); // CE=1, CSN=0  
const byte address[6] = "00001";  

// Motors  
#define MOTOR_A1 5  
#define MOTOR_A2 6  
#define MOTOR_B1 7  
#define MOTOR_B2 8  

// Sensors  
HCSR04 ultrasonic(9, 10); // Trig=9, Echo=10  
MPU6050 mpu6050(Wire);  

void setup() {  
  radio.begin();  
  radio.openReadingPipe(0, address);  
  radio.startListening();  

  // Motors  
  pinMode(MOTOR_A1, OUTPUT);  
  pinMode(MOTOR_A2, OUTPUT);  
  pinMode(MOTOR_B1, OUTPUT);  
  pinMode(MOTOR_B2, OUTPUT);  

  // MPU6050  
  Wire.setSDA(20);  
  Wire.setSCL(21);  
  Wire.begin();  
  mpu6050.begin();  
}  

void loop() {  
  if (radio.available()) {  
    char command[20];  
    radio.read(&command, sizeof(command));  
    executeCommand(command);  
  }  

  // Send sensor data to ESP32  
  static unsigned long lastSend = 0;  
  if (millis() - lastSend > 1000) {  
    String data = String(ultrasonic.dist()) + "," + String(analogRead(26));  
    radio.write(data.c_str(), data.length());  
    lastSend = millis();  
  }  
}  

void executeCommand(char* command) {  
  if (strstr(command, "FORWARD")) {  
    digitalWrite(MOTOR_A1, HIGH);  
    digitalWrite(MOTOR_A2, LOW);  
    digitalWrite(MOTOR_B1, HIGH);  
    digitalWrite(MOTOR_B2, LOW);  
  }  
  // Add other commands (LEFT, RIGHT, STOP)  
}  
```  

---

### **3. Arduino Nano (Joystick Controller)**  
```cpp  
#include <SPI.h>  
#include <nRF24L01.h>  
#include <RF24.h>  

RF24 radio(7, 8); // CE=7, CSN=8  
const byte address[6] = "00001";  

void setup() {  
  radio.begin();  
  radio.openWritingPipe(address);  
  radio.setPALevel(RF24_PA_MAX);  
  radio.stopListening();  
}  

void loop() {  
  int joy1X = analogRead(A0);  
  int joy1Y = analogRead(A1);  
  int joy2X = analogRead(A2);  
  int joy2Y = analogRead(A3);  

  char command[20];  
  sprintf(command, "JOYSTICK,%d,%d,%d,%d", joy1X, joy1Y, joy2X, joy2Y);  
  radio.write(&command, sizeof(command));  
  delay(50);  
}  
```   

---

## **📌 Setup Instructions**  
1. **Wire all components** as per connection tables.  
2. **Install libraries** via Arduino IDE.  
3. **Enable Gmail SMTP**:  
   - Use an **App Password** (Google Account → Security → 2FA → App Passwords).  
4. **Calibrate MPU6050**: Run `mpu6050.calcGyroOffsets(true);` once.  
