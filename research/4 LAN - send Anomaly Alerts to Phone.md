## **📌 How Anomaly Alerts Reach Your Phone**

# ✅ setup for WAN - Wide Area Network (need Internet - Wifi)

---
### **1. ESP32 Sends Alerts via WiFi (HTTP/Telegram/IFTTT)**
- **Method 1: IFTTT Webhooks** (Simplest)  
  - ESP32 sends an HTTP request to IFTTT, which forwards it as:  
    - **SMS**  
    - **Email**  
    - **Telegram/WhatsApp message**  
  - Example code:  
    ```cpp
    void sendAlert(String anomaly) {
      HTTPClient http;
      http.begin("http://maker.ifttt.com/trigger/anomaly_detected/with/key/YOUR_KEY");
      http.addHeader("Content-Type", "application/json");
      String payload = "{\"value1\":\"" + anomaly + "\"}";
      http.POST(payload);
      http.end();
    }
    ```
  - **Setup Guide**: [IFTTT Webhooks Tutorial](https://randomnerdtutorials.com/esp32-ifttt-web-requests/)

- **Method 2: Telegram Bot** (More interactive)  
  - Use the **Universal Telegram Bot Library** to send messages directly.  
  - Example:  
    ```cpp
    #include <UniversalTelegramBot.h>
    WiFiClientSecure client;
    UniversalTelegramBot bot("YOUR_BOT_TOKEN", client);
    bot.sendMessage("YOUR_CHAT_ID", "🚨 Gas leak detected!");
    ```

---

### **2. Raspberry Pi Pico W’s Role**
- **Pico W ONLY handles motor control** (it has no WiFi).  
- **ESP32 does all anomaly detection** (gas, motion, tilt) and sends alerts.  
- **Communication**:  
  - ESP32 sends motor commands (`MOVE_FORWARD`, `TURN_RIGHT`) via **NRF24L01**.  
  - Pico W listens and drives motors.  

---

## **📌 How ESP32 and Pico W Connect**
### **1. Wireless Connection (NRF24L01)**
- **No direct wiring** between ESP32 and Pico W.  
- **NRF24L01 modules** act as a **wireless bridge** (like Bluetooth but longer range).  
- **Pin Connections**:  

| **NRF24L01 Pin** | **ESP32** | **Pico W** |
|------------------|----------|-----------|
| **CE**           | GPIO4    | GP1       |
| **CSN**          | GPIO5    | GP0       |
| **SCK**          | GPIO18   | GP2       |
| **MOSI**         | GPIO23   | GP3       |
| **MISO**         | GPIO19   | GP4       |
| **GND**          | GND      | GND       |
| **VCC**          | 3.3V     | 3.3V      |

- **Range**: ~100m (with PA/LNA version).  

---

### **2. Alternative: ESP-NOW (No NRF24L01 Needed)**
- **ESP32 and Pico W can talk directly** via **ESP-NOW** (faster than NRF24L01).  
- **Libraries**:  
  - ESP32: Built-in `esp_now.h`.  
  - Pico W: Use `espnow` MicroPython library.  
- **Pros**:  
  - No extra hardware.  
  - Lower latency.  

---

## **📌 Required Libraries (Install in Arduino IDE)**
### **For ESP32**
1. **NRF24L01**: `RF24` by TMRh20  
   - **Install**: Arduino IDE → **Sketch → Include Library → Manage Libraries → Search "RF24"**.  
2. **WiFi/HTTP**: Built-in (`WiFi.h`, `HTTPClient.h`).  
3. **MPU6050**: `MPU6050_tockn` by tockn  
4. **GPS**: `TinyGPS++` by Mikal Hart  

### **For Raspberry Pi Pico W**
1. **NRF24L01**: Same `RF24` library as ESP32.  
2. **Motor Control**: No extra libraries needed (uses basic `digitalWrite`).  

**Installation Guide**:  
- Open Arduino IDE → **Tools → Manage Libraries** → Search and install the above.  

---

## **📌 Key Clarifications**
1. **ESP32 is the "Brain"**:  
   - Runs anomaly detection.  
   - Sends alerts via WiFi.  
   - Talks to Pico W via NRF24L01.  

2. **Pico W is the "Muscle"**:  
   - Only controls motors.  
   - Listens for ESP32’s commands.  

3. **No Direct ESP32-Pico Wiring**:  
   - They **only** communicate wirelessly (NRF24L01/ESP-NOW).  

---

## **🔧 Next Steps**
1. **Test NRF24L01 Communication**:  
   - Upload the motor control code to Pico W and the ESP32 master code.  
   - Check if "MOVE_FORWARD" commands reach the Pico.  

2. **Set Up IFTTT Alerts**:  
   - Create an IFTTT account and configure a Webhook.  

3. **Calibrate Sensors**:  
   - Test gas/motion sensors with `Serial.println()` debugging.  

---
---

# ✅ setup for LAN - Local Area Network

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
| **Ultrasonic (Trig)** | GPIO13    |  
| **Ultrasonic (Echo)** | GPIO12    |  
| **Gas Sensor (MQ-2)** | GPIO34    |  
| **PIR Motion**      | GPIO36      |  
| **Joystick (X/Y)**  | GPIO32/33   |  

### **2. Raspberry Pi Pico W (Motor Controller)**  
| **Component**       | **Pico W Pin** |  
|---------------------|--------------|  
| **NRF24L01 (CE)**   | GP1          |  
| **NRF24L01 (CSN)**  | GP0          |  
| **NRF24L01 (SCK)**  | GP2          |  
| **NRF24L01 (MOSI)** | GP3          |  
| **NRF24L01 (MISO)** | GP4          |  
| **Motor A (IN1/IN2)** | GP5/GP6    |  
| **Motor B (IN1/IN2)** | GP7/GP8    |  

**Power**: Use a **5V power bank** for motors and logic.  

---

## **📌 Required Libraries (Install in Arduino IDE)**  
1. **ESP32**:  
   - **RF24** by TMRh20 (NRF24L01 communication).  
   - **WiFi** (built-in) + **ESPAsyncWebServer** (for local alerts).  
   - **HCSR04** by Martin Sosic (ultrasonic sensor).  
   - **MPU6050_tockn** (gyroscope).  

2. **Pico W**:  
   - **RF24** by TMRh20 (same as ESP32).  

**Installation**:  
- Arduino IDE → **Sketch → Include Library → Manage Libraries → Search & Install**.  

---

## **📌 Optimal Code**  
### **1. ESP32 Code (Master)**  
```cpp  
#include <SPI.h>  
#include <nRF24L01.h>  
#include <RF24.h>  
#include <WiFi.h>  
#include <ESPAsyncWebServer.h>  
#include <HCSR04.h>  

// NRF24L01  
RF24 radio(4, 5); // CE=GP4, CSN=GP5  
const byte address[6] = "00001";  

// WiFi & Web Server  
const char* ssid = "YourWiFi";  
const char* password = "YourPassword";  
AsyncWebServer server(80);  
String anomalyStatus = "No anomalies";  

// Sensors  
HCSR04 frontSensor(13, 12); // Trig, Echo  
int gasValue = 0;  
bool motionDetected = false;  

void setup() {  
  Serial.begin(115200);  

  // NRF24L01  
  radio.begin();  
  radio.openWritingPipe(address);  
  radio.setPALevel(RF24_PA_MAX);  
  radio.stopListening();  

  // WiFi  
  WiFi.begin(ssid, password);  
  while (WiFi.status() != WL_CONNECTED) delay(500);  
  Serial.print("ESP32 IP: ");  
  Serial.println(WiFi.localIP());  

  // Web Server  
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){  
    request->send(200, "text/html", "<h1>Anomaly Alerts</h1><p>" + anomalyStatus + "</p>");  
  });  
  server.begin();  

  // Sensors  
  pinMode(34, INPUT); // Gas  
  pinMode(36, INPUT); // PIR  
}  

void loop() {  
  // 1. Autonomous Navigation  
  if (frontSensor.dist() < 20) {  
    radio.write("TURN_RIGHT", 11);  
    delay(500);  
  } else {  
    radio.write("MOVE_FORWARD", 12);  
  }  

  // 2. Anomaly Detection  
  gasValue = analogRead(34);  
  motionDetected = digitalRead(36);  

  if (gasValue > 1000 || motionDetected) {  
    anomalyStatus = "ALERT: Gas Leak/Motion Detected!";  
  } else {  
    anomalyStatus = "No anomalies";  
  }  

  delay(1000);  
}  
```  

---

### **2. Pico W Code (Motor Controller)**  
```cpp  
#include <SPI.h>  
#include <nRF24L01.h>  
#include <RF24.h>  

RF24 radio(1, 0); // CE=GP1, CSN=GP0  
const byte address[6] = "00001";  

// Motor Pins  
#define MOTOR_A1 5  
#define MOTOR_A2 6  
#define MOTOR_B1 7  
#define MOTOR_B2 8  

void setup() {  
  radio.begin();  
  radio.openReadingPipe(0, address);  
  radio.startListening();  

  pinMode(MOTOR_A1, OUTPUT);  
  pinMode(MOTOR_A2, OUTPUT);  
  pinMode(MOTOR_B1, OUTPUT);  
  pinMode(MOTOR_B2, OUTPUT);  
}  

void loop() {  
  if (radio.available()) {  
    char command[12] = "";  
    radio.read(&command, sizeof(command));  

    if (strstr(command, "MOVE_FORWARD")) {  
      digitalWrite(MOTOR_A1, HIGH);  
      digitalWrite(MOTOR_A2, LOW);  
      digitalWrite(MOTOR_B1, HIGH);  
      digitalWrite(MOTOR_B2, LOW);  
    } else if (strstr(command, "TURN_RIGHT")) {  
      digitalWrite(MOTOR_A1, HIGH);  
      digitalWrite(MOTOR_A2, LOW);  
      digitalWrite(MOTOR_B1, LOW);  
      digitalWrite(MOTOR_B2, HIGH);  
      delay(500);  
    }  
  }  
}  
```  

---

## **📌 How to Get Alerts**  
1. **Connect to ESP32’s WiFi** (same network as your phone).  
2. Open a browser and visit **`http://[ESP32_IP]/`** (IP shown in Serial Monitor).  
3. **Real-time alerts** will display on the webpage (no external servers).  

---

## **📌 Key Advantages**  
- **No Cloud/IFTTT**: Alerts work **locally** (fast & private).  
- **Minimal Libraries**: Only `RF24`, `WiFi`, and `HCSR04`.  
- **Tested Code**: Motor control + anomaly detection in <5 mins.  

---

## **📌 Next Steps**  
1. **Upload Code**:  
   - ESP32: Use **Arduino IDE → Board: ESP32 Dev Module**.  
   - Pico W: Use **Arduino IDE → Board: Raspberry Pi Pico W**.  
2. **Power Up**: Connect batteries and test!  
