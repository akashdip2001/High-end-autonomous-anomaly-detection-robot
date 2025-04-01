Because of I am *familiar with Arduino IDE* with *ESP32 + Raspberry Pi Pico W* robot, So i desided to choose CPP.

## *📌 Hardware Connections*  
### *1. ESP32 (Master Controller)*
| *Component*       | *ESP32 Pin*  | *Notes*                     |
|---------------------|--------------|-----------------------------|
| *NRF24L01 (CE)*   | GPIO4        | Radio enable                 |
| *NRF24L01 (CSN)*  | GPIO5        | Chip select                  |
| *NRF24L01 (SCK)*  | GPIO18       | SPI clock                    |
| *NRF24L01 (MOSI)* | GPIO23       | SPI data out                 |
| *NRF24L01 (MISO)* | GPIO19       | SPI data in                  |
| *Ultrasonic (Trig)* | GPIO13      | Front sensor trigger         |
| *Ultrasonic (Echo)* | GPIO12      | Front sensor echo            |
| *PIR Motion*      | GPIO36       | Interrupt-capable pin        |
| *Gas Sensor (MQ-2)* | GPIO34      | Analog input                 |
| *GPS (TX)*        | GPIO16       | U2RX (Serial2)               |
| *GPS (RX)*        | GPIO17       | U2TX (Serial2)               |
| *Joystick (X-Axis)* | GPIO32     | Analog input (ADC1)          |
| *Joystick (Y-Axis)* | GPIO33     | Analog input (ADC1)          |

### *2. Raspberry Pi Pico W (Motor Controller)*
| *Component*       | *Pico W Pin* | *Notes*                    |
|---------------------|---------------|----------------------------|
| *NRF24L01 (CE)*   | GP1           | Radio enable                |
| *NRF24L01 (CSN)*  | GP0           | Chip select                 |
| *NRF24L01 (SCK)*  | GP2           | SPI clock                   |
| *NRF24L01 (MOSI)* | GP3           | SPI data out                |
| *NRF24L01 (MISO)* | GP4           | SPI data in                 |
| *Motor A (IN1)*   | GP5           | L298N driver input          |
| *Motor A (IN2)*   | GP6           | L298N driver input          |
| *Motor B (IN1)*   | GP7           | L298N driver input          |
| *Motor B (IN2)*   | GP8           | L298N driver input          |

---

## *🚀 C++ Code*  
### *1. ESP32 (Master) - Arduino IDE*  

```cpp
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <MPU6050.h>

// NRF24L01 Setup
RF24 radio(4, 5); // CE, CSN
const byte address[6] = "00001";

// WiFi & Alerts
const char* ssid = "YOUR_WIFI";
const char* password = "YOUR_PASS";
const char* serverUrl = "http://your-server.com/alerts";

// Sensors
#define TRIG_PIN 13
#define ECHO_PIN 12
#define PIR_PIN 36
#define GAS_PIN 34

TinyGPSPlus gps;
HardwareSerial SerialGPS(2); // UART2 (GPIO16=RX, GPIO17=TX)
MPU6050 mpu;

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
  Serial.println("WiFi Connected");

  // GPS
  SerialGPS.begin(9600, SERIAL_8N1, 16, 17);

  // MPU6050
  Wire.begin();
  mpu.initialize();

  // Ultrasonic
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  // 1. Autonomous Navigation
  avoidObstacles();

  // 2. Anomaly Detection
  checkAnomalies();

  // 3. Manual Control (Joystick)
  if (analogRead(32) > 2000) radio.write("TURN_RIGHT", 10);
}

void avoidObstacles() {
  long duration = pulseIn(ECHO_PIN, HIGH);
  int distance = duration * 0.034 / 2;
  
  if (distance < 20) {
    radio.write("TURN_RIGHT", 10);
    delay(500);
  } else {
    radio.write("MOVE_FORWARD", 12);
  }
}

void checkAnomalies() {
  // Gas Leak
  if (analogRead(GAS_PIN) > 1000) {
    sendAlert("GAS_LEAK");
  }

  // Motion Detection
  if (digitalRead(PIR_PIN)) {
    sendAlert("INTRUDER");
  }

  // Tilt Detection
  if (abs(mpu.getAccelerationX()) > 10000) {
    sendAlert("TILT");
  }
}

void sendAlert(String type) {
  if (WiFi.status() == WL_CONNECTED) {
    HTTPClient http;
    http.begin(serverUrl);
    http.addHeader("Content-Type", "application/json");
    String payload = "{\"type\":\"" + type + "\",\"location\":\"" + getGPSLocation() + "\"}";
    http.POST(payload);
    http.end();
  }
}

String getGPSLocation() {
  while (SerialGPS.available() > 0) {
    if (gps.encode(SerialGPS.read())) {
      if (gps.location.isValid()) {
        return String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6);
      }
    }
  }
  return "UNKNOWN";
}
```

---

### *2. Raspberry Pi Pico W (Motor Controller) - Arduino IDE*  

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

    if (strcmp(command, "MOVE_FORWARD") == 0) {
      moveForward();
    } else if (strcmp(command, "TURN_RIGHT") == 0) {
      turnRight();
    }
  }
}

void moveForward() {
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, HIGH);
  digitalWrite(MOTOR_B2, LOW);
}

void turnRight() {
  digitalWrite(MOTOR_A1, HIGH);
  digitalWrite(MOTOR_A2, LOW);
  digitalWrite(MOTOR_B1, LOW);
  digitalWrite(MOTOR_B2, HIGH);
}
```

---

## *🔧 Key Optimizations*
1. *NRF24L01: Uses **auto-acknowledgment* for reliable communication.  
2. *WiFi Alerts*: HTTP POST to a server (replace your-server.com with IFTTT/Telegram).  
3. *GPS Parsing: Uses **TinyGPS++* for accurate location logging.  
4. *Motor Control: L298N driver logic for **bidirectional movement*.  

---

## *📌 Next Steps*
1. *Test NRF24L01 Communication* (ESP32 → Pico W).  
2. *Calibrate Sensors* (Ultrasonic, MPU6050, Gas).  
3. *Implement SLAM* (Store ultrasonic data in an array for mapping).

## *📌 Future*
1. *ROS integration* for advanced navigation.
2. custom *PCB design*
