bool ledState = false;
bool lastButtonState = HIGH;
#include <WiFi.h>
#include <esp_now.h>
#include <DHT.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <TFT_eSPI.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <PulseSensorPlayground.h>  // Includes the PulseSensorPlayground Library.

//  Variables
const int PulseWire = 34;
int Threshold = 1000;
#define BUTTON_PIN 12        // Push button connected to GPIO 2
#define PANIC_BUTTON_PIN 13  // Panic Button
#define BUZZER 14

TFT_eSPI tft = TFT_eSPI();
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

#include "MAX30105.h"
#include "heartRate.h"


const int NO_MOTION_TIME = 5000;  // Time in milliseconds (5 seconds)
const float THRESHOLD = 0.5;      // Sensitivity threshold (adjust as needed)
unsigned long lastMovementTime = 0;


// 🔹 Device Identification
#define DEVICE_NAME "MAVERICK"
char receivedDevice[] = "GOOSE";
// 🔹 Peer MAC Addresses
uint8_t peerMAC1[] = {0xAC,0x15,0x18,0xEA,0x02,0x58 };  // MAVERICK MAC 
uint8_t peerMAC2[] = { 0xF0, 0x24, 0xF9, 0x5A, 0x4E, 0x6C };  // GOOSE MAC
 // F0:24:F9:5A:4E:6C 

uint8_t *peerMAC = (strcmp(DEVICE_NAME, "MAVERICK") == 0) ? peerMAC2 : peerMAC1;

// 🔹 DHT Sensor Setup
#define DHTPIN 15
#define DHTTYPE DHT11
DHT dht(DHTPIN, DHTTYPE);

// 🔹 MPU6050 Sensor Setup
Adafruit_MPU6050 mpu;
PulseSensorPlayground pulseSensor;  // Creates an instance of the PulseSensorPlayground object called "pulseSensor"
                                    // 🔹 Heartbeat Sensor Pin
//char receivedDevice = received_device;


// 🔹 Data Structure
typedef struct {
  //char deviceName[20];  // 📌 Added Device Name (max 20 characters)
  float latitude;
  float longitude;
  uint8_t hour, minute, second;
  uint8_t day, month;
  uint16_t year;
  float temperature;
  float humidity;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  float heartbeat;
  bool panicAlert;
} testData_t;

testData_t sendData;
testData_t receivedData;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.printf("Data Sent - Status: %s\n", (status == ESP_NOW_SEND_SUCCESS) ? "Success" : "Fail");
}

void onDataRecv(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&receivedData, incomingData, sizeof(receivedData));

  Serial.println("\n📩 Data Received:");
  Serial.printf("Latitude: %.6f, Longitude: %.6f\n", receivedData.latitude, receivedData.longitude);
  Serial.printf("Time: %02d:%02d:%02d, Date: %02d/%02d/%d\n",
                receivedData.hour, receivedData.minute, receivedData.second,
                receivedData.day, receivedData.month, receivedData.year);
  Serial.printf("🌡 Temperature: %.1f°C, 💧 Humidity: %.1f%%\n", receivedData.temperature, receivedData.humidity);
  Serial.printf("📍 Acceleration: X=%.2f Y=%.2f Z=%.2f m/s²\n", receivedData.accelX, receivedData.accelY, receivedData.accelZ);
  Serial.printf("🔄 Gyroscope: X=%.2f Y=%.2f Z=%.2f rad/s\n", receivedData.gyroX, receivedData.gyroY, receivedData.gyroZ);
  Serial.printf("❤️ Heartbeat: %.2f\n", receivedData.heartbeat);

  if (receivedData.panicAlert) {
    Serial.printf("🚨 PANIC ALERT from [%s]! Device in danger! 🚨\n", receivedDevice);
    panic_alert();
    digitalWrite(BUZZER,HIGH);
    delay(100);
    digitalWrite(BUZZER,LOW);
    delay(100);
  }
  else{
    digitalWrite(BUZZER,LOW);
  }
}
MAX30105 particleSensor;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  WiFi.mode(WIFI_STA);
  dht.begin();
  Wire.begin();

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050");

    //while (1) { delay(10); }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_5_HZ);

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW Init Failed");
    return;
  }
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, peerMAC, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }
  Serial.println("Initializing MAX30105...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30105 was not found. Please check wiring/power.");
    //while (1)
      ;
  }

  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x0A);
  particleSensor.setPulseAmplitudeGreen(0);
  tft.begin();
  tft.setRotation(4);
  pinMode(BUTTON_PIN, INPUT_PULLUP);        // Enable internal pull-up resistor
  pinMode(PANIC_BUTTON_PIN, INPUT_PULLUP);        // Enable internal pull-up resistor
 pinMode(BUZZER, OUTPUT);  // buzzer

  // attachIntert(digitalPinToInterrupt(BUTTON_PIN), handleButtonPress, FALLING);
  pulseSensor.analogInput(PulseWire);
  pulseSensor.setThreshold(Threshold);
  if (pulseSensor.begin()) {
    Serial.println("We created a pulseSensor Object !");  //This prints one time at Arduino power-up,  or on Arduino reset.
  }
}
bool lastPanicButtonState;
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0;
long lastBeat = 0;
float beatsPerMinute;
int beatAvg;
void loop() {
  sendData.panicAlert = false;  // Reset panic flag

  if (digitalRead(PANIC_BUTTON_PIN) == LOW && lastPanicButtonState == HIGH) {
    sendData.panicAlert = true;  // 🔴 Set panic flag when button is pressed
    Serial.println("🚨 PANIC BUTTON PRESSED! SENDING ALERT... 🚨");
    delay(500);  // Debounce delay
  }
  lastPanicButtonState = digitalRead(PANIC_BUTTON_PIN);
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {
    sendData.latitude = gps.location.lat();
    sendData.longitude = gps.location.lng();
  }
  if (gps.date.isValid()) {
    sendData.day = gps.date.day();
    sendData.month = gps.date.month();
    sendData.year = gps.date.year();
  }
  if (gps.time.isValid()) {
    sendData.hour = gps.time.hour();
    sendData.minute = gps.time.minute();
    sendData.second = gps.time.second();
  }

  sendData.temperature = dht.readTemperature();
  sendData.humidity = dht.readHumidity();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  sendData.accelX = a.acceleration.x;
  sendData.accelY = a.acceleration.y;
  sendData.accelZ = a.acceleration.z;
  sendData.gyroX = g.gyro.x;
  sendData.gyroY = g.gyro.y;
  sendData.gyroZ = g.gyro.z;
  int myBPM;
  if (pulseSensor.sawStartOfBeat()) {              // Constantly test to see if "a beat happened".
    myBPM = pulseSensor.getBeatsPerMinute();       // Calls function on our pulseSensor object that returns BPM as an "int".
                                                   // "myBPM" hold this BPM value now.
    Serial.println("♥  A HeartBeat Happened ! ");  // If test is "true", print a message "a heartbeat happened".
    Serial.print("BPM: ");                         // Print phrase "BPM: "
    Serial.println(myBPM);                         // Print the value inside of myBPM.
  }
  sendData.heartbeat = myBPM;

  esp_now_send(peerMAC, (uint8_t *)&sendData, sizeof(sendData));

  Serial.printf("\n📤 Sending Data: Latitude=%.6f, Longitude=%.6f\n", sendData.latitude, sendData.longitude);
  Serial.printf("Time: %02d:%02d:%02d, Date: %02d/%02d/%d\n",
                sendData.hour, sendData.minute, sendData.second,
                sendData.day, sendData.month, sendData.year);
  Serial.printf("🌡 Temperature: %.1f°C, 💧 Humidity: %.1f%%\n", sendData.temperature, sendData.humidity);
  Serial.printf("📍 Acceleration: X=%.2f Y=%.2f Z=%.2f m/s²\n", sendData.accelX, sendData.accelY, sendData.accelZ);
  Serial.printf("🔄 Gyroscope: X=%.2f Y=%.2f Z=%.2f rad/s\n", sendData.gyroX, sendData.gyroY, sendData.gyroZ);
  Serial.printf("❤️ Heartbeat: %.2f\n", sendData.heartbeat);
  //handleButtonPress();
  bool buttonState = digitalRead(BUTTON_PIN);
  Serial.println(buttonState);
  // Toggle on button press (LOW)
  if (buttonState == LOW && lastButtonState == HIGH) {
    ledState = !ledState;  // Toggle LED state
    //digitalWrite(LED_PIN, ledState);
    if (ledState == 0) {
      received_display();

    } else {
      update_display();
    }
    delay(100);  // Simple debounce delay
  }
  //panicalert();
  if (ledState == 0) {
    received_display();

  } else {
    update_display();
  }

  lastButtonState = buttonState;
  bool motion = 1;
  if (abs(receivedData.accelY) > THRESHOLD) {
    lastMovementTime = millis();         // Reset timer
    Serial.println("Motion Deteceted");  // motion =0
  }

  // Check if no motion for the set time
  if (millis() - lastMovementTime > NO_MOTION_TIME) {
    Serial.println("No Motion Detected!");
    motion = 0;
    lastMovementTime = millis();  // Prevent continuous alerts
  } else {
    motion = 1;
  }

  if (receivedData.temperature > 34) {
    dangertemp();
    delay(100);
  }
  if (motion == 0) {
    dangermotion();
    delay(100);
  }
  //update_display();

  esp_now_send(peerMAC, (uint8_t *)&sendData, sizeof(sendData));

  if (sendData.panicAlert) {    
    Serial.println("🚨 PANIC ALERT SENT! 🚨");
  }



    long irValue = particleSensor.getIR();
    
    if (checkForBeat(irValue)) {
        long delta = millis() - lastBeat;
        lastBeat = millis();

        beatsPerMinute = 60 / (delta / 1000.0);

        if (beatsPerMinute < 255 && beatsPerMinute > 20) {
            rates[rateSpot++] = (byte)beatsPerMinute;
            rateSpot %= RATE_SIZE;

            beatAvg = 0;
            for (byte x = 0; x < RATE_SIZE; x++)
                beatAvg += rates[x];
            beatAvg /= RATE_SIZE;
        }
    }

    Serial.print("IR=");
    Serial.print(irValue);
    Serial.print(", BPM=");
    Serial.print(beatsPerMinute);
    Serial.print(", Avg BPM=");
    Serial.print(beatAvg);

    if (irValue < 50000)
        Serial.print(" No finger?");

    Serial.println();

    sendData.heartbeat = beatAvg;

  delay(2000);
}
void update_display() {
  tft.fillRect(0, 0, 128, 161, TFT_BLACK);  // Clear display

  tft.setTextColor(TFT_ORANGE);
  tft.setTextSize(1);
  tft.setFreeFont();
  tft.drawString("Local Data", 37, 2);

  // GPS Section
  tft.drawRect(0, 14, 128, 13, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("GPS", 3, 17);

  tft.drawRect(0, 27, 128, 35, TFT_YELLOW);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Latitude :", 4, 30);
  tft.drawString("Longitude:", 4, 40);
  tft.drawString("Time", 4, 51);

  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.latitude), 74, 30);
  tft.drawString(String(sendData.longitude), 74, 41);
  tft.drawString(String(sendData.year), 75, 52);

  // MPU Section
  tft.drawRect(0, 62, 128, 14, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("MPU", 3, 65);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Acclero", 3, 79);

  tft.drawString("X:", 4, 90);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.accelX, 2), 23, 90);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Y:", 44, 90);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.accelY, 2), 61, 90);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Z:", 85, 90);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.accelZ, 2), 98, 90);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Gyro", 3, 102);

  tft.drawString("X:", 2, 113);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.gyroX, 2), 23, 113);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Y:", 44, 113);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.gyroY, 2), 63, 113);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Z:", 85, 113);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.gyroZ, 2), 101, 113);

  // Heartbeat Section
  tft.drawRect(0, 123, 128, 10, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Heartbeat:", 3, 124);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.heartbeat, 2), 68, 124);

  // Temperature Section
  tft.drawRect(0, 133, 128, 12, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Temperature:", 3, 135);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.temperature, 2), 69, 135);

  // Humidity Section
  tft.drawRect(0, 145, 128, 14, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Humidity:", 4, 148);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(sendData.humidity, 2), 70, 148);
}
void received_display() {
  tft.fillRect(0, 0, 128, 161, TFT_BLACK);  // Clear display

  tft.setTextColor(TFT_ORANGE);
  tft.setTextSize(1);
  tft.setFreeFont();
  tft.drawString("GOOSE", 37, 2);

  // GPS Section
  tft.drawRect(0, 14, 128, 13, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("GPS", 3, 17);

  tft.drawRect(0, 27, 128, 35, TFT_YELLOW);
  tft.setTextColor(TFT_WHITE);
  tft.drawString("Latitude :", 4, 30);
  tft.drawString("Longitude:", 4, 40);
  tft.drawString("Time", 4, 51);

  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.latitude), 74, 30);
  tft.drawString(String(receivedData.longitude), 74, 41);
  tft.drawString(String(receivedData.year), 75, 52);

  // MPU Section
  tft.drawRect(0, 62, 128, 14, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("MPU", 3, 65);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Acclero", 3, 79);

  tft.drawString("X:", 4, 90);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.accelX, 2), 23, 90);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Y:", 44, 90);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.accelY, 2), 61, 90);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Z:", 85, 90);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.accelZ, 2), 98, 90);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Gyro", 3, 102);

  tft.drawString("X:", 2, 113);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.gyroX, 2), 23, 113);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Y:", 44, 113);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.gyroY, 2), 63, 113);

  tft.setTextColor(TFT_WHITE);
  tft.drawString("Z:", 85, 113);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.gyroZ, 2), 101, 113);

  // Heartbeat Section
  tft.drawRect(0, 123, 128, 10, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Heartbeat:", 3, 124);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.heartbeat, 2), 68, 124);

  // Temperature Section
  tft.drawRect(0, 133, 128, 12, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Temperature:", 3, 135);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.temperature, 2), 69, 135);

  // Humidity Section
  tft.drawRect(0, 145, 128, 14, TFT_YELLOW);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("Humidity:", 4, 148);
  tft.setTextColor(TFT_CYAN);
  tft.drawString(String(receivedData.humidity, 2), 70, 148);
}

void handleButtonPress() {
  bool ledState = !ledState;
  if (ledState == 0) {
    update_display();
  } else {
    received_display();
  };
}

void dangertemp() {
  tft.fillRect(1, 0, 128, 160, 0x0);

  tft.setTextColor(0xFAAA);
  tft.setTextSize(1);
  tft.setFreeFont();
  tft.drawString("Warning  !!!", 34, 24);

  tft.drawString("ABNORMAL TEMPERATURE", 5, 83);

  tft.drawString("DETECECTED", 36, 105);
}

void dangermotion() {
  tft.fillRect(1, 0, 128, 160, 0x0);

  tft.setTextColor(0xFAAA);
  tft.setTextSize(1);
  tft.setFreeFont();
  tft.drawString("Warning  !!!", 34, 24);

  tft.drawString("NO MOTION DETECTED", 7, 83);

  //tft.drawString("DETECECTED", 36, 105);
}
void panic_alert() {
  tft.fillRect(1, 0, 128, 160, 0x0);

  tft.setTextColor(0xFAAA);
  tft.setTextSize(1);
  tft.setFreeFont();
  tft.drawString("Warning  !!!", 34, 24);

  tft.drawString("PANIC AlERT GOOSE", 5, 83);

  tft.drawString(receivedDevice, 36, 105);
}
