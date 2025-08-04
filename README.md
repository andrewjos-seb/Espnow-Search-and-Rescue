

# 🚨 Search and Rescue IoT System - MAVERICK & GOOSE

This project is a **Search and Rescue Communication & Monitoring System** between two ESP32-based wearable devices codenamed **MAVERICK** and **GOOSE**. Using ESP-NOW for low-power peer-to-peer communication, the system continuously shares **critical data** such as GPS, vitals (heartbeat, temperature, humidity), motion status, and panic alerts.

## 📦 Features

* 🔄 **Two-Way Communication** (via ESP-NOW)
* 📡 **GPS Data Exchange** (Latitude, Longitude, Date, Time)
* 🌡 **Environmental Monitoring** (Temperature & Humidity via DHT11)
* 🩺 **Heartbeat Monitoring** (Pulse Sensor & MAX30105)
* ⚙️ **Motion & Orientation Sensing** (MPU6050 IMU)
* 🚨 **Panic Button System**
* 📟 **TFT Display Output** (Local vs. Remote data toggle via button)
* 🔊 **Buzzer Feedback** for Alerts
* 🛑 **No Motion Warning** (e.g. unconsciousness)
* ⚠️ **Abnormal Temperature Warning**
* 🔁 **Dual Heartbeat Source:** PulseSensor & MAX30105 Fusion

---

## 🔧 Hardware Requirements

| Component             | Description                       |
| --------------------- | --------------------------------- |
| ESP32 Dev Board (x2)  | One for MAVERICK, one for GOOSE   |
| DHT11                 | Temperature & Humidity sensor     |
| Pulse Sensor          | Heartbeat sensor (analog)         |
| MAX30105              | Optical pulse and SpO2 sensor     |
| MPU6050               | IMU sensor (Accelerometer + Gyro) |
| TFT Display (128x160) | SPI-based color screen            |
| GPS Module (Neo6M)    | GPS via UART                      |
| Push Buttons (x2)     | Data Toggle & Panic Button        |
| Buzzer                | Alert output                      |
| Wires, Breadboard     | As needed                         |

---

## 📲 How It Works

Each device continuously:

1. Reads sensor data (GPS, MPU6050, DHT11, Heartbeat)
2. Checks panic button state
3. Sends its data to the peer device over **ESP-NOW**
4. Displays either local or received data on the **TFT screen**
5. Triggers alerts (no motion, abnormal temperature, panic) using:

   * TFT screen warning
   * Buzzer sound

---

## 🧩 Libraries Used

Ensure the following libraries are installed in Arduino IDE:

* `esp_now.h`
* `WiFi.h`
* `Wire.h`
* `DHT.h`
* `Adafruit_MPU6050`
* `Adafruit_Sensor`
* `TFT_eSPI`
* `TinyGPS++`
* `PulseSensorPlayground`
* `MAX30105` + `heartRate.h`

---

## 🔌 Pin Connections (MAVERICK/GOOSE)

| Signal         | ESP32 Pin                                  |
| -------------- | ------------------------------------------ |
| DHT11          | GPIO 15                                    |
| Pulse Sensor   | GPIO 34                                    |
| Panic Button   | GPIO 13                                    |
| Toggle Button  | GPIO 12                                    |
| Buzzer         | GPIO 14                                    |
| MAX30105 (I2C) | SDA/SCL                                    |
| MPU6050 (I2C)  | SDA/SCL                                    |
| GPS RX/TX      | GPIO 16 / 17                               |
| TFT (SPI)      | Default SPI pins (as per TFT\_eSPI config) |

---

## 📺 Display Functionality

The TFT screen shows:

* GPS Location & Timestamp
* Acceleration & Gyro values
* Temperature & Humidity
* Heartbeat (BPM)
* Toggle between local and remote device data using button

---

## 🔔 Alert Conditions

| Condition               | Response                       |
| ----------------------- | ------------------------------ |
| Panic Button Pressed    | Display alert + buzzer alert   |
| No Motion Detected (5s) | Display alert + buzzer alert   |
| Temperature > 34°C      | Display alert + buzzer alert   |
| No Finger on Sensor     | Skip BPM update, print warning |

---

## 🔗 Communication Setup

* Devices use **ESP-NOW** for fast, low-power, local wireless communication.
* Each device knows the other's MAC address and sends data periodically (every \~2s).
* Device role is set using:

  ```cpp
  #define DEVICE_NAME "MAVERICK"
  ```

---

## 🛠 Configuration Notes

* Change `DEVICE_NAME` to `"GOOSE"` on the second device.
* Update peer MAC addresses appropriately:

  ```cpp
  uint8_t peerMAC1[] = {MAC for MAVERICK};
  uint8_t peerMAC2[] = {MAC for GOOSE};
  ```

---

## ✅ Future Improvements

* Add SpO₂ monitoring from MAX30105
* Add location mapping via Bluetooth or Wi-Fi to phone
* Integrate battery monitoring
* Add haptic feedback
* Use BLE mesh for long-range team communication

---

## 🧑‍🚀 Author

**Andrewjos Sebastian**
Robotics & Automation | IoT Developer | Embedded Systems

