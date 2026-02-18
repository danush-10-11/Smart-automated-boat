# Autonomous GPS-Navigated Rescue Vessel

An Arduino-based autonomous boat designed for water-based rescue missions. Features real-time GPS waypoint navigation, ultrasonic obstacle avoidance, and Bluetooth manual override.

## 🚀 Key Features
* **Autopilot Mode:** Uses Haversine formula to calculate distance and bearing to a saved "Home" coordinate.
* **Safety First:** Integrated HC-SR04 sensors for automatic emergency braking.
* **Dual Control:** Seamless switching between manual Bluetooth control and autonomous navigation.
* **Kickstart Logic:** Custom PWM logic to overcome static water resistance.

## 🛠️ Tech Stack
* **Hardware:** Arduino Nano, NEO-6M GPS, HC-05 Bluetooth, L293D Motor Driver, MG995 Servo.
* **Libraries:** TinyGPS++, SoftwareSerial, Servo.

## 📸 System Architecture
![Wiring Diagram](link-to-your-image-here.jpg)
