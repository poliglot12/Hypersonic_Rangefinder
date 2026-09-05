# Hypersonic_Rangefinder

* Ultrasonic Rangefinding Radar

## Hardware

1. ESP32 Dev Board
2. HC-SR04 Ultrasonic Sensor
3. SG90 Micro Servo Motor
4. SSD1306 OLED Display
5. Passive Piezo Buzzer
6. LED

## Behaviour

* Servo sweeps the sensor from 20 to 180 degrees and back.
* Distance is measured at each position.
* OLED displays the angle and distance.
* LED turns on if distance is less than 50 cm.
* Piezo beeps with frequency inversely proportional to distance.