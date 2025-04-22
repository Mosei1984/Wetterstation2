https://www.az-delivery.de/blogs/azdelivery-blog-fur-arduino-und-raspberry-pi/part-2

Weather Station with Multi-Display Setup
Overview
This project is a sophisticated weather station that utilizes multiple circular displays to show different weather metrics including temperature, humidity, barometric pressure, and time. The system features an analog-style presentation with moving needle indicators and includes LED visual effects to represent current weather conditions.

Features
Three GC9A01A Round Displays:

Temperature & Humidity comfort meter display
Barometric pressure display with analog needle
Analog clock with date display
Multiple Sensors:

BMP180 pressure sensor with calibration capability
DHT22 temperature and humidity sensor
DS3231 real-time clock module
Visual Indicators:

RGB LED weather status indicators with dynamic effects
Mechanical stepper motor for physical weather status indicator
Lightning effects during stormy conditions
Interactive Controls:

Serial command interface for settings and calibration
EEPROM storage for calibration data persistence
Weather Prediction:

Weather forecasting based on barometric pressure trends
Dynamic visual representation of current conditions
Hardware Requirements
Arduino-compatible microcontroller
3× GC9A01A round LCD displays (240×240 pixels)
BMP180 pressure sensor
DHT22 temperature and humidity sensor
DS3231 RTC module
Stepper motor
WS2812B LED strip (10 LEDs)
Limit switches for stepper calibration
Various resistors, wires, and power supply
Installation & Setup
Connect all hardware components according to the pin definitions in the code
Install the required libraries:
Adafruit_GC9A01A
BMP180
RTClib
DHT
FastLED
SPI
Wire
Upload the code to your Arduino-compatible board
First time setup will require RTC time setting and sensor calibration
Usage
The weather station operates automatically once powered and initialized. The displays show:

Temperature and humidity on the "Comfort Meter" display
Current barometric pressure on the barometer display
Current time and date on the clock display
The LED strip provides ambient indication of weather conditions:

Purple: Stormy weather
Blue: Rainy conditions
Yellow: Changing weather
Light blue: Fair weather
Orange: Sunny conditions
During stormy conditions, the LEDs will occasionally flash to simulate lightning.

Calibration
The system supports various calibration options through the serial interface (9600 baud):

Available commands:
settime - Set date and time
rawpressure - Show raw pressure sensor data
calibrate - Calibrate pressure sensor
calibratestepper - Calibrate stepper position
resetcalibration - Reset all calibration data
help - Show this help

Copy


Pressure Calibration
The pressure sensor can be calibrated by comparing its readings with a known reference and storing the offset in EEPROM.

Stepper Calibration
The stepper motor can be calibrated to properly indicate weather conditions by setting the pressure thresholds for "rain" and "fair" weather.

Contributing
Contributions to improve the weather station functionality are welcome. Please feel free to submit pull requests or open issues for bugs and feature requests.
