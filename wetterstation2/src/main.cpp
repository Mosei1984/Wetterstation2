#include <Arduino.h>
#include <Wire.h>
#include "SPI.h"
#include "Adafruit_GC9A01A.h"
#include "BMP180.h"
#include "RTClib.h"
#include <DHT.h>
#include "StepperControl.h"
#include "ErrorHandling.h"
#include <EEPROM.h>
#include <FastLED.h>

#ifndef DEG_TO_RAD
#define DEG_TO_RAD 0.017453292519943295
#endif
void drawComfortMeter(float temp, float humi);
void drawAnalogClock(DateTime now);
void updateBarometerDisplay(float pressure);


// EEPROM addresses for calibration data
#define EEPROM_CALIBRATION_MARKER 0    // Marker to check if calibration data exists
#define EEPROM_CALIBRATION_OFFSET 1    // Address for offset value (4 bytes)
// EEPROM addresses for stepper calibration
#define EEPROM_STEPPER_CALIBRATION_MARKER 10
#define EEPROM_RAIN_PRESSURE 11  // 4 bytes
#define EEPROM_FAIR_PRESSURE 15  // 4 bytes

// Default values for pressure ranges
float rainPressure = 900.0;  // Default value for rain
float fairPressure = 1020.0; // Default value for fair weather

// Define pins
#define tft_dc 7
#define tft_cs_pressure 10
#define tft_cs_clock 2
#define tft_cs_temperature 3

Adafruit_GC9A01A tft_pressure(tft_cs_pressure, tft_dc);
Adafruit_GC9A01A tft_clock(tft_cs_clock, tft_dc);
Adafruit_GC9A01A tft_temperature(tft_cs_temperature, tft_dc);

#define dht_pin 9
#define dht_type DHT22
DHT dht(dht_pin, dht_type);
float humi;
float temp;
char humidity[5];
char temperature[5];

BMP180 bmp180;
float pressu;
char pressure[6];

RTC_DS3231 rtc;
char daysOfTheWeek[7][12] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday", "Friday", "Saturday" };
int time_h;
int time_m;
char time_hours[2];
char time_minutes[2];

float previousPressure = 0;

// Timing variables
unsigned long previousMillis = 0;  // Stores the last time the loop was executed
const long interval = 2000;       // Interval at which to perform actions (milliseconds)

// Pressure calibration variables
float pressureCalibrationOffset = 0.0;
bool showRawPressure = false;
int center_x_temperature = 149;
int center_y_temperature = 150;
float pivot_x_temperature, pivot_y_temperature, pivot_x_temperature_old, pivot_temperature_y_old;
float p1_x_temperature, p1_y_temperature, p2_x_temperature, p2_y_temperature, p3_x_temperature, p3_y_temperature;
float p1_x_old_temperature, p1_y_old_temperature, p2_x_old_temperature, p2_y_old_temperature, p3_x_old_temperature, p3_y_old_temperature;
float arc_x_temperature;
float arc_y_temperature;
float needleAngle_temperature = 0;
float needle_setter_temperature;

int center_x_humidity = 89;
int center_y_humidity = 150;
float pivot_x_humidity, pivot_y_humidity, pivot_x_humidity_old, pivot_humidity_y_old;
float p4_x_humidity, p4_y_humidity, p5_x_humidity, p5_y_humidity, p6_x_humidity, p6_y_humidity;
float p4_x_old_humidity, p4_y_old_humidity, p5_x_old_humidity, p5_y_old_humidity, p6_x_old_humidity, p6_y_old_humidity;
float arc_x_humidity;
float arc_y_humidity;
float needleAngle_humidity = 0;
float needle_setter_humidity;

int radius_temperature = 100;
// --- Druckanzeige Barometer-Variablen ---
int center_x_pressure = 120;        // Zentrum des Kreises (x)
int center_y_pressure = 120;        // Zentrum des Kreises (y)
int radius_pressure = 65;           // Radius für die Skala

float needle_setter_pressure = 1000;  // Zeigerzielwert (wird später gesetzt)
float pivot_x_pressure, pivot_y_pressure;  // Drehpunkt des Zeigers

// Aktuelle und alte Werte für die Zeigerspitze (für Löschen/Zeichnen)
float p1_x_pressure, p1_y_pressure;
float p2_x_pressure, p2_y_pressure;
float p3_x_pressure, p3_y_pressure;

float p1_x_old_pressure, p1_y_old_pressure;
float p2_x_old_pressure, p2_y_old_pressure;
float p3_x_old_pressure, p3_y_old_pressure;


// Function declarations
void setRTCTime(int year, int month, int day, int hour, int minute, int second);
void setDisplayRotations(uint8_t rotationPressure, uint8_t rotationClock, uint8_t rotationTemperature);
bool findStepperHome();
void checkSerialCommands();
bool readTimeFromSerial();
void stopStepperIfLimitSwitchTriggered();
void calibratePressureSensor();
void calibrateStepperPosition();
void updateStepperPosition(float pressure);
void printCenteredText(Adafruit_GC9A01A &tft, const char* text, int y, uint16_t color, int textSize);
void handleSensorInitError(const char* sensorName);
void handleRTCInitError();
int errorCounter = 0;  // Counter for tracking initialization errors
const int maxAttempts = 3;  // Maximum number of attempts before reset
void setupLEDs();
CRGB getPressureColor(float pressu);
void createLightningEffect();
void updateWeatherLEDs();
void create_dial_temperature();
void draw_pivot_temperature();
void draw_pivot_humidity();
void needle_temperature();
void needle_humidity();
void draw_pivot_pressure();
void draw_needle_pressure();
void draw_scale_pressure();

// LED definitions and variables
#define NUM_LEDS 10
#define LED_DATA_PIN 15  // Choose an appropriate pin
CRGB leds[NUM_LEDS];
unsigned long ledLastCheckTime = 0;
unsigned long ledDisplayStartTime = 0;
bool ledDisplayActive = false;
const unsigned long LED_CHECK_INTERVAL = 5 * 60 * 1000;  // 15 minutes
const unsigned long LED_DISPLAY_DURATION = 120 * 1000;     // 2 minutes
const unsigned long LIGHTNING_CHECK_INTERVAL = 50;       // Check for lightning every 500ms
unsigned long lastLightningCheck = 0;

// Function to check for serial commands
void checkSerialCommands() {
    if (Serial.available()) {
        String command = Serial.readStringUntil('\n');
        command.trim();
        
        if (command == "settime") {
            readTimeFromSerial();
        } else if (command == "help") {
            Serial.println("Verfügbare Befehle:");
            Serial.println("settime - Datum und Zeit einstellen");
            Serial.println("rawpressure - Zeige Rohdaten des Drucksensors");
            Serial.println("calibrate - Drucksensor kalibrieren");
            Serial.println("calibratestepper - Stepper-Position kalibrieren");
            Serial.println("resetcalibration - Alle Kalibrierungen zurücksetzen");
            Serial.println("help - Diese Hilfe anzeigen");
        } else if (command == "rawpressure") {
            showRawPressure = !showRawPressure;
            Serial.print("Rohdaten des Drucksensors: ");
            Serial.println(showRawPressure ? "EIN" : "AUS");
        } else if (command == "calibrate") {
            calibratePressureSensor();
        } else if (command == "calibratestepper") {
            calibrateStepperPosition();
        } else if (command == "resetcalibration") {
            pressureCalibrationOffset = 0.0;
            rainPressure = 990.0;
            fairPressure = 1020.0;
            EEPROM.write(EEPROM_CALIBRATION_MARKER, 0); // Clear marker
            EEPROM.write(EEPROM_STEPPER_CALIBRATION_MARKER, 0); // Clear marker
            Serial.println("Alle Kalibrierungen zurückgesetzt.");
            
            // Update previous pressure value and stepper position
            float rawPressure = bmp180.getPressure();
            pressu = rawPressure;
            previousPressure = pressu;
            updateStepperPosition(pressu);
        }
    }
}

// Function to read time from serial input
bool readTimeFromSerial() {
    Serial.println("Bitte gib Datum und Zeit im Format YYYY,MM,DD,HH,MM,SS ein");
    Serial.println("Beispiel: 2024,5,15,14,30,0 für 15. Mai 2024, 14:30:00");
    
    unsigned long startTime = millis();
    while (!Serial.available()) {
        // Timeout after 30 seconds
        if (millis() - startTime > 30000) {
            Serial.println("Timeout: Keine Eingabe erhalten.");
            return false;
        }
        delay(100);
    }
    
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Check format
    int commaCount = 0;
    for (unsigned int i = 0; i < input.length(); i++) {
        if (input.charAt(i) == ',') commaCount++;
    }
    
    if (commaCount != 5) {
        Serial.println("Ungültiges Format. Bitte verwende YYYY,MM,DD,HH,MM,SS");
        return false;
    }
    
    // Parse input
    int year = input.substring(0, input.indexOf(',')).toInt();
    input = input.substring(input.indexOf(',') + 1);
    int month = input.substring(0, input.indexOf(',')).toInt();
    input = input.substring(input.indexOf(',') + 1);
    int day = input.substring(0, input.indexOf(',')).toInt();
    input = input.substring(input.indexOf(',') + 1);
    int hour = input.substring(0, input.indexOf(',')).toInt();
    input = input.substring(input.indexOf(',') + 1);
    int minute = input.substring(0, input.indexOf(',')).toInt();
    input = input.substring(input.indexOf(',') + 1);
    int second = input.toInt();
    
    // Validate values
    if (year < 2000 || year > 2099 || month < 1 || month > 12 || day < 1 || day > 31 ||
        hour < 0 || hour > 23 || minute < 0 || minute > 59 || second < 0 || second > 59) {
        Serial.println("Ungültige Werte. Bitte überprüfe deine Eingabe.");
        return false;
    }
    
    // Set time
    setRTCTime(year, month, day, hour, minute, second);
    
    Serial.print("Zeit gesetzt auf: ");
    Serial.print(day);
    Serial.print(".");
    Serial.print(month);
    Serial.print(".");
    Serial.print(year);
    Serial.print(" ");
    if (hour < 10) Serial.print("0");
    Serial.print(hour);
    Serial.print(":");
    if (minute < 10) Serial.print("0");
    Serial.print(minute);
    Serial.print(":");
    if (second < 10) Serial.print("0");
    Serial.println(second);
    
    return true;
}

// Function to set RTC time
void setRTCTime(int year, int month, int day, int hour, int minute, int second) {
    rtc.adjust(DateTime(year, month, day, hour, minute, second));
    Serial.println("RTC time set.");
}
// Dummy-Klasse definieren, falls nicht vorhanden
class WeatherLEDStrip {
    public:
      enum WeatherType { SUNNY, RAIN, THUNDERSTORM };
      int getWeatherTypeFromPressure(float pressure, float rainThreshold, float fairThreshold) {
          if (pressure < rainThreshold) return THUNDERSTORM;
          else if (pressure < fairThreshold) return RAIN;
          else return SUNNY;
      }
  };
  WeatherLEDStrip weatherLEDs;
  
// Setup function
void setup() {
    // Initialize serial communication
    Serial.begin(9600);
    Serial.println("GC9A01A screens init!");
    
    // Initialize I2C
    Wire.begin();
    Serial.println("I2C initialized");
    
    // Initialize displays
    tft_pressure.begin();
    tft_clock.begin();
    tft_temperature.begin();
    yield();
    
    setDisplayRotations(1, 2, 3);
    
    // Optional: Splash-Text beim Start
    printCenteredText(tft_clock, "Wetterstation startet...", 120, GC9A01A_BLACK, 2);
    delay(1500);

// Bildschirme initial leeren (weiß)
    tft_temperature.fillScreen(GC9A01A_WHITE);
    tft_pressure.fillScreen(GC9A01A_WHITE);
    tft_clock.fillScreen(GC9A01A_WHITE);

    
    dht.begin();
    
    bmp180.init();
    Serial.println("BMP180 init");
    if (!bmp180.hasValidID()) {
        handleSensorInitError("BMP180");
    }
    
    if (!rtc.begin()) {
        handleRTCInitError();
    }
    
    // Ask for time setting at startup
    Serial.println("Möchtest du die Zeit einstellen? (y/n)");
    unsigned long startTime = millis();
    while (!Serial.available()) {
        if (millis() - startTime > 10000) {  // 10 second timeout
            Serial.println("Keine Antwort. Verwende gespeicherte RTC-Zeit.");
            break;
        }
        delay(100);
    }
    
    if (Serial.available()) {
        char response = Serial.read();
        if (response == 'y' || response == 'Y') {
            while (Serial.available()) Serial.read();  // Clear buffer
            readTimeFromSerial();
        } else {
            Serial.println("Verwende gespeicherte RTC-Zeit.");
        }
    }
    
    // Initialize stepper motor parameters
    setMaxAngle(65.0);
    setHomeOffset(0.0);
    
    // Load pressure sensor calibration data
    if (EEPROM.read(EEPROM_CALIBRATION_MARKER) == 0xAA) {
        EEPROM.get(EEPROM_CALIBRATION_OFFSET, pressureCalibrationOffset);
        Serial.print("Drucksensor-Kalibrierungsdaten geladen. Offset: ");
        Serial.print(pressureCalibrationOffset, 2);
        Serial.println(" hPa");
    } else {
        pressureCalibrationOffset = 0.0;
        Serial.println("Keine Drucksensor-Kalibrierungsdaten gefunden. Verwende Standardwerte.");
    }
    
    // Load stepper calibration data
    if (EEPROM.read(EEPROM_STEPPER_CALIBRATION_MARKER) == 0xBB) {
        EEPROM.get(EEPROM_RAIN_PRESSURE, rainPressure);
        EEPROM.get(EEPROM_FAIR_PRESSURE, fairPressure);
        Serial.print("Stepper-Kalibrierungsdaten geladen. 'Regen' bei: ");
        Serial.print(rainPressure, 2);
        Serial.print(" hPa, 'Schönes Wetter' bei: ");
        Serial.print(fairPressure, 2);
        Serial.println(" hPa");
    } else {
        rainPressure = 990.0; // Default value for rain
        fairPressure = 1020.0; // Default value for fair weather
        Serial.println("Keine Stepper-Kalibrierungsdaten gefunden. Verwende Standardwerte.");
    }
    
    // Initialize previous pressure value with calibrated value
    float rawPressure = bmp180.getPressure();
    pressu = rawPressure + pressureCalibrationOffset;
    previousPressure = pressu;
    
    // Initialize stepper
    initStepper();
    if (findStepperHome()) {
        // Set stepper to current pressure position
        updateStepperPosition(pressu);
    }
    
    // Add this at the end of the setup() function
    setupLEDs();
}

void loop() {
    unsigned long currentMillis = millis();
    
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
        
        // Read sensors
        float rawPressure = bmp180.getPressure();
        humi = dht.readHumidity();
        temp = dht.readTemperature();
        
                // Apply calibration offset to get the calibrated pressure
                float calibratedPressure = rawPressure + pressureCalibrationOffset;
                // Store the calibrated pressure for use in display and stepper
                pressu = calibratedPressure;
                
                // Get current time from RTC
                DateTime now = rtc.now();
                time_h = now.hour();
                time_m = now.minute();
                
                // Neue Temperatur- und Feuchteanzeige
    if (isnan(humi) || isnan(temp)) {
        Serial.println("DHT22-Fehler: Keine gültigen Daten.");
        } else {
            
            drawComfortMeter(temp, humi);
        }

              // Neue analoge Uhr mit Datum oben
            drawAnalogClock(now);

              // Wetter-Icon basierend auf aktuellem Druck
        weatherLEDs.getWeatherTypeFromPressure(pressu, rainPressure, fairPressure);
            updateBarometerDisplay(pressu);


                
                // Show raw pressure if enabled
                if (showRawPressure) {
                    Serial.print("Rohdruck: ");
                    Serial.print(rawPressure, 2);
                    Serial.print(" hPa, Kalibriert: ");
                    Serial.print(pressu, 2);
                    Serial.print(" hPa, Offset: ");
                    Serial.print(pressureCalibrationOffset, 2);
                    Serial.println(" hPa");
                }
                
                // Update stepper position based on calibrated pressure
                float pressureChange = pressu - previousPressure;
                if (abs(pressureChange) >= 0.5) {  // Only move if pressure changed by at least 0.5 hPa
                    updateStepperPosition(pressu);
                    previousPressure = pressu;
                }
            }
            
            // Check for serial commands
            checkSerialCommands();
            
            // Check if the limit switch is triggeGC9A01A_RED
            stopStepperIfLimitSwitchTriggered();
            updateWeatherLEDs();
        }
        
        // Function to set display rotations
        void setDisplayRotations(uint8_t rotationPressure, uint8_t rotationClock, uint8_t rotationTemperature) {
            tft_pressure.setRotation(rotationPressure);
            tft_clock.setRotation(rotationClock);
            tft_temperature.setRotation(rotationTemperature);
            tft_temperature.fillScreen(GC9A01A_WHITE);

// Initiale Nadeldaten setzen
pivot_x_temperature = center_x_temperature;
pivot_y_temperature = center_y_temperature;
p1_x_old_temperature = center_x_temperature; p1_y_old_temperature = center_y_temperature;
p2_x_old_temperature = center_x_temperature; p2_y_old_temperature = center_y_temperature;
p3_x_old_temperature = center_x_temperature; p3_y_old_temperature = center_y_temperature;

pivot_x_humidity = center_x_humidity;
pivot_y_humidity = center_y_humidity;
p4_x_old_humidity = center_x_humidity; p4_y_old_humidity = center_y_humidity;
p5_x_old_humidity = center_x_humidity; p5_y_old_humidity = center_y_humidity;
p6_x_old_humidity = center_x_humidity; p6_y_old_humidity = center_y_humidity;

// Anzeige erzeugen
create_dial_temperature();
draw_pivot_temperature();
draw_pivot_humidity();

        }
        
        // Function to calibrate pressure sensor
        void calibratePressureSensor() {
            float rawPressure = bmp180.getPressure();
            
            Serial.println("Aktuelle Rohdaten des Drucksensors: " + String(rawPressure, 2) + " hPa");
            Serial.println("Bitte gib den tatsächlichen Luftdruck in hPa ein (z.B. 1013.25):");
            
            unsigned long startTime = millis();
            while (!Serial.available()) {
                // Timeout after 30 seconds
                if (millis() - startTime > 30000) {
                    Serial.println("Timeout: Keine Eingabe erhalten.");
                    return;
                }
                delay(100);
            }
            
            String input = Serial.readStringUntil('\n');
            input.trim();
            
            float actualPressure = input.toFloat();
            if (actualPressure < 800 || actualPressure > 1200) {
                Serial.println("Ungültiger Wert. Der Luftdruck sollte zwischen 800 und 1200 hPa liegen.");
                return;
            }
            
            pressureCalibrationOffset = actualPressure - rawPressure;
            
            // Save calibration value to EEPROM
            EEPROM.put(EEPROM_CALIBRATION_OFFSET, pressureCalibrationOffset);
            EEPROM.write(EEPROM_CALIBRATION_MARKER, 0xAA); // Set marker that calibration data exists
            
            Serial.print("Kalibrierung erfolgreich. Offset: ");
            Serial.print(pressureCalibrationOffset, 2);
            Serial.println(" hPa");
            Serial.println("Kalibrierungswert wurde im EEPROM gespeichert.");
            
            // Update previous pressure value so stepper responds correctly
            previousPressure = rawPressure + pressureCalibrationOffset;
        }
        
        // Function to calibrate stepper position
        void calibrateStepperPosition() {
            Serial.println("Stepper-Positionskalibrierung");
            Serial.println("Bitte gib den Druckwert für 'Regen' (niedrigster Druck) ein:");
            
            unsigned long startTime = millis();
            while (!Serial.available()) {
                if (millis() - startTime > 30000) {
                    Serial.println("Timeout: Keine Eingabe erhalten.");
                    return;
                }
                delay(100);
            }
            
            String input = Serial.readStringUntil('\n');
            input.trim();
            
            float newRainPressure = input.toFloat();
            if (newRainPressure < 800 || newRainPressure > 1200) {
                Serial.println("Ungültiger Wert. Der Druckwert sollte zwischen 800 und 1200 hPa liegen.");
                return;
            }
            
            Serial.println("Bitte gib den Druckwert für 'Schönes Wetter' (höchster Druck) ein:");
            
            startTime = millis();
            while (!Serial.available()) {
                if (millis() - startTime > 30000) {
                    Serial.println("Timeout: Keine Eingabe erhalten.");
                    return;
                }
                delay(100);
            }
            
            input = Serial.readStringUntil('\n');
            input.trim();
            
            float newFairPressure = input.toFloat();
            if (newFairPressure < 800 || newFairPressure > 1200 || newFairPressure <= newRainPressure) {
                Serial.println("Ungültiger Wert. Der Druckwert sollte zwischen 800 und 1200 hPa liegen und größer als der Regenwert sein.");
                return;
            }
            
            // Save pressure values to EEPROM
            rainPressure = newRainPressure;
            fairPressure = newFairPressure;
            EEPROM.put(EEPROM_RAIN_PRESSURE, rainPressure);
            EEPROM.put(EEPROM_FAIR_PRESSURE, fairPressure);
            EEPROM.write(EEPROM_STEPPER_CALIBRATION_MARKER, 0xBB); // Set marker
            
            Serial.print("Stepper-Kalibrierung erfolgreich. 'Regen' bei: ");
            Serial.print(rainPressure, 2);
            Serial.print(" hPa, 'Schönes Wetter' bei: ");
            Serial.print(fairPressure, 2);
            Serial.println(" hPa");
            
            // Update stepper position immediately
            updateStepperPosition(pressu);
        }
        // --- COMFORTMETER-FUNKTIONEN ---

void create_dial_temperature() {
    for (float temperature = -20; temperature <= 40; temperature += 10) {
      float angle = (temperature * DEG_TO_RAD) - 2.7;
      float x = pivot_x_temperature + ((radius_temperature + 4) * cos(angle));
      float y = pivot_y_temperature + ((radius_temperature + 4) * sin(angle));
      tft_temperature.fillCircle(x, y, 2, GC9A01A_RED);
    }
  
    tft_temperature.setTextColor(GC9A01A_RED);
    tft_temperature.setTextSize(2);
    tft_temperature.setCursor(64, 16);
    tft_temperature.print("TEMP");
    tft_temperature.setCursor(80, 38); tft_temperature.print("40");
    tft_temperature.setCursor(62, 51); tft_temperature.print("30");
    tft_temperature.setCursor(47, 66); tft_temperature.print("20");
    tft_temperature.setCursor(36, 82); tft_temperature.print("10");
    tft_temperature.setCursor(38, 98); tft_temperature.print("0");
    tft_temperature.setCursor(8, 116); tft_temperature.print("-10");
    tft_temperature.setCursor(5, 134); tft_temperature.print("-20");
    tft_temperature.setCursor(29, 159); tft_temperature.print("C");
    tft_temperature.fillCircle(24, 161, 2, GC9A01A_RED);
    tft_temperature.fillCircle(24, 161, 1, GC9A01A_WHITE);
  
    for (float humidity = 0; humidity <= 100; humidity += 20) {
      float angle = (humidity * DEG_TO_RAD * 0.6) - 1.15;
      float x = pivot_x_humidity + ((radius_temperature + 4) * cos(angle));
      float y = pivot_y_humidity + ((radius_temperature + 4) * sin(angle));
      tft_temperature.fillCircle(x, y, 2, GC9A01A_BLUE);
    }
  
    tft_temperature.setTextColor(GC9A01A_BLUE);
    tft_temperature.setCursor(132, 16); tft_temperature.print("HUMI");
    tft_temperature.setCursor(137, 38); tft_temperature.print("0");
    tft_temperature.setCursor(157, 51); tft_temperature.print("20");
    tft_temperature.setCursor(173, 70); tft_temperature.print("40");
    tft_temperature.setCursor(187, 90); tft_temperature.print("60");
    tft_temperature.setCursor(195, 111); tft_temperature.print("80");
    tft_temperature.setCursor(198, 132); tft_temperature.print("100");
    tft_temperature.setCursor(200, 158); tft_temperature.print("%");
    tft_temperature.setTextColor(GC9A01A_BLACK);
    tft_temperature.setCursor(49, 195); tft_temperature.print("CONFORTMETER");
  }
  
  void draw_pivot_temperature() {
    tft_temperature.fillCircle(pivot_x_temperature, pivot_y_temperature, 8, GC9A01A_RED);
  }
  
  void draw_pivot_humidity() {
    tft_temperature.fillCircle(pivot_x_humidity, pivot_y_humidity, 8, GC9A01A_BLUE);
  }
  
  void needle_temperature() {
    tft_temperature.drawLine(pivot_x_temperature, pivot_y_temperature, p1_x_old_temperature, p1_y_old_temperature, GC9A01A_WHITE);
    tft_temperature.fillTriangle(p1_x_old_temperature, p1_y_old_temperature, p2_x_old_temperature, p2_y_old_temperature, p3_x_old_temperature, p3_y_old_temperature, GC9A01A_WHITE);
    float angle = (needle_setter_temperature * DEG_TO_RAD) - 2.7;
    p1_x_temperature = pivot_x_temperature + ((radius_temperature - 1) * cos(angle));
    p1_y_temperature = pivot_y_temperature + ((radius_temperature - 1) * sin(angle));
    p2_x_temperature = pivot_x_temperature + ((radius_temperature - 16) * cos(angle - 0.05));
    p2_y_temperature = pivot_y_temperature + ((radius_temperature - 16) * sin(angle - 0.05));
    p3_x_temperature = pivot_x_temperature + ((radius_temperature - 16) * cos(angle + 0.05));
    p3_y_temperature = pivot_y_temperature + ((radius_temperature - 16) * sin(angle + 0.05));
    p1_x_old_temperature = p1_x_temperature; p1_y_old_temperature = p1_y_temperature;
    p2_x_old_temperature = p2_x_temperature; p2_y_old_temperature = p2_y_temperature;
    p3_x_old_temperature = p3_x_temperature; p3_y_old_temperature = p3_y_temperature;
    tft_temperature.drawLine(pivot_x_temperature, pivot_y_temperature, p1_x_temperature, p1_y_temperature, GC9A01A_RED);
    tft_temperature.fillTriangle(p1_x_temperature, p1_y_temperature, p2_x_temperature, p2_y_temperature, p3_x_temperature, p3_y_temperature, GC9A01A_RED);
  }
  
  void needle_humidity() {
    tft_temperature.drawLine(pivot_x_humidity, pivot_y_humidity, p4_x_old_humidity, p4_y_old_humidity, GC9A01A_WHITE);
    tft_temperature.fillTriangle(p4_x_old_humidity, p4_y_old_humidity, p5_x_old_humidity, p5_y_old_humidity, p6_x_old_humidity, p6_y_old_humidity, GC9A01A_WHITE);
    float angle = (needle_setter_humidity * DEG_TO_RAD * 0.6) - 1.15;
    p4_x_humidity = pivot_x_humidity + ((radius_temperature - 1) * cos(angle));
    p4_y_humidity = pivot_y_humidity + ((radius_temperature - 1) * sin(angle));
    p5_x_humidity = pivot_x_humidity + ((radius_temperature - 16) * cos(angle - 0.05));
    p5_y_humidity = pivot_y_humidity + ((radius_temperature - 16) * sin(angle - 0.05));
    p6_x_humidity = pivot_x_humidity + ((radius_temperature - 16) * cos(angle + 0.05));
    p6_y_humidity = pivot_y_humidity + ((radius_temperature - 16) * sin(angle + 0.05));
    p4_x_old_humidity = p4_x_humidity; p4_y_old_humidity = p4_y_humidity;
    p5_x_old_humidity = p5_x_humidity; p5_y_old_humidity = p5_y_humidity;
    p6_x_old_humidity = p6_x_humidity; p6_y_old_humidity = p6_y_humidity;
    tft_temperature.drawLine(pivot_x_humidity, pivot_y_humidity, p4_x_humidity, p4_y_humidity, GC9A01A_BLUE);
    tft_temperature.fillTriangle(p4_x_humidity, p4_y_humidity, p5_x_humidity, p5_y_humidity, p6_x_humidity, p6_y_humidity, GC9A01A_BLUE);
  }
  
  // --- ENDE COMFORTMETER-FUNKTIONEN ---
  
        // Function to update stepper position based on pressure
        void updateStepperPosition(float pressure) {
            // Since home position (0) is "maximum rain" and maxSteps is "very nice weather",
            // we need to map pressure values accordingly
            
            // Calculate pressure range
            float pressureRange = fairPressure - rainPressure;
            
            // Calculate relative position in range (0.0 to 1.0)
            // 0.0 = Rain (lowest pressure), 1.0 = Fair weather (highest pressure)
            float relativePosition = (pressure - rainPressure) / pressureRange;
            
            // Constrain position to valid range
            relativePosition = constrain(relativePosition, 0.0, 1.0);
            
            // Calculate steps - since 0 = Rain and maxSteps = Fair weather
            int targetSteps = round(relativePosition * maxSteps);
            
            // Move stepper to calculated position
            stepper.moveTo(targetSteps);
            stepper.runToPosition();
            
            if (showRawPressure) {
                Serial.print("Druck: ");
                Serial.print(pressure);
                Serial.print(" hPa, Stepper-Position: ");
                Serial.print(targetSteps);
                Serial.print("/");
                Serial.println(maxSteps);
            }
        }
        
        // Function to center text on display
        void printCenteredText(Adafruit_GC9A01A &tft, const char* text, int y, uint16_t color, int textSize) {
            tft.setTextSize(textSize);
            tft.setTextColor(color);
            
            // Calculate the width of the text
            int16_t x1, y1;
            uint16_t w, h;
            tft.getTextBounds(text, 0, 0, &x1, &y1, &w, &h);
            
            // Calculate the position to center the text
            // GC9A01A is 240x240 pixels
            int x = (240 - w) / 2;
            
            // Set cursor and print text
            tft.setCursor(x, y);
            tft.println(text);
        }
        
void setupLEDs() {
    FastLED.addLeds<WS2812B, LED_DATA_PIN, GRB>(leds, NUM_LEDS);
    FastLED.setBrightness(75);  // 75% max brightness = 100% × 0.75
    FastLED.clear();
    FastLED.show();
  }
  
  // Function to get color based on pressure value
  CRGB getPressureColor(float pressure) {
    // Map pressure to the ranges defined for the weather station
    float pressureRange = fairPressure - rainPressure;
    float relativePosition = (pressure - rainPressure) / pressureRange;
    relativePosition = constrain(relativePosition, 0.0, 1.0);
    
    // Define colors for different weather conditions
    if (relativePosition < 0.2) {
      // Stormy - purple
      return CRGB(100, 0, 100);
    } else if (relativePosition < 0.4) {
      // Rainy - blue
      return CRGB(0, 0, 100);
    } else if (relativePosition < 0.6) {
      // Changing - yellow
      return CRGB(100, 100, 0);
    } else if (relativePosition < 0.8) {
      // Fair - light blue
      return CRGB(0, 100, 100);
    } else {
      // Sunny - orange
      return CRGB(255, 165, 0);
    }
  }
  
  // Function to create lightning effect
  void createLightningEffect() {
    // Only create lightning in stormy conditions (low pressure)
    float pressureRange = fairPressure - rainPressure;
    float relativePosition = (pressu - rainPressure) / pressureRange;
    
    if (relativePosition < 0.2) {  // Stormy conditions
      if (random(100) < 20) {  // 20% chance for lightning
        uint8_t ledCount = random(1, 4);  // Flash 1-3 LEDs
        for (uint8_t i = 0; i < ledCount; i++) {
          uint8_t ledIndex = random(NUM_LEDS);
          CRGB originalColor = leds[ledIndex];
          
          // Flash white at 25% brightness
          leds[ledIndex] = CRGB(64, 64, 64);  // 25% of full white (255)
          FastLED.show();
          delay(50 + random(100));  // Lightning duration
          
          // Return to original color
          leds[ledIndex] = originalColor;
          FastLED.show();
        }
      }
    }
  }
  
// Function to update LEDs based on weather conditions
void updateWeatherLEDs() {
    unsigned long currentMillis = millis();
    
    // Check if it's time to start LED display
    if (!ledDisplayActive && (currentMillis - ledLastCheckTime >= LED_CHECK_INTERVAL)) {
      ledDisplayActive = true;
      ledDisplayStartTime = currentMillis;
      ledLastCheckTime = currentMillis;
      
      // Set LED colors based on current pressure
      for (int i = 0; i < NUM_LEDS; i++) {
        leds[i] = getPressureColor(pressu);
      }
      
      // GC9A01A_REDuce brightness to 10% for regular display
      FastLED.setBrightness(25);  // 10% of max brightness (10% of 255 ≈ 25)
      FastLED.show();
    }
    
    // Check if LED display period is active
    if (ledDisplayActive) {
      // Check if it's time to create lightning effect
      if (currentMillis - lastLightningCheck >= LIGHTNING_CHECK_INTERVAL) {
        lastLightningCheck = currentMillis;
        createLightningEffect();
      }
      
      // Check if display time is over
      if (currentMillis - ledDisplayStartTime >= LED_DISPLAY_DURATION) {
        ledDisplayActive = false;
        // Turn off all LEDs
        FastLED.clear();
        FastLED.show();
      }
    }
}

// Temperatur → Farbe
uint16_t getTemperatureColor(float t) {
    if (t < 0) return GC9A01A_BLUE;
    else if (t < 10) return GC9A01A_CYAN;
    else if (t < 20) return GC9A01A_GREEN;
    else if (t < 30) return GC9A01A_ORANGE;
    else return GC9A01A_RED;
}

void drawAnalogClock(DateTime now) {
    tft_clock.fillScreen(GC9A01A_WHITE);
    int cx = 120, cy = 130, r = 90;

    // Datum oben
    char dateStr[20];
    sprintf(dateStr, "%02d.%02d.%04d", now.day(), now.month(), now.year());
    printCenteredText(tft_clock, dateStr, 20, GC9A01A_BLACK, 2);

    tft_clock.drawCircle(cx, cy, r, GC9A01A_BLACK);
    tft_clock.drawCircle(cx, cy, r + 1, GC9A01A_BLACK);

    // Zahlen für Stunden
    for (int i = 1; i <= 12; i++) {
        float angle = (i * 30 - 90) * DEG_TO_RAD;
        int x = cx + cos(angle) * (r - 20);
        int y = cy + sin(angle) * (r - 20);
        char buf[3];
        sprintf(buf, "%d", i);
        tft_clock.setCursor(x - 5, y - 5);
        tft_clock.setTextColor(GC9A01A_BLACK);
        tft_clock.setTextSize(1);
        tft_clock.print(buf);
    }

    // Zeiger zeichnen
    float hourAngle = ((now.hour() % 12) + now.minute() / 60.0) * 30 - 90;  // -90 zum Ausrichten auf 12 Uhr oben
    float minAngle  = now.minute() * 6 - 90;
    float secAngle  = now.second() * 6 - 90;

    float radHour = hourAngle * DEG_TO_RAD;
    float radMin  = minAngle * DEG_TO_RAD;
    float radSec  = secAngle * DEG_TO_RAD;

    int hx = cx + cos(radHour) * (r - 40);
    int hy = cy + sin(radHour) * (r - 40);
    int mx = cx + cos(radMin) * (r - 25);
    int my = cy + sin(radMin) * (r - 25);
    int sx = cx + cos(radSec) * (r - 15);
    int sy = cy + sin(radSec) * (r - 15);

    tft_clock.drawLine(cx, cy, hx, hy, GC9A01A_BLACK);
    tft_clock.drawLine(cx, cy, mx, my, GC9A01A_DARKGREY);
    tft_clock.drawLine(cx, cy, sx, sy, GC9A01A_RED);  // Sekundenzeiger
    tft_clock.fillCircle(cx, cy, 3, GC9A01A_BLACK);
}


    void drawComfortMeter(float temp, float humi) {
        // Werte vorbereiten
        needle_setter_temperature = temp;
        needle_setter_humidity = humi;
      
        // Nur beim ersten Aufruf: Hintergrund zeichnen
        static bool initialized = false;
        if (!initialized) {
          tft_temperature.fillScreen(GC9A01A_WHITE);
          create_dial_temperature();        // Skala und Beschriftung
          draw_pivot_temperature();         // roter Drehpunkt
          draw_pivot_humidity();            // blauer Drehpunkt
          initialized = true;
        }
      
        // Zeiger aktualisieren
        needle_temperature();               // roter Zeiger
        needle_humidity();                  // blauer Zeiger
      }
      
// --- BAROMETER-FUNKTIONEN (NEU) ---

void create_dial_pressure() {
    for (int pressure_scale = 900; pressure_scale <= 1050; pressure_scale += 2) {
      float angle = map(pressure_scale, 900, 1050, 45, -225) * DEG_TO_RAD;
      float x = pivot_x_pressure + ((radius_pressure + 4) * cos(angle));
      float y = pivot_y_pressure + ((radius_pressure + 4) * sin(angle));
      tft_pressure.fillCircle(x, y, 1, GC9A01A_BLACK);
    }
    for (int pressure_scale = 900; pressure_scale <= 1050; pressure_scale += 15) {
      float angle = map(pressure_scale, 900, 1050, 45, -225) * DEG_TO_RAD;
      float x = pivot_x_pressure + ((radius_pressure + 4) * cos(angle));
      float y = pivot_y_pressure + ((radius_pressure + 4) * sin(angle));
      tft_pressure.fillCircle(x, y, 2, GC9A01A_BLUE);
    }
    tft_pressure.setTextColor(GC9A01A_BLACK);
    tft_pressure.setTextSize(2);
    tft_pressure.setCursor(40, 178); tft_pressure.print("900");
    tft_pressure.setCursor(18, 149); tft_pressure.print("915");
    tft_pressure.setCursor(9, 113);  tft_pressure.print("930");
    tft_pressure.setCursor(18, 76);  tft_pressure.print("945");
    tft_pressure.setCursor(34, 47);  tft_pressure.print("960");
    tft_pressure.setCursor(96, 22);  tft_pressure.print("975");
    tft_pressure.setCursor(158, 47); tft_pressure.print("990");
    tft_pressure.setCursor(184, 76); tft_pressure.print("1005");
    tft_pressure.setCursor(192, 113);tft_pressure.print("1020");
    tft_pressure.setCursor(184, 149);tft_pressure.print("1035");
    tft_pressure.setCursor(158, 178);tft_pressure.print("1050");
    tft_pressure.setTextColor(GC9A01A_RED);
    tft_pressure.setCursor(67, 205); tft_pressure.print("BAROMETER");
    tft_pressure.setCursor(103, 223);tft_pressure.print("hPa");
  }
  
  void draw_pivot_pressure() {
    tft_pressure.fillCircle(pivot_x_pressure, pivot_y_pressure, 8, GC9A01A_RED);
    tft_pressure.drawCircle(pivot_x_pressure, pivot_y_pressure, 8, GC9A01A_BLACK);
    tft_pressure.drawCircle(pivot_x_pressure, pivot_y_pressure, 3, GC9A01A_BLACK);
  }
  
  void needle_pressure() {
    tft_pressure.drawLine(pivot_x_pressure, pivot_y_pressure, p1_x_old_pressure, p1_y_old_pressure, GC9A01A_WHITE);
    tft_pressure.fillTriangle(p1_x_old_pressure, p1_y_old_pressure, p2_x_old_pressure, p2_y_old_pressure, p3_x_old_pressure, p3_y_old_pressure, GC9A01A_WHITE);
  
    float angle = map(needle_setter_pressure, 900, 1050, 225, -45) * DEG_TO_RAD;
    p1_x_pressure = pivot_x_pressure + ((radius_pressure - 1) * cos(angle));
    p1_y_pressure = pivot_y_pressure + ((radius_pressure - 1) * sin(angle));
    p2_x_pressure = pivot_x_pressure + ((radius_pressure - 16) * cos(angle - 0.05));
    p2_y_pressure = pivot_y_pressure + ((radius_pressure - 16) * sin(angle - 0.05));
    p3_x_pressure = pivot_x_pressure + ((radius_pressure - 16) * cos(angle + 0.05));
    p3_y_pressure = pivot_y_pressure + ((radius_pressure - 16) * sin(angle + 0.05));
  
    p1_x_old_pressure = p1_x_pressure; p1_y_old_pressure = p1_y_pressure;
    p2_x_old_pressure = p2_x_pressure; p2_y_old_pressure = p2_y_pressure;
    p3_x_old_pressure = p3_x_pressure; p3_y_old_pressure = p3_y_pressure;
  
    tft_pressure.drawLine(pivot_x_pressure, pivot_y_pressure, p1_x_pressure, p1_y_pressure, GC9A01A_BLACK);
    tft_pressure.fillTriangle(p1_x_pressure, p1_y_pressure, p2_x_pressure, p2_y_pressure, p3_x_pressure, p3_y_pressure, GC9A01A_BLACK);
  }
  
  void updateBarometerDisplay(float pressure) {
    float calibratedPressure = pressure;
    needle_setter_pressure = constrain(calibratedPressure, 900.0, 1050.0);
  
    static bool initialized = false;
    if (!initialized) {
      tft_pressure.fillScreen(GC9A01A_WHITE);
      pivot_x_pressure = center_x_pressure;
      pivot_y_pressure = center_y_pressure;
      create_dial_pressure();
      draw_pivot_pressure();
      p1_x_old_pressure = center_x_pressure; p1_y_old_pressure = center_y_pressure;
      p2_x_old_pressure = center_x_pressure; p2_y_old_pressure = center_y_pressure;
      p3_x_old_pressure = center_x_pressure; p3_y_old_pressure = center_y_pressure;
      initialized = true;
    }
    needle_pressure();
  
   
  }
  
  // --- ENDE BAROMETER-FUNKTIONEN ---
  