#ifndef ERROR_HANDLING_H
#define ERROR_HANDLING_H

#include <Arduino.h>

// Forward declarations for display functions
class Adafruit_GC9A01A;
void printCenteredText(Adafruit_GC9A01A &tft, const char* text, int y, uint16_t color, int textSize);

// External references to display objects
extern Adafruit_GC9A01A tft_pressure;
extern Adafruit_GC9A01A tft_clock;
extern Adafruit_GC9A01A tft_temperature;

// Function to reset the Teensy 4.0
inline void resetController() {
    SCB_AIRCR = 0x05FA0004;  // Write the reset value to the AIRCR register
}

// Global variables
extern int errorCounter;  // Counter for tracking initialization errors
extern const int maxAttempts;  // Maximum number of attempts before reset

// Function to log errors to the Serial Monitor
inline void logError(const char* errorMessage) {
    Serial.println(errorMessage);
}

// Function to handle sensor initialization errors
inline void handleSensorInitError(const char* sensorName) {
    logError("Failed to initialize sensor:");
    logError(sensorName);
    logError("Please check your wiring and try again.");
    
    // Display error on screen
    tft_pressure.fillScreen(0xF800); // RED
    printCenteredText(tft_pressure, "SENSOR ERROR", 120, 0xFFFF, 2); // WHITE text
    delay(2000);
    
    errorCounter++;
    if (errorCounter >= maxAttempts) {
        logError("Maximum attempts reached. Resetting controller...");
        delay(1000);
        resetController();  // Reset the controller
    }
}

// Function to handle RTC initialization errors
inline void handleRTCInitError() {
    logError("Failed to initialize RTC module.");
    logError("Please check your wiring and try again.");
    
    // Display error on screen
    tft_clock.fillScreen(0xF800); // RED
    printCenteredText(tft_clock, "RTC ERROR", 120, 0xFFFF, 2); // WHITE text
    delay(2000);
    
    errorCounter++;
    if (errorCounter >= maxAttempts) {
        logError("Maximum attempts reached. Resetting controller...");
        delay(1000);
        resetController();  // Reset the controller
    }
}

// Function to handle stepper motor errors
inline void handleStepperError() {
    logError("Stepper motor failed to find home position.");
    
    // Display error on screen
    tft_pressure.fillScreen(0xF800); // RED
    printCenteredText(tft_pressure, "STEPPER ERROR", 120, 0xFFFF, 2); // WHITE text
    delay(2000);
    
    errorCounter++;
    if (errorCounter >= maxAttempts) {
        logError("Maximum attempts reached. Resetting controller...");
        delay(1000);
        resetController();  // Reset the controller
    }
}

// Function to handle generic errors
inline void handleError(const char* errorMessage) {
    logError(errorMessage);
    
    // Display error on all screens
    tft_pressure.fillScreen(0xF800); // RED
    tft_clock.fillScreen(0xF800); // RED
    tft_temperature.fillScreen(0xF800); // RED
    
    printCenteredText(tft_pressure, "ERROR", 100, 0xFFFF, 2); // WHITE text
    printCenteredText(tft_pressure, errorMessage, 140, 0xFFFF, 1);
    delay(2000);
    
    errorCounter++;
    if (errorCounter >= maxAttempts) {
        logError("Maximum attempts reached. Resetting controller...");
        delay(1000);
        resetController();  // Reset the controller
    }
}

#endif // ERROR_HANDLING_H
