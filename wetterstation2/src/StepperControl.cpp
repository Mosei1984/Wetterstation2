#ifndef STEPPERCONTROL_H
#define STEPPERCONTROL_H

#include <AccelStepper.h>

// Pin definitions
#define IN1 4
#define IN2 5
#define IN3 8
#define IN4 1
#define LIMIT_SWITCH_PIN 6

// Stepper motor configuration
AccelStepper stepper(AccelStepper::FULL4WIRE, IN1, IN3, IN2, IN4);

// Pressure and stepper parameters
float maxPressureChange = 300.0;
int maxSteps = 2048;  // Total steps for a full revolution
float maxAngle = 65.0;
int homeOffsetSteps = 0;
float calibrationOffset = 1172.3 - 850.5;  // Calibration offset (321.8)
float stepsPerDegree = 2048.0 / 360.0;  // Corrected for 2048-step motor

// Pressure range for the barometer
const float minPressure = 900.0;
const float maxPressure = 1050.0;

// Initialize the stepper motor
void initStepper() {
    stepper.setMaxSpeed(1000);
    stepper.setAcceleration(500);
    
    Serial.println("Stepper motor initialized");
    Serial.print("Steps per degree: ");
    Serial.println(stepsPerDegree);
}

// Setup the limit switch
void setupLimitSwitch() {
    pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
}

// Check if the stepper is at the home position
bool isStepperAtHome() {
    return digitalRead(LIMIT_SWITCH_PIN) == LOW;
}

// Stop the stepper if the limit switch is triggered
void stopStepperIfLimitSwitchTriggered() {
    if (isStepperAtHome()) {
        stepper.stop();  // Stop the stepper motor
        Serial.println("Limit switch triggered! Stepper motor stopped.");
    }
}

// Convert pressure to stepper steps
int pressureToSteps(float pressure) {
    // Constrain pressure to the valid range
    if (pressure < minPressure) {
        pressure = minPressure;
    } else if (pressure > maxPressure) {
        pressure = maxPressure;
    }
    
    // Calculate angle based on pressure
    float angle = ((pressure - minPressure) / (maxPressure - minPressure)) * maxAngle;
    
    // Convert angle to steps
    return static_cast<int>((angle / 360.0) * maxSteps);
}

// Move the stepper based on pressure change
void moveStepperBasedOnPressure(float currentPressure, float previousPressure) {
    // Calculate steps for current and previous pressure
    int currentSteps = pressureToSteps(currentPressure);
    int previousSteps = pressureToSteps(previousPressure);
    int stepDifference = currentSteps - previousSteps;
    
    // Debug output
    Serial.print("Current Pressure: ");
    Serial.println(currentPressure);
    Serial.print("Previous Pressure: ");
    Serial.println(previousPressure);
    Serial.print("Step Difference: ");
    Serial.println(stepDifference);
    
    // Move the stepper motor by the step difference
    stepper.move(stepDifference);
    stepper.runToPosition();
}

// Find the home position using the limit switch
bool findStepperHome() {
    setupLimitSwitch();
    stepper.setSpeed(100);
    stepper.setAcceleration(500);
    
    // Move until limit switch is triggered
    while (!isStepperAtHome()) {
        stepper.move(-3);
        stepper.runSpeed();
    }
    
    // Set current position as zero
    stepper.setCurrentPosition(0);
    
    // Move slightly away from the limit switch
    int oneDegreeSteps = static_cast<int>(maxSteps / 360.0);
    stepper.move(oneDegreeSteps);
    stepper.runToPosition();
    stepper.setCurrentPosition(0);
    
    return true;
}

// Set the home offset in degrees
void setHomeOffset(float offsetDegrees) {
    homeOffsetSteps = static_cast<int>((offsetDegrees / 360.0) * maxSteps);
}

// Move stepper to a specific position
void moveStepperToPosition(int targetPosition) {
    stepper.moveTo(targetPosition);
    stepper.runToPosition();
}

// Set the maximum angle for the stepper
void setMaxAngle(float angle) {
    maxAngle = angle;
    
    // Calculate maxSteps based on angle
    maxSteps = static_cast<int>(angle * stepsPerDegree);
    
    Serial.print("Max angle set to: ");
    Serial.print(maxAngle);
    Serial.print(" degrees (");
    Serial.print(maxSteps);
    Serial.println(" steps)");
}

#endif // STEPPERCONTROL_H