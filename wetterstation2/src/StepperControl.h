#ifndef STEPPERCONTROL_H
#define STEPPERCONTROL_H

#include <AccelStepper.h>

// Declare the stepper object as extern
extern AccelStepper stepper;

// Declare global variables
extern float maxPressureChange;
extern int maxSteps;
extern float maxAngle;
extern int homeOffsetSteps;
extern float calibrationOffset;
extern float stepsPerDegree;
extern const float minPressure;
extern const float maxPressure;

// Function declarations
void initStepper();
void setupLimitSwitch();
bool isStepperAtHome();
void stopStepperIfLimitSwitchTriggered();
int pressureToSteps(float pressure);
void moveStepperBasedOnPressure(float currentPressure, float previousPressure);
bool findStepperHome();
void setHomeOffset(float offsetDegrees);
void moveStepperToPosition(int targetPosition);
void setMaxAngle(float angle);

#endif // STEPPERCONTROL_H