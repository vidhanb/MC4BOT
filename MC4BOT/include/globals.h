#ifndef GLOBALS_H
#define GLOBALS_H

// Note: the "extern" keyword makes the definitions in globals.cpp available to any file which includes this header

// Motor definitions
extern FEHMotor motorLeft;
extern FEHMotor motorRight;

// Servo definitions
extern FEHServo servoLever;
extern FEHServo servoCoin;
extern FEHServo servoClaw;

// Sensor definitions
extern AnalogInputPin cdsCell;
extern DigitalEncoder encoderLeft;
extern DigitalEncoder encoderRight;

// Error detection and reaction values
extern int encoderErrors;
extern bool encodersEnabled;
extern int rpsErrors;
extern bool rpsEnabled;

#endif // GLOBALS_H