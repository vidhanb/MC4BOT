#ifndef GLOBALS_H
#define GLOBALS_H

// Note: the "extern" keyword makes the definitions in globals.cpp available to any file which includes this header

// Motor definitions
extern FEHMotor g_motorLeft;
//// Note that the right motor is backwards
////   This means that its power needs to be negative compared
////   to the left value in order to go straight
extern FEHMotor g_motorRight;

// Servo definitions
extern FEHServo g_servoLever;
extern FEHServo g_servoCoin;
extern FEHServo g_servoClaw;

// Sensor definitions
extern AnalogInputPin g_cdsCell;
extern DigitalEncoder g_encoderLeft;
extern DigitalEncoder g_encoderRight;

// RPS adjustment values
extern float g_adjustX;
extern float g_adjustY;

// Error detection and reaction values
extern int g_encoderErrors;
extern bool g_encodersEnabled;
extern int g_rpsErrors;
extern bool g_rpsEnabled;

#endif // GLOBALS_H