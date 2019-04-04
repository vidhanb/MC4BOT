// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"

// Motor definitions
FEHMotor g_motorLeft(MOTOR_PORT_FL, MOTOR_VOLTS);
FEHMotor g_motorRight(MOTOR_PORT_FR, MOTOR_VOLTS);

// Servo definitions
FEHServo g_servoLever(SERVO_PORT_LEVER);
FEHServo g_servoCoin(SERVO_PORT_COIN);
FEHServo g_servoClaw(SERVO_PORT_CLAW);

// Sensor definitions
AnalogInputPin g_cdsCell(CDS_CELL_PORT);
DigitalEncoder g_encoderLeft(ENCODER_LEFT_PORT);
DigitalEncoder g_encoderRight(ENCODER_RIGHT_PORT);

// Error detection and reaction values
int g_encoderErrors = 0;
bool g_encodersEnabled = true;
int g_rpsErrors = 0;
bool g_rpsEnabled = true;
