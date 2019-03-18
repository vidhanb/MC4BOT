// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"

// Motor definitions
FEHMotor motorLeft(MOTOR_PORT_FL, MOTOR_VOLTS);
FEHMotor motorRight(MOTOR_PORT_FR, MOTOR_VOLTS);

// Servo definitions
FEHServo servoLever(SERVO_PORT_LEVER);
FEHServo servoCoin(SERVO_PORT_COIN);
FEHServo servoClaw(SERVO_PORT_CLAW);

// Sensor definitions
AnalogInputPin cdsCell(CDS_CELL_PORT);
DigitalEncoder encoderLeft(ENCODER_LEFT_PORT);
DigitalEncoder encoderRight(ENCODER_RIGHT_PORT);

// Error detection and reaction values
int encoderErrors = 0;
bool encodersEnabled = true;
int rpsErrors = 0;
bool rpsEnabled = true;
