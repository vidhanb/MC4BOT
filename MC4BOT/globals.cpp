// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"

// Motor and servo definitions
FEHMotor motorLeft(MOTOR_PORT_FL, MOTOR_VOLTS);
// Left motor - negative moves forward
FEHMotor motorRight(MOTOR_PORT_FR, MOTOR_VOLTS);
FEHServo servoLever(SERVO_PORT_LEVER);

// Sensor definitions
AnalogInputPin cdsCell(CDS_CELL_PORT);
DigitalEncoder leftEncoder(ENCODER_LEFT_PORT);
DigitalEncoder rightEncoder(ENCODER_RIGHT_PORT);
