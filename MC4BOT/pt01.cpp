// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHLCD.h>

// Required personal libraries
#include "include/constants.h"

using namespace std;

void initRobot() {
    // Init and print info for left drive motor
    FEHMotor motorLeft(MOTOR_PORT_FL, MOTOR_VOLTS);
    LCD.WriteLine("Left Motor Init:");
    LCD.Write("-Port: ");
    LCD.WriteLine(MOTOR_PORT_FL);
    LCD.Write("-Volts: ");
    LCD.WriteLine(MOTOR_VOLTS);
    // Init and print info for right drive motor
    FEHMotor motorRight(MOTOR_PORT_FR, MOTOR_VOLTS);
    LCD.WriteLine("Right Motor Init:");
    LCD.Write("-Port: ");
    LCD.WriteLine(MOTOR_PORT_FR);
    LCD.Write("-Volts: ");
    LCD.WriteLine(MOTOR_VOLTS);
    // Init and print info for lever servo motor
    FEHServo servoLever(SERVO_PORT_LEVER);
    servoLever.SetMin(SERVO_LEVER_MIN);
    servoLever.SetMax(SERVO_LEVER_MAX);
    LCD.WriteLine("Lever Servo Init:");
    LCD.Write("-Port: ");
    LCD.WriteLine(SERVO_PORT_LEVER);
    LCD.Write("-Min: ");
    LCD.WriteLine(SERVO_LEVER_MIN);
    LCD.Write("-Max: ");
    LCD.WriteLine(SERVO_LEVER_MAX);

    LCD.WriteLine("Init complete.");
    return;
}