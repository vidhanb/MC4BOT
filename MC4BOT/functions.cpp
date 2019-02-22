// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHLCD.h>
#include <FEHIO.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

void initRobot() {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    servoLever.SetMin(755);
    servoLever.SetMax(2495);
    servoLever.SetDegree(SERVO_LEVER_POS_NEUTRAL);
}

void printInit() {
    // Print info for left drive motor
    LCD.WriteLine("Left Motor Init:");
    LCD.Write("-Port: ");
    LCD.WriteLine(MOTOR_PORT_FL);
    LCD.Write("-Volts: ");
    LCD.WriteLine(MOTOR_VOLTS);
    // Print info for right drive motor
    LCD.WriteLine("Right Motor Init:");
    LCD.Write("-Port: ");
    LCD.WriteLine(MOTOR_PORT_FR);
    LCD.Write("-Volts: ");
    LCD.WriteLine(MOTOR_VOLTS);
    // Print info for lever servo motor
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

void testDrive() {
    motorLeft.SetPercent(100);
    motorRight.SetPercent(-100);
    Sleep(3.0);
    motorLeft.SetPercent(-100);
    motorRight.SetPercent(100);
    Sleep(3.0);
    motorLeft.Stop();
    motorRight.Stop();
    return;
}

void sensorsTest() {
    int counts = 0;
    while(counts < 10) {
        LCD.Write("CdS: ");
        LCD.WriteLine(cdsCell.Value());
        LCD.Write("Left encoder: ");
        LCD.WriteLine(leftEncoder.Counts());
        LCD.Write("Right encoder: ");
        LCD.WriteLine(rightEncoder.Counts());
        Sleep(0.3);
        counts++;
    }
}

void leverTest() {
    servoLever.SetDegree(180);
    motorLeft.SetPercent(50);
    motorRight.SetPercent(-50);
    Sleep(2.0);
    motorLeft.SetPercent(0);
    motorRight.SetPercent(0);
    Sleep(2.0);
    servoLever.SetDegree(90);
    Sleep(1.0);
    motorLeft.SetPercent(-50);
    motorRight.SetPercent(50);
    Sleep(3.0);
    motorLeft.Stop();
    motorRight.Stop();
}