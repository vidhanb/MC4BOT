// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHLCD.h>
#include <FEHIO.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

void pt01() {
    initRobot();
    printInit();

    //Detect color change and move robot for 5 seconds
    //if (cdsCell.Value() == START_COLOR_VALUE) {
    //Move robot up slightly
    motorRight.SetPercent(-MOTOR_PERCENT_WEAK);
    motorLeft.SetPercent(MOTOR_PERCENT_WEAK);
    Sleep(1.5);

    //Angle the robot around to face the ramp
    motorRight.SetPercent(MOTOR_PERCENT_WEAK);
    motorLeft.SetPercent(MOTOR_PERCENT_WEAK);
    Sleep(1.5);

    //Stop robot
    motorRight.Stop();
    motorLeft.Stop();

    //Move robot towards the ramp
    motorRight.SetPercent(-MOTOR_PERCENT_MEDIUM);
    motorLeft.SetPercent(MOTOR_PERCENT_MEDIUM);
    Sleep(3.5);

    motorRight.Stop();
    motorLeft.Stop();

    //Angle robot towards ramp
    motorRight.SetPercent(-MOTOR_PERCENT_WEAK);
    motorLeft.SetPercent(-MOTOR_PERCENT_WEAK);
    Sleep(4.2); //Sometimes 4.3 based on position of robot
    //------------------------------------------------------------------------------------------------------------------------------------------------//
    //CODE: Moves robot up the ramp, turns left to face the lever, goes to the lever, flips the lever, and then retraces its path
    //      back to the starting box
    //TODO: Set the Sleep times to make sure the robot is moving the appropriate distance and correct angles


    //Move robot up the ramp
    motorRight.SetPercent(-MOTOR_PERCENT_STRONG);
    motorLeft.SetPercent(MOTOR_PERCENT_STRONG);
    Sleep(5.);

    //Stop robot
    motorRight.Stop();
    motorLeft.Stop();

    //Turn robot to the left to face the lever
    motorRight.SetPercent(-MOTOR_PERCENT_WEAK);
    motorLeft.SetPercent(-MOTOR_PERCENT_WEAK);
    Sleep(4.2);

    //Move robot to the lever
    motorRight.SetPercent(-MOTOR_PERCENT_MEDIUM);
    motorLeft.SetPercent(MOTOR_PERCENT_MEDIUM);
    Sleep(3.);

    //Flip lever
    servoLever.SetDegree(SERVO_LEVER_POS_ACTIVE);
    Sleep(1.);
    servoLever.SetDegree(SERVO_LEVER_POS_NEUTRAL);

    //Move robot back
    motorRight.SetPercent(MOTOR_PERCENT_STRONG);
    motorLeft.SetPercent(-MOTOR_PERCENT_STRONG);
    Sleep(3.);

    //Angle robot towards ramp
    motorRight.SetPercent(MOTOR_PERCENT_WEAK);
    motorLeft.SetPercent(MOTOR_PERCENT_WEAK);
    Sleep(4.2);

    //Move robot down the ramp
    motorRight.SetPercent(MOTOR_PERCENT_STRONG);
    motorLeft.SetPercent(-MOTOR_PERCENT_STRONG);
    Sleep(5.);

    //Angle robot away ramp
    motorRight.SetPercent(MOTOR_PERCENT_WEAK);
    motorLeft.SetPercent(MOTOR_PERCENT_WEAK);
    Sleep(4.2); //Sometimes 4.3 based on position of robot

    //Move robot back towards starting box
    motorRight.SetPercent(MOTOR_PERCENT_MEDIUM);
    motorLeft.SetPercent(-MOTOR_PERCENT_MEDIUM);
    Sleep(3.5);

    //Angle robot towards starting box
    motorRight.SetPercent(-MOTOR_PERCENT_WEAK);
    motorLeft.SetPercent(-MOTOR_PERCENT_WEAK);
    Sleep(1.5);

    //Move robot into starting box
    motorRight.SetPercent(MOTOR_PERCENT_WEAK);
    motorLeft.SetPercent(-MOTOR_PERCENT_WEAK);
    Sleep(1.5);

    //Stop robot
    motorRight.Stop();
    motorLeft.Stop();
}
