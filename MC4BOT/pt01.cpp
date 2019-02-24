// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
//#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

void pt01() {
    initRobot();
    printInit();

    // Wait until start light turns on to move
    while (cdsCell.Value() > CDS_CELL_START_THRESH);

    // Adjust robot position to face ramp
    driveForDistance(2.0, MotorPercentMedium, DirectionForward);
    // Wait for momentum to stop
    Sleep(1.0);
    turnForAngle(90, MotorPercentWeak, DirectionCounterClockwise);
    Sleep(1.0);
    driveForDistance(2.0, MotorPercentMedium, DirectionForward);
    Sleep(1.0);
    turnForAngle(45, MotorPercentWeak, DirectionClockwise);
    Sleep(1.0);
    //////////////////////////// adjust all distances below this line after measuring course ///////////////////////////////
    // Drive up the ramp and across the top platform
    driveForDistance(12.0, MotorPercentStrong, DirectionForward);
    driveForDistance(8.0, MotorPercentMedium, DirectionForward);
    Sleep(1.0);
    // Turn and advance the robot to face the lever
    turnForAngle(30, MotorPercentWeak, DirectionClockwise);
    Sleep(1.0);
    driveForDistance(2.0, MotorPercentWeak, DirectionForward);
    Sleep(1.0);
    flipLever();
    Sleep(1.0);
    // Retreat and turn robot
    driveForDistance(2.0, MotorPercentWeak, DirectionBackward);
    Sleep(1.0);
    turnForAngle(30, MotorPercentWeak, DirectionCounterClockwise);
    Sleep(1.0);
    // Retreat across the top platform and down the ramp
    driveForDistance(8.0, MotorPercentMedium, DirectionBackward);
    driveForDistance(12.0, MotorPercentWeak, DirectionBackward);
    return;
}
