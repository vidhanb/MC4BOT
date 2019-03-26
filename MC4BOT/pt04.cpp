// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

// TODO - check rpsCheckHeading() function
//      - adjust CdS cell values (pause?)
//      - check proper turning to get to foosball
//      - make sure foosball claw is attached properly - setup method to attach it at right angle every time, or glue it
//      - switch to driveForDistance?
//      - use rpsCheckX() or rpsCheckY()? - need to get desired positions first

void pt04() {
    initRobot();
    printInit();

    testSensors();

    // Wait until start light turns on to move
    while (cdsCell.Value() > CDS_CELL_START_THRESH);

    //--------------------------------------------------------------------------------------------------------------------------------------------------//

        //Move robot up slightly
        driveForDistance(4.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Turn the robot CW to face the DDR button
        turnForAngle(45, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);

        rpsCheckHeading(270.0);
        Sleep(ACTION_SEP_PAUSE);


        //Move robot towards DDR
        driveForDistance(14.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        rpsCheckXCoord(31.5);
        Sleep(ACTION_SEP_PAUSE);


        //Angle robot towards DDR
        turnForAngle(90, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);

        rpsCheckHeading(180.0);
        Sleep(ACTION_SEP_PAUSE);


        //Press button
        driveForDistance(6.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Back away from DDR
        driveForDistance(3.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);

        //Rotate robot 180 degrees to face ramp
        turnForAngle(180, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);

        //USE RPS TO CHECK ANGLE
        rpsCheckHeading(0.0);
        Sleep(ACTION_SEP_PAUSE);

//--------------------------------------------------------------------------------------------------------------------------------------------------//
        //Foosball
        //Move robot up ramp to foosball
        driveForDistance(32.0, MotorPercentStrong, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        rpsCheckYCoord(53.0);
        Sleep(ACTION_SEP_PAUSE);

        //USE RPS TO CHECK ANGLE after up ramp
        rpsCheckHeading(0.0);
        Sleep(ACTION_SEP_PAUSE);

        //Continue moving robot forward to foosball
        driveForDistance(15.0, MotorPercentStrong, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);


        //MIGHT NEED TO ROTATE, MOVE FORWARD, ROTATE

        //Rotate robot 90 counterclockwise
        turnForAngle(90, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);

        driveForDistance(1.0, MotorPercentWeak, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Drop arm
        servoClaw.SetDegree(SERVO_CLAW_POS_ACTIVE);
        Sleep(ACTION_SEP_PAUSE);

        //Move robot forward
        driveForDistance(10.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Lift arm
        servoClaw.SetDegree(SERVO_CLAW_POS_NEUTRAL);
        Sleep(ACTION_SEP_PAUSE);

        driveForDistance(8.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);

        turnForAngle(90, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);

        driveForDistance(15.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);

        rpsCheckHeading(0.0);
        Sleep(ACTION_SEP_PAUSE);

        driveForDistance(35.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);


    return;
}
