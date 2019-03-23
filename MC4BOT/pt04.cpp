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
//      - use rpsCheckX() or rpsCheckY()?

void pt04() {
    initRobot();
    printInit();

    // Wait until start light turns on to move
    while (cdsCell.Value() > CDS_CELL_START_THRESH);

    //--------------------------------------------------------------------------------------------------------------------------------------------------//

        //Move robot up slightly
        driveForTime(2.1, MotorPercentWeak, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Turn the robot CW to face the DDR button
        turnForTime(1.7, MotorPercentWeak, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);

        //Move robot towards DDR
        driveForTime(2.9, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Angle robot towards DDR
        turnForTime(3.2, MotorPercentWeak, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);

        //Press button
        driveForTime(2.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Back away from DDR
        driveForTime(1.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);

        //Rotate robot 180 degrees to face ramp
        turnForTime(7.0, MotorPercentWeak, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);

        //USE RPS TO CHECK ANGLE
        rpsCheckHeading(95.0);
        Sleep(ACTION_SEP_PAUSE);

//--------------------------------------------------------------------------------------------------------------------------------------------------//
        //Foosball
        //Move robot up ramp to foosball
        driveForTime(4.0, MotorPercentStrong, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //USE RPS TO CHECK ANGLE after up ramp
        rpsCheckHeading(95.0);
        Sleep(ACTION_SEP_PAUSE);


        //Continue moving robot forward to foosball
        driveForTime(3.0, MotorPercentStrong, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);


        //MIGHT NEED TO ROTATE, MOVE FORWARD, ROTATE

        //Rotate robot 90 counterclockwise
        turnForTime(3.1, MotorPercentWeak, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);

        //Move forward slightly so claw can reach foosball counter
        driveForTime(1.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Drop arm
        servoClaw.SetDegree(SERVO_CLAW_POS_ACTIVE);
        Sleep(ACTION_SEP_PAUSE);

        //Move robot forward
        driveForTime(3.0, MotorPercentWeak, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        //Lift arm
        servoClaw.SetDegree(SERVO_CLAW_POS_NEUTRAL);
        Sleep(ACTION_SEP_PAUSE);


    return;
}
