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

void pt04() {
    initRobot();
    printInit();

    // Wait until start light turns on to move
    while (cdsCell.Value() > CDS_CELL_START_THRESH);

    //--------------------------------------------------------------------------------------------------------------------------------------------------//

        //Detect color change and move robot for 5 seconds
        //while (cdsCell.Value() >= START_COLOR_VALUE);

        //Move robot up slightly
        driveForTime(2.1, MotorPercentWeak, DirectionForward);
        Sleep(1.0);

        //Turn the robot CW to face the DDR button
        turnForTime(1.7, MotorPercentWeak, DirectionClockwise);
        Sleep(1.0);

        //Move robot towards DDR
        driveForTime(2.9, MotorPercentMedium, DirectionForward);
        Sleep(1.0);

        //Angle robot towards DDR
        turnForTime(3.2, MotorPercentWeak, DirectionClockwise);
        Sleep(1.0);

        //Press button
        driveForTime(2.0, MotorPercentMedium, DirectionForward);
        Sleep(1.0);

        //Back away from DDR
        driveForTime(1.0, MotorPercentMedium, DirectionBackward);
        Sleep(1.0);

        //Rotate robot 180 degrees to face ramp
        turnForTime(7.0, MotorPercentWeak, DirectionClockwise);
        Sleep(1.0);

        //USE RPS TO CHECK ANGLE
        rpsCheckHeading(95.0);
//--------------------------------------------------------------------------------------------------------------------------------------------------//
        //Foosball
        //Move robot up ramp to foosball
        driveForTime(4.0, MotorPercentStrong, DirectionForward);
        Sleep(1.0);

        //USE RPS TO CHECK ANGLE after up ramp
        rpsCheckHeading(95.0);

        //Continue moving robot forward to foosball
        driveForTime(3.0, MotorPercentStrong, DirectionForward);

        //MIGHT NEED TO ROTATE, MOVE FORWARD, ROTATE

        //Rotate robot 90 counterclockwise
        turnForTime(3.1, MotorPercentWeak, DirectionCounterClockwise);
        Sleep(1.0);

        //Move forward slightly so claw can reach foosball counter
        driveForTime(1.0, MotorPercentMedium, DirectionForward);
        Sleep(1.0);

        //Drop arm
        servoClaw.SetDegree(SERVO_CLAW_POS_ACTIVE);
        Sleep(1.0);

        //Move robot forward
        driveForTime(3.0, MotorPercentWeak, DirectionForward);
        Sleep(1.0);

        //Lift arm
        servoClaw.SetDegree(SERVO_CLAW_POS_NEUTRAL);

    return;
}
