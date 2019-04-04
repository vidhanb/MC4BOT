// Required proteus firmware libraries
#include <FEHUtility.h>
#include <FEHLCD.h>
#include <FEHSD.h>
#include <FEHIO.h>
#include <FEHServo.h>
#include <FEHMotor.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

// Try not to print anything to LCD in this file
//   functions.cpp should be the single point-of-change
//   for data logging

void finalComp() {

    // Init sequence
    // If testing without RPS, remember to change in initRobot() function
    initRobot();
    //printInit();
    //competitionStart();

    // Moving from start box to DDR light
    {

        // Robot starts at (11.0, 11.0)

        // Move forward/northeast
        driveForDistanceProportion(1.75, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(312.0);

        // Rotate CW
        turnForAngleProportion(45.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(270.0);
        rpsCheckXCoordDynamic(12.25);

        // Move forward/east
        driveForDistanceProportion(12.25, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(24.5);

    }

    // Completing DDR machine interactions
    // Robot before this split is at (24.5, 12.25)

    if( g_cdsCell.Value() < CDS_CELL_DIV_BLUE_RED ) {
        
        // Red light
        LCD.SetBackgroundColor(RED);

        // Rotate CW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(12.25);

        // Move forward/south
        //   Note that proportional driving is not used so that the
        //   motors will be strong enough to keep moving forward until
        //   the encoder counts are reached even if the robot runs up
        //   against the DDR machine
        driveForDistance(3.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        // Mechanism
        rpsResetPress();
        Sleep(ACTION_SEP_PAUSE);

        // Move backward/north
        driveForDistanceProportion(4.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(13.0);

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(25.0);

        // Move forward/east
        driveForDistanceProportion(7.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(32.0);

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(13.0);

        // Robot is now facing ramp at (32.0, 13.0)

    } else {

        // Blue light
        LCD.SetBackgroundColor(BLUE);

        // Move forward/east
        driveForDistanceProportion(5.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(30.0);

        // Rotate CW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(12.25);

        // Move forward/south
        driveForDistance(3.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        // Mechanism
        rpsResetPress();
        Sleep(ACTION_SEP_PAUSE);

        // Move backward/north
        driveForDistanceProportion(4.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(13.0);

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(30.0);

        // Move forward/east
        driveForDistanceProportion(2.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(32.0);

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(13.0);

        // Robot is now facing ramp at (32.0, 13.0)

    }

    // Moving from DDR machine to foosball counter
    {

        // Move forward/north, up ramp
        driveForDistanceProportion(40.0, MotorPercentStrong, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(53.0);

        // Move forward/north, across upper level
        driveForDistanceProportion(15.0, MotorPercentStrong, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(68.0);

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);
        rpsCheckXCoordDynamic(32.0);

    }

    // Completing foosball counter interactions
    {
    
        // Move forward/west
        driveForDistanceProportion(2.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);
        rpsCheckXCoordDynamic(34.0);

        // Mechanism
        foosballDeploy();
        Sleep(ACTION_SEP_PAUSE);

        // Move counter slider
        //   Note that the right side is 20% stronger to account for friction from the arm
        turnForRatioTime(3.5, MotorPercentMedium, DirectionCounterClockwise, 0.8);
        
        // Mechanism and re-align, because sliding might mess up driving
        foosballRetract();
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);
        rpsCheckXCoordDynamic(24.0);

        // Move forward/west
        driveForDistanceProportion(6.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);

    }

    // Moving to and completing claw game joystick interactions
    {

        // Rotate CCW
        turnForAngleProportion(20.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(107.0);

        // Move forward/westsouthwest
        driveForDistanceProportion(6.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);

        // Rotate CW
        turnForAngleProportion(40.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);

        // Mechanism
        flipLever();

        // Move backward/eastsoutheast to knock claw joystick down
        driveForDistanceProportion(2.0, MotorPercentMedium, DirectionBackward);

        // Mechanism
        flipLeverReset();

        // Move forward/westnorthwest
        driveForDistanceProportion(2.0, MotorPercentMedium, DirectionForward);

        // Rotate CCW
        turnForAngleProportion(65.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(132.0);

    }

    // Moving to and completing claw game joystick interactions
    {

        // Move forward/southwest
        driveForDistanceProportion(8.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(132.0);

        // Rotate CCW
        turnForAngleProportion(45.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        // Move forward/south
        driveForDistanceProportion(20.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        // Rotate CW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);

        // Move backward/east
        driveForDistanceProportion(14.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);

        // Rotate CW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);

        // Move backward/south
        //   Note that proportional driving is not used so that the
        //   motors will be strong enough to keep moving backward until
        //   the encoder counts are reached even if the robot runs up
        //   against the coin machine
        driveForDistance(6.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);

        // Mechanism
        coinRelease();
        Sleep(ACTION_SEP_PAUSE);

        // Move forward/north
        driveForDistanceProportion(4.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);

    }

    // Moving to and pushing final button
    {
        
        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);

        // Move forward/west
        driveForDistanceProportion(14.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        // Move forward/south
        driveForDistanceProportion(36.0, MotorPercentMedium, DirectionForward);

    }

    // By this point the robot should have hit the final button and turned itself off

    SD.CloseLog();

    return;
}
