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

// Try not to print anything to LCD in this file.
//   functions.cpp should be the single point-of-change
//   for data logging

void finalComp() {

    // Init sequence
    // If testing without RPS, remember to change in initRobot() function
    initRobot();
    printInit();
    //competitionStart();

    // Moving from start box to DDR light
    {

        // Robot starts at (9.5, 12.5)
        rpsCheckHeadingDynamic(312.0);

        // Move forward/northeast
        driveForDistanceProportion(2.4, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(312.0);

        // Robot should now be at (11.2, 14.2)

        // Rotate CW
        turnForAngleProportion(45.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(11.5 + g_adjustX);

        // Robot should now be at (11.5, 12.8)

        // Move forward/east
        driveForDistanceProportion(12.8, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(24.3 + g_adjustX);

    }

    // Completing DDR machine interactions
    // Robot before this split is at (24.3, 12.8)

    if( g_cdsCell.Value() < CDS_CELL_DIV_BLUE_RED ) {
        
        // Red light
        LCD.SetBackgroundColor(RED);

        driveForDistanceProportion(2.5, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(26.8 + g_adjustX);

        // Robot should now be at (26.8, 12.8)

        // Rotate CW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(11.8 + g_adjustY);

        // Robot should now be at (24.8, 11.8)

        // Move forward/south
        //   Note that proportional driving is not used so that the
        //   motors will be strong enough to keep moving forward until
        //   the encoder counts are reached even if the robot runs up
        //   against the DDR machine
        driveForDistance(3.7, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        // Mechanism
        rpsResetPress();
        Sleep(ACTION_SEP_PAUSE);

        // Robot should now be at (24.8, 9.1)

        // Move backward/north
        driveForDistanceProportion(3.9, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(13.0 + g_adjustY);

        // Robot should now be at (24.8, 13.0)

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(25.0 + g_adjustX);

        // Robot should now be at (25.8, 15.0)

        // Move forward/east
        driveForDistanceProportion(6.2, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(32.0 + g_adjustX);

        // Robot should now be at (32.0, 15.0)

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(16.0 + g_adjustY);

        // Robot is now facing ramp at (30.0, 16.0)

    } else {

        // Blue light
        LCD.SetBackgroundColor(BLUE);

        // Move forward/east
        driveForDistanceProportion(6.6, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(30.9 + g_adjustX);

        // Robot should now be at (30.9, 12.8)

        // Rotate CW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(11.8 + g_adjustY);

        // Robot should now be at (28.9, 11.8)

        // Move forward/south
        driveForDistance(3.6, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        // Mechanism
        rpsResetPress();
        Sleep(ACTION_SEP_PAUSE);

        // Robot should now be at (28.9, 9.2)

        // Move backward/north
        driveForDistanceProportion(3.8, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(13.0 + g_adjustY);

        // Robot should now be at (28.9, 13.0)

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(30.0 + g_adjustX);

        // Robot should now be at (30.0, 15.0)

        // Move forward/east
        driveForDistanceProportion(2.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(32.0 + g_adjustX);

        // Robot should now be at (32.0, 15.0)

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(16.0 + g_adjustY);

        // Robot is now facing ramp at (30.0, 16.0)

    }

    // Moving from DDR machine to foosball counter
    {

        // Move forward/north, up ramp
        driveForDistanceProportion(40.0, MotorPercentStrong, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(53.0 + g_adjustY);

        // Move forward/north, across upper level
        driveForDistanceProportion(15.0, MotorPercentStrong, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(68.0 + g_adjustY);

        // Rotate CCW
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);
        rpsCheckXCoordDynamic(32.0 + g_adjustX);

    }

    // Completing foosball counter interactions
    {
    
        // Move forward/west
        driveForDistanceProportion(2.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(87.0);
        rpsCheckXCoordDynamic(34.0 + g_adjustX);

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
        rpsCheckXCoordDynamic(24.0 + g_adjustX);

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
