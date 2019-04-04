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

void finalComp() {
    initRobot();
    //printInit();
    //competitionStart();

    // Robot starts at (11.0, 11.0)

    driveForDistanceProportion(1.75, MotorPercentMedium, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(312.0);

    turnForAngleProportion(45.0, MotorPercentMedium, DirectionClockwise);
    Sleep(ACTION_SEP_PAUSE);

    rpsCheckHeadingDynamic(270.0);
    rpsCheckXCoordDynamic(12.25);

    // Robot now at (12.25, 12.25)

    driveForDistanceProportion(12.25, MotorPercentMedium, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);

    rpsCheckHeadingDynamic(267.0);
    rpsCheckXCoordDynamic(24.5);

    // Robot before this split is at (24.5, 12.25)

    if( cdsCell.Value() < CDS_CELL_DIV_BLUE_RED ) {
        // Red light
        LCD.SetBackgroundColor(RED);
        turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(12.25);

        driveForDistance(3.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        rpsResetPress();

        driveForDistanceProportion(4.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(13.0);

        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(25.0);

        driveForDistanceProportion(7.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(32.0);

        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(13.0);

        // Robot is now facing ramp at (32.0, 13.0)

    } else {
        // Blue light
        LCD.SetBackgroundColor(BLUE);
        driveForDistanceProportion(5.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(30.0);

        turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(12.25);

        driveForDistance(3.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);

        rpsResetPress();
        Sleep(ACTION_SEP_PAUSE);

        driveForDistanceProportion(4.0, MotorPercentMedium, DirectionBackward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(177.0);
        rpsCheckYCoordDynamic(13.0);

        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(30.0);

        driveForDistanceProportion(2.0, MotorPercentMedium, DirectionForward);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(267.0);
        rpsCheckXCoordDynamic(32.0);

        turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(ACTION_SEP_PAUSE);
        rpsCheckHeadingDynamic(357.0);
        rpsCheckYCoordDynamic(13.0);

        // Robot is now facing ramp at (32.0, 13.0)

    }

    driveForDistanceProportion(40.0, MotorPercentStrong, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(357.0);
    rpsCheckYCoordDynamic(53.0);

    driveForDistanceProportion(15.0, MotorPercentStrong, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(357.0);
    rpsCheckYCoordDynamic(68.0);

    turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(87.0);
    rpsCheckXCoordDynamic(32.0);
    
    driveForDistanceProportion(2.0, MotorPercentMedium, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(87.0);
    rpsCheckXCoordDynamic(34.0);

    foosballDeploy();
    Sleep(ACTION_SEP_PAUSE);

    turnForRatioTime(3.0, MotorPercentMedium, DirectionCounterClockwise, 0.7);
    
    foosballRetract();
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(87.0);
    rpsCheckXCoordDynamic(24.0);

    turnForAngleProportion(20.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(107.0);

    driveForDistanceProportion(6.0, MotorPercentMedium, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);

    turnForAngleProportion(40.0, MotorPercentMedium, DirectionClockwise);
    Sleep(ACTION_SEP_PAUSE);

    flipLever();

    driveForDistanceProportion(2.0, MotorPercentMedium, DirectionBackward);

    flipLeverReset();

    driveForDistanceProportion(2.0, MotorPercentMedium, DirectionForward);

    turnForAngleProportion(65.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(132.0);

    driveForDistanceProportion(8.0, MotorPercentMedium, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(132.0);

    turnForAngleProportion(45.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(177.0);

    driveForDistanceProportion(20.0, MotorPercentMedium, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(177.0);

    turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(87.0);

    driveForDistanceProportion(14.0, MotorPercentMedium, DirectionBackward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(87.0);

    turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(357.0);

    driveForDistance(6.0, MotorPercentMedium, DirectionBackward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(357.0);

    coinRelease();
    Sleep(ACTION_SEP_PAUSE);

    driveForDistanceProportion(4.0, MotorPercentMedium, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(357.0);

    turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(87.0);

    driveForDistanceProportion(14.0, MotorPercentMedium, DirectionForward);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(87.0);

    turnForAngleProportion(90.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(ACTION_SEP_PAUSE);
    rpsCheckHeadingDynamic(177.0);

    driveForDistanceProportion(36.0, MotorPercentMedium, DirectionForward);



    /*
    | PSEUDOCODE
    |
    | - Wait until cds cell reading is below threshold OR 30 seconds passes, whichever comes first
    | - Drive forward for ~6 inches
    | - Rotate 45degrees clockwise
    | - Drive forward ~8 inches
    | - Rotate 90 degrees clockwise
    | - Detect light color in front of DDR machine using cds cell
    | - Turn to appropriate DDR button
    | - Drive forwards until microswitch is pushed, indicating that DDR button has been pushed
    | - Drop lever servo in order to push RPS reset button, then pick back up
    | - Drive backwards two inches
    | - Rotate 90 degrees counterclockwise
    | - Drive forward 2 inches
    | - Rotate 90 degrees counterclockwise
    | - Drive with motors on high up ramp
    | - Drive across top section until microswitch hits foosball section
    | - Rotate 90 degrees counterclockwise
    | - Extend/grip claw
    | - Drive forwards until front-right microswitch hits far side of foosball section
    | - Release/retract claw
    | - Rotate 20 degrees counterclockwise
    | - Drive fowards 6 inches
    | - Rotate 40 degrees clockwise
    | - Drop lever servo
    | - Drive backwards 2 inches (to pull down lever)
    | - Turn 65 degrees counterclockwise
    | - Drive forward until front-right microswitch hits wall
    | - Rotate 45degrees counterclockwise
    | - Drive 1 foot forward
    | - Turn 90degrees clockwise
    | - Drive 8 inches backwards
    | - Release coin into slot from top level
    | - Drive forward 8 inches
    | - Rotate 90 degrees counterclockwise
    | - Drive forward 1 foot down ramp
    | - Rotate 45 degrees clockwise
    | - Drive forward until we push the finish button
    */

    SD.CloseLog();

    return;
}
