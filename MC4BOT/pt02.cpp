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

void pt02() {
    initRobot();
    printInit();

    testTreadTurns();

    Sleep(15.0);

    // Wait until start light turns on to move
    while (cdsCell.Value() > CDS_CELL_START_THRESH);
    
    /*
    | Start with light
    | Move right to lights in front of DDR
    | Detect light, move to and push in appropriate button
    | Move back to appropriate spot
    | Rotate/Move to and up ramp
    | Move across top level and touch foosball lever
    */

   //Move up 4 inches
   driveForDistance(4., MotorPercentMedium, DirectionForward);

   //Turn 45 degrees clockwise
   turnForTime(2.9, MotorPercentWeak, DirectionClockwise);

   //Move 13 inches to the first light spot on the DDR track
   driveForDistance(14., MotorPercentMedium, DirectionForward);

   //Turn 90 degrees to face the DDR machine
   turnForTime(4., MotorPercentWeak, DirectionClockwise);

   //Move forward 3 incehs to read color on ground
    driveForDistance(3., MotorPercentMedium, DirectionForward);

   //Read value of color, first if statement is for red second is for blue
   bool color = true;
   if (cdsCell.Value() > 1 && cdsCell.Value() < 1.5 ) {
       //Drive into red button
       driveForTime(3., MotorPercentStrong, DirectionForward);
   } else {
       //Drive back
       driveForDistance(3., MotorPercentMedium, DirectionBackward);
       //Turn 90 degrees counter clockwise
       turnForTime(4., MotorPercentWeak, DirectionCounterClockwise);
       //Drive 5 inches and face blue button
       driveForDistance(5., MotorPercentMedium, DirectionForward);
       turnForTime(4., MotorPercentWeak, DirectionClockwise);
       //Drive into blue button
       driveForTime(3., MotorPercentStrong, DirectionForward);
       color = false;
   }

    //Drive up ramp to the foosball counters
    if (color) {
        driveForDistance(3., MotorPercentMedium, DirectionBackward);
        turnForTime(4., MotorPercentWeak, DirectionCounterClockwise);
        driveForDistance(63., MotorPercentMedium, DirectionForward);
    } else {
        driveForDistance(63., MotorPercentMedium, DirectionBackward);
    }
    
}
