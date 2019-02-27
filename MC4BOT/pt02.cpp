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

void pt02() {
    initRobot();
    printInit();

    /*
    | Measured movements:
    |
    | counterclockwise 70-80 degrees
    | move forward ~7 inches
    | clockwise 35 degrees
    | 6 inches to get to ramp
    | 10 inches up ramp
    | 23 inches across top
    | move right tread only for 4 inches
    | flip lever
    | redo backwards
    |
    */

    // Wait until start light turns on to move
    //while (cdsCell.Value() > CDS_CELL_START_THRESH);
    flipLeverReset();
    turnForAngle(20, MotorPercentWeak, DirectionClockwise);

    //Turn to face the direction to take
    motorRight.SetPercent(-MotorPercentWeak);
    motorLeft.SetPercent(-MotorPercentWeak);
    Sleep(3.0);
    motorRight.Stop();
    motorLeft.Stop();

    //Drive towards ramp
    driveForDistance(7.0, MotorPercentMedium, DirectionForward);
    
    //Turn to face ramp
    motorRight.SetPercent(MotorPercentWeak);
    motorLeft.SetPercent(MotorPercentWeak);
    Sleep(2.0);
    motorRight.Stop();
    motorLeft.Stop();
    
    //Drive up ramp
    driveForDistance(16, MotorPercentStrong, DirectionBackward);
    
    //Drive to lever
    driveForDistance(23, MotorPercentMedium, DirectionForward);

    //Move right tread
    motorRight.SetPercent(-MotorPercentMedium);
    Sleep(3.0);
    motorRight.Stop();

    //FLip the lever
    flipLever();
    Sleep(1.0);

    turnForAngle(45, MotorPercentMedium, DirectionClockwise);
    //yuhhhh
    return;
}
