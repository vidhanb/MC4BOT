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
    | 
    |
    */

    // Wait until start light turns on to move
    //while (cdsCell.Value() > CDS_CELL_START_THRESH);
    
    /*
    | Start with light
    | Move right to lights in front of DDR
    | Detect light, move to and push in appropriate button
    | Move back to appropriate spot
    | Rotate/Move to and up ramp
    | Move across top level and touch foosball lever
    */

    return;
}
