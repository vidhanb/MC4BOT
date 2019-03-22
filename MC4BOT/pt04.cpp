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

    

    return;
}
