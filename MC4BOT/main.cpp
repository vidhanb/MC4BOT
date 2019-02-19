// Required proteus firmware libraries
#include <FEHLCD.h>
#include <FEHUtility.h>
#include <FEHMotor.h>

// Required personal libraries
#include "include/constants.h"
#include "include/functions.h"

//yuhhh

int main(void)
{
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    initRobot();
    
    return 0;
}
