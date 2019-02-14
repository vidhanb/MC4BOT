// Required proteus firmware libraries
#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHMotor.h>
#include <FEHServo.h>

// Required personal libraries
#include "include/constants.h"

//yuhhh

int main(void)
{

    float x,y;

    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    while( true )
    {
        if( LCD.Touch(&x,&y) )
        {
            LCD.WriteLine(MOTOR_FR);
            LCD.WriteLine(MOTOR_TEST);
            Sleep( 100 );
        }
    }
    return 0;
}
