#include <FEHLCD.h>
#include <FEHIO.h>
#include <FEHUtility.h>
#include <FEHServo.h>
#include <FEHMotor.h>

//Servo Motor Definitions
#define LEVER_MIN 740
#define LEVER_MAX 24950
#define LEVER_FLIP 90.
#define LEVER_NEUTRAL 180.
#define COIN_MIN 520
#define COIN_MAX 2455
#define COIN_NEUTRAL 180
#define COIN_FLIP 85

//Wheel Motor Definitions
//Right - negative goes forward
#define RIGHT_MOTOR_PORT Motor3
#define LEFT_MOTOR_PORT Motor2
#define MOTOR_PERCENT_STRONG 70
#define MOTOR_PERCENT_MEDIUM 50
#define MOTOR_PERCENT_WEAK 30

//CdS Cell Definitions
#define CDS_PORT P0_1
#define START_COLOR_VALUE 1.6

//Yuh

int main(void)
{
    //Initialize CdS cell
    AnalogInputPin cdsCell(FEHIO::CDS_PORT);

    //Initialize motors
    FEHMotor rightMotor(FEHMotor::RIGHT_MOTOR_PORT, 9.0);
    FEHMotor leftMotor(FEHMotor::LEFT_MOTOR_PORT, 9.0);

    //Initialize lever servo
    FEHServo leverServo(FEHServo::Servo0);
    leverServo.SetMin(LEVER_MIN);
    leverServo.SetMax(LEVER_MAX);
    leverServo.SetDegree(LEVER_NEUTRAL);

    //Initialize coin servo
    FEHServo coinServo(FEHServo::Servo3);
    coinServo.SetMin(COIN_MIN);
    coinServo.SetMax(COIN_MAX);
    coinServo.SetDegree(COIN_NEUTRAL);
    Sleep(1.0);

    //Detect color change and move robot for 5 seconds
    while (cdsCell.Value() >= START_COLOR_VALUE);

        //Move robot up slightly
        rightMotor.SetPercent(-MOTOR_PERCENT_WEAK);
        leftMotor.SetPercent(MOTOR_PERCENT_WEAK);
        Sleep(2.1);

        //Stop robot
        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Angle the robot around to face the ramp
        rightMotor.SetPercent(MOTOR_PERCENT_WEAK);
        leftMotor.SetPercent(MOTOR_PERCENT_WEAK);
        Sleep(1.8);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Move robot towards the ramp
        rightMotor.SetPercent(-MOTOR_PERCENT_MEDIUM);
        leftMotor.SetPercent(MOTOR_PERCENT_MEDIUM);
        Sleep(3.);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Angle robot towards ramp
        rightMotor.SetPercent(-MOTOR_PERCENT_WEAK);
        leftMotor.SetPercent(-MOTOR_PERCENT_WEAK);
        Sleep(3.8); //Sometimes 4.3 based on position of robot

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Move robot up the ramp
        rightMotor.SetPercent(-MOTOR_PERCENT_STRONG);
        leftMotor.SetPercent(MOTOR_PERCENT_STRONG);
        Sleep(5.0);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Angle robot towards the coin drop
        rightMotor.SetPercent(-MOTOR_PERCENT_WEAK);
        leftMotor.SetPercent(-MOTOR_PERCENT_WEAK);
        Sleep(3.8);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Move robot in front of the coin drop area
        rightMotor.SetPercent(-MOTOR_PERCENT_MEDIUM);
        leftMotor.SetPercent(MOTOR_PERCENT_MEDIUM);
        Sleep(3.0);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Angle so the rear of the robot faces the coin drop
        rightMotor.SetPercent(MOTOR_PERCENT_WEAK);
        leftMotor.SetPercent(MOTOR_PERCENT_WEAK);
        Sleep(3.7);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Back robot up to the coin drop
        rightMotor.SetPercent(MOTOR_PERCENT_MEDIUM);
        leftMotor.SetPercent(-MOTOR_PERCENT_MEDIUM);
        Sleep(2.0);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Drop coin
        coinServo.SetDegree(COIN_FLIP);
        Sleep(1.0);
//------------------------------------------------------------------------------------------------------------------------------------------------//
        //Move up slightly
        rightMotor.SetPercent(-MOTOR_PERCENT_MEDIUM);
        leftMotor.SetPercent(MOTOR_PERCENT_MEDIUM);
        Sleep(1.);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Angle robot to the left
        rightMotor.SetPercent(-MOTOR_PERCENT_WEAK);
        leftMotor.SetPercent(-MOTOR_PERCENT_WEAK);
        Sleep(2.2);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Move up slightly
        rightMotor.SetPercent(-MOTOR_PERCENT_MEDIUM);
        leftMotor.SetPercent(MOTOR_PERCENT_MEDIUM);
        Sleep(1.8);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Angle straight forward
        rightMotor.SetPercent(MOTOR_PERCENT_WEAK);
        leftMotor.SetPercent(MOTOR_PERCENT_WEAK);
        Sleep(2.2);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Move forward to the lever
        rightMotor.SetPercent(-MOTOR_PERCENT_MEDIUM);
        leftMotor.SetPercent(MOTOR_PERCENT_MEDIUM);
        Sleep(2.);

        rightMotor.Stop();
        leftMotor.Stop();
        Sleep(1.0);

        //Move right motor forward
        rightMotor.SetPercent(-MOTOR_PERCENT_MEDIUM);
        Sleep(2.);

        rightMotor.Stop();
        Sleep(1.0);

        //Flip down lever
        leverServo.SetDegree(LEVER_FLIP);
        Sleep(1.0);

        //Back up robot
        rightMotor.SetPercent(MOTOR_PERCENT_MEDIUM);
        leftMotor.SetPercent(-MOTOR_PERCENT_MEDIUM);
        Sleep(2.);

        rightMotor.Stop();
        leftMotor.Stop();

}