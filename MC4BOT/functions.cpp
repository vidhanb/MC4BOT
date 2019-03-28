// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHIO.h>
#include <FEHLCD.h>
#include <FEHRPS.h>
#include <FEHSD.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

// Required C++ libraries
#include <cmath>

// Init functions //////////////////////////////////////////

void initRobot() {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    servoLever.SetMin(SERVO_LEVER_MIN);
    servoLever.SetMax(SERVO_LEVER_MAX);
    flipLeverReset();
    servoCoin.SetMin(SERVO_COIN_MIN);
    servoCoin.SetMax(SERVO_COIN_MAX);
    servoCoin.SetDegree(SERVO_COIN_POS_NEUTRAL);
    servoClaw.SetMin(SERVO_CLAW_MIN);
    servoClaw.SetMax(SERVO_CLAW_MAX);
    servoClaw.SetDegree(SERVO_CLAW_POS_NEUTRAL);

    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();

    //RPS.InitializeTouchMenu();

    SD.OpenLog();

    return;
}

void printInit() {
    // decide on preambles for SD data logging - init, run? where to put initial RPS tests?

    // Print info for drive motors
    LCD.Write("Left Motor Init-{");
    LCD.Write("Port:");
    LCD.Write(MOTOR_PORT_FL);
    LCD.Write(",Volts:");
    LCD.Write(MOTOR_VOLTS);
    LCD.WriteLine("}");

    LCD.Write("Right Motor Init-{");
    LCD.Write("Port:");
    LCD.Write(MOTOR_PORT_FR);
    LCD.Write(",Volts:");
    LCD.Write(MOTOR_VOLTS);
    LCD.WriteLine("}");

    Sleep(1.0);

    // Print info for servo motors
    LCD.Write("Lever Servo Init-{");
    LCD.Write("Port:");
    LCD.Write(SERVO_PORT_LEVER);
    LCD.Write(",Min:");
    LCD.WriteLine(SERVO_LEVER_MIN);
    LCD.Write(",Max:");
    LCD.Write(SERVO_LEVER_MAX);
    LCD.WriteLine("}");

    LCD.Write("Coin Servo Init-{");
    LCD.Write("Port:");
    LCD.Write(SERVO_PORT_COIN);
    LCD.Write(",Min:");
    LCD.WriteLine(SERVO_COIN_MIN);
    LCD.Write(",Max:");
    LCD.Write(SERVO_COIN_MAX);
    LCD.WriteLine("}");

    LCD.Write("Claw Servo Init-{");
    LCD.Write("Port:");
    LCD.Write(SERVO_PORT_CLAW);
    LCD.Write(",Min:");
    LCD.WriteLine(SERVO_CLAW_MIN);
    LCD.Write(",Max:");
    LCD.Write(SERVO_CLAW_MAX);
    LCD.WriteLine("}");

    Sleep(1.0);

    // Print info for sensors
    LCD.Write("CdS Cell Init-{");
    LCD.Write("Port:");
    LCD.Write(CDS_CELL_PORT);
    LCD.Write(",D/B thresh:");
    LCD.Write(CDS_CELL_DIV_DARK_BLUE);
    LCD.Write(",B/R thresh:");
    LCD.Write(CDS_CELL_DIV_BLUE_RED);
    LCD.WriteLine("}");

    LCD.Write("Encoder left port: ");
    LCD.WriteLine(ENCODER_LEFT_PORT);
    LCD.Write("Encoder right port: ");
    LCD.WriteLine(ENCODER_RIGHT_PORT);
    LCD.Write("Encoder counts/inch: ");
    LCD.WriteLine(ENCODER_CTS_PER_INCH);

    LCD.WriteLine("Setup and init log complete.");
    return;
}

void competitionStart() {
    LCD.Clear();
    LCD.WriteLine("");
    LCD.WriteLine("AWAITING FINAL ACTION");
    LCD.WriteLine("");
    LCD.WriteLine("PRESS ANYWHERE TO BEGIN");
    float xTouch, yTouch;
    // Wait until screen is touched...
    while( !LCD.Touch(&xTouch, &yTouch) ) {};
    // ... then released
    while( LCD.Touch(&xTouch, &yTouch) ) {};
    unsigned int finalActionTime = TimeNowSec();
    LCD.WriteLine("STARTED");
    LCD.WriteLine("AWAITING COURSE OR TIMEOUT");
    // Wait until the start light turns on, or somehow 30 seconds has passed and we missed it
    while ( (cdsCell.Value() > CDS_CELL_DIV_DARK_BLUE) && (TimeNowSec() < (finalActionTime + 30)) );
    // Exit this function and let the games begin!
}

////////////////////////////////////////////////////////////
// Test functions //////////////////////////////////////////

void testDriveStraight() {
    motorLeft.SetPercent(100);
    motorRight.SetPercent(-100);
    Sleep(3.0);
    motorLeft.SetPercent(-100);
    motorRight.SetPercent(100);
    Sleep(3.0);
    motorLeft.Stop();
    motorRight.Stop();
    return;
}

void testSensors() {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    while(true) {
        LCD.Write("CdS: ");
        LCD.WriteLine(cdsCell.Value());
        LCD.Write("Left encoder: ");
        LCD.WriteLine(encoderLeft.Counts());
        LCD.Write("Right encoder: ");
        LCD.WriteLine(encoderRight.Counts());
        Sleep(0.3);
    }
}

void testRPS() {
    float touch_x, touch_y;
    RPS.InitializeTouchMenu();

    LCD.WriteLine("Press Screen to Start");
    while(!LCD.Touch(&touch_x, &touch_y));

    LCD.Clear();

    LCD.WriteRC("RPS Test Program", 0, 0);
    LCD.WriteRC("X Position: ", 2, 0);
    LCD.WriteRC("Y Position: ", 3, 0);
    LCD.WriteRC("   Heading: ", 4, 0);

    while(true) {
        LCD.WriteRC(RPS.X(), 2, 13);
        LCD.WriteRC(RPS.Y(), 3, 13);
        LCD.WriteRC(RPS.Heading(), 4, 13);

        Sleep(10); // 10ms
    }
    // Will never exit infinite loop to reach here
    return;
}

void testDriveDistanceLong() {
    driveForDistance(50.0, MotorPercentMedium, DirectionForward);
    LCD.Write("left: ");
    LCD.WriteLine(encoderLeft.Counts());
    LCD.Write("right: ");
    LCD.WriteLine(encoderRight.Counts());
}

void testDriveDirections() {
    LCD.WriteLine(MotorPercentMedium * MOTOR_SIDE_DIR_CORRECTOR);
    LCD.WriteLine("turn right");
    turnForAngle(30.0, MotorPercentMedium, DirectionClockwise);
    Sleep(2.0);
    LCD.WriteLine("turn left");
    turnForAngle(30.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(2.0);
    LCD.WriteLine("forwards");
    driveForDistance(2.0, MotorPercentMedium, DirectionForward);
    Sleep(2.0);
    LCD.WriteLine("backwards");
    driveForDistance(2.0, MotorPercentMedium, DirectionBackward);
    Sleep(4.0);
}

void testDriveFunctions() {
    driveForDistance(4.0, MotorPercentMedium, DirectionForward);
    driveForTime(4.0, MotorPercentMedium, DirectionBackward);
    turnForAngle(90.0, MotorPercentMedium, DirectionClockwise);
    turnForTime(2.0, MotorPercentMedium, DirectionCounterClockwise);
    turnToCourseAngle(90.0, 180, MotorPercentMedium);
    turnToCourseAngle(90.0, 0, MotorPercentMedium);
    LCD.WriteLine("Done");
}

void testTreadTurns() {
   while(true) {
       turnForAngle(180.0, MotorPercentMedium, DirectionClockwise);
       Sleep(1.0);
       turnForAngle(180.0, MotorPercentMedium, DirectionCounterClockwise);
       Sleep(1.0);
   }
}

void testServoRange() {
    servoLever.SetDegree(180);
    LCD.WriteLine(180);
    Sleep(2.0);
    servoLever.SetDegree(135);
    LCD.WriteLine(135);
    Sleep(2.0);
    servoLever.SetDegree(90);
    LCD.WriteLine(90);
    Sleep(2.0);
    servoLever.SetDegree(45);
    LCD.WriteLine(45);
    Sleep(2.0);
}

void testServos() {
    LCD.WriteLine("servo time");
    rpsResetPress();
    Sleep(1.0);
    LCD.WriteLine("coin");
    coinRelease();
    Sleep(1.0);
    servoCoin.SetDegree(SERVO_COIN_POS_NEUTRAL);
    Sleep(1.0);
    LCD.WriteLine("claw");
    foosballDeploy();
    LCD.WriteLine("claw down");
    Sleep(1.0);
    foosballRetract();
    LCD.WriteLine("claw up");
    Sleep(1.0);
    LCD.WriteLine("servo tests done");
}

////////////////////////////////////////////////////////////
// Mechanism functions /////////////////////////////////////

void flipLever() {
    servoLever.SetDegree(SERVO_LEVER_POS_ACTIVE);
    return;
}

void flipLeverReset() {
    servoLever.SetDegree(SERVO_LEVER_POS_NEUTRAL);
    return;
}

void rpsResetPress() {
    int currAngle = SERVO_LEVER_POS_NEUTRAL;
    // Slow down the servo motor's movement so that it has more torque
    // 45 2-degree steps * 0.01seconds per step = 0.45 seconds 
    while(currAngle > SERVO_LEVER_POS_ACTIVE) {
        servoLever.SetDegree(currAngle);
        currAngle-= 2;
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
    Sleep(SERVO_LEVER_RESET_PAUSE);
    while(currAngle < SERVO_LEVER_POS_NEUTRAL) {
        servoLever.SetDegree(currAngle);
        currAngle+= 2;
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
    return;
}

void foosballDeploy() {
    int currAngle = SERVO_CLAW_POS_NEUTRAL;
    // Slow down the servo motor's movement so that it has more torque
    // 38 2-degree steps * 0.01seconds per step = 0.38 seconds 
    while(currAngle < SERVO_CLAW_POS_ACTIVE) {
        servoClaw.SetDegree(currAngle);
        currAngle+= 2;
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
}

void foosballRetract() {
    int currAngle = SERVO_CLAW_POS_ACTIVE;
    // Slow down the servo motor's movement so that it has more torque
    // 38 2-degree steps * 0.01seconds per step = 0.38 seconds 
    while(currAngle > SERVO_CLAW_POS_NEUTRAL) {
        servoClaw.SetDegree(currAngle);
        currAngle-= 2;
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
}

void coinRelease() {
    servoCoin.SetDegree(SERVO_COIN_POS_ACTIVE);
}

////////////////////////////////////////////////////////////
// Drive functions /////////////////////////////////////////

float accelerationFunction(float ratio) {
    // Equivalent to function: -16(x-0.5)^4 + 1
    float result = (-10.0 * std::pow( (ratio - 0.5), 4.0) ) + 1.0;
    return result;
}

void driveForDistance(float inches, MotorPower motorPercent, DriveDirection direction) {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    float expectedEncoderCounts = inches * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionForward) {
        LCD.WriteLine("Going FW");
        // Drive left motor forwards
        motorLeft.SetPercent(motorPercent);
        // Drive right motor forwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.WriteLine("Going BW");
        // Drive left motor backwards
        motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        // Drive right motor backwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_STR_CORRECTOR);
    }
    float currentEncoderCounts = 0.0;
    while( currentEncoderCounts < expectedEncoderCounts) {
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( encoderLeft.Counts() + encoderRight.Counts() ) / 2.0;
    }
    LCD.Write("Left encoder: ");
    LCD.WriteLine(encoderLeft.Counts());
    LCD.Write("Right encoder: ");
    LCD.WriteLine(encoderRight.Counts());
    motorLeft.Stop();
    motorRight.Stop();
    LCD.WriteLine("--- Drive Done ---");
    return;
}

void driveForDistanceAccelMap(float inches, int motorPercent, DriveDirection direction) {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    float expectedEncoderCounts = inches * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionForward) {
        LCD.WriteLine("Going FW");
    } else {
        LCD.WriteLine("Going BW");
        motorPercent *= MOTOR_SIDE_DIR_CORRECTOR;
    }
    float currentEncoderCounts = 0.0;
    float currentDistanceRatio = 0.0;
    float currentAccelMult = 0.0;
    while( currentEncoderCounts < expectedEncoderCounts) {
        // See how much of our journey we've completed so far
        currentDistanceRatio = ( currentEncoderCounts / expectedEncoderCounts );
        // Map above proportion value to a multiplier for smoother acceleration
        currentAccelMult = accelerationFunction(currentDistanceRatio);
        // Set motor percents according to above mapped value
        // Drive left motor
        motorLeft.SetPercent(motorPercent * currentAccelMult);
        // Drive right motor, with strength adjuster
        motorRight.SetPercent(motorPercent * currentAccelMult * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( encoderLeft.Counts() + encoderRight.Counts() ) / 2.0;
    }
    LCD.Write("Left encoder: ");
    LCD.WriteLine(encoderLeft.Counts());
    LCD.Write("Right encoder: ");
    LCD.WriteLine(encoderRight.Counts());
    motorLeft.Stop();
    motorRight.Stop();
    LCD.WriteLine("--- Drive Done ---");
    return;
}

void driveForDistanceProportion(float inches, int motorPercent, DriveDirection direction) {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    float expectedEncoderCounts = inches * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionForward) {
        LCD.WriteLine("Going FW");
    } else {
        LCD.WriteLine("Going BW");
        motorPercent *= MOTOR_SIDE_DIR_CORRECTOR;
    }
    float leftEncoderCounts = 0.0;
    float rightEncoderCounts = 0.0;
    float encoderProportion = 0.0;
    float currentEncoderCounts = 0.0;
    float currentDistanceRatio = 0.0;
    float currentAccelMult = 0.0;
    while( currentEncoderCounts < expectedEncoderCounts) {
        // See how much of our journey we've completed so far
        currentDistanceRatio = ( currentEncoderCounts / expectedEncoderCounts );
        // Map above proportion value to a multiplier for smoother acceleration
        currentAccelMult = accelerationFunction(currentDistanceRatio);
        if( currentDistanceRatio > 0.1 ) {
            encoderProportion = (leftEncoderCounts / rightEncoderCounts) * IDEAL_RTOL_ENCODER_RATIO;
        } else {
            encoderProportion = 1.0;
        }
        // Set motor percents according to above mapped value
        // Drive left motor
        motorLeft.SetPercent(motorPercent * currentAccelMult);
        // Drive right motor, with strength adjuster
        motorRight.SetPercent(motorPercent * currentAccelMult * encoderProportion * MOTOR_SIDE_DIR_CORRECTOR);
        //motorRight.SetPercent(motorPercent * currentAccelMult * encoderProportion * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
        //motorRight.SetPercent(motorPercent * currentAccelMult * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
        leftEncoderCounts = encoderLeft.Counts();
        rightEncoderCounts = encoderRight.Counts();
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( leftEncoderCounts + rightEncoderCounts ) / 2.0;
    }
    LCD.Write("Left encoder: ");
    LCD.WriteLine(encoderLeft.Counts());
    LCD.Write("Right encoder: ");
    LCD.WriteLine(encoderRight.Counts());
    motorLeft.Stop();
    motorRight.Stop();
    LCD.WriteLine("--- Drive Done ---");
    return;
}

void driveForTime(float seconds, MotorPower motorPercent, DriveDirection direction) {
    if(direction == DirectionForward) {
        LCD.WriteLine("Going FW");
        // Drive left motor forwards
        motorLeft.SetPercent(motorPercent);
        // Drive right motor forwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.WriteLine("Going BW");
        // Drive left motor backwards
        motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        // Drive right motor backwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_STR_CORRECTOR);
    }
    LCD.Write("Drive time: ");
    LCD.WriteLine(seconds);
    Sleep(seconds);
    motorLeft.Stop();
    motorRight.Stop();
    LCD.WriteLine("--- Drive Done ---");
    return;
}

void turnForTime(float seconds, MotorPower motorPercent, TurnDirection direction) {
    if(direction == DirectionClockwise) {
        LCD.WriteLine("Going CW");
        // Drive left motor forwards
        motorLeft.SetPercent(motorPercent);
        // Drive right motor backwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.WriteLine("Going CNTCW");
        // Drive left motor backwards
        motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        // Drive right motor forwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    }
    LCD.Write("Turn time: ");
    LCD.WriteLine(seconds);
    Sleep(seconds);
    motorLeft.Stop();
    motorRight.Stop();
    LCD.WriteLine("--- Turn Done ---");
    return;
}

void turnForRatioTime(float seconds, MotorPower motorPercent, TurnDirection direction, float motorRatio) {
    if(direction == DirectionClockwise) {
        LCD.Write("Going CW, ratio ");
        LCD.WriteLine(motorRatio);
        // Drive left motor forwards
        motorLeft.SetPercent(motorPercent);
        // Drive right motor, at a proportional lesser value, forwards, with strength adjuster
        motorRight.SetPercent(motorPercent * motorRatio * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.Write("Going CNTCW, ratio ");
        LCD.WriteLine(motorRatio);
        // Drive left motor forwards, at a proportional lesser value
        motorLeft.SetPercent(motorPercent * motorRatio);
        // Drive right motor, forwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    }
    LCD.Write("Turn time: ");
    LCD.WriteLine(seconds);
    Sleep(seconds);
    motorLeft.Stop();
    motorRight.Stop();
    LCD.WriteLine("--- Turn Done ---");
    return;
}

void turnForAngle(float targetAngle, MotorPower motorPercent, TurnDirection direction) {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    float arcLength = (targetAngle / 360.0) * ROBOT_TURN_CIRC;
    LCD.Write("Turn arc length: ");
    LCD.WriteLine(arcLength);   
    float expectedEncoderCounts = arcLength * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionClockwise) {
        LCD.WriteLine("Going CW");
        // Drive left motor forward
        motorLeft.SetPercent(motorPercent);
        // Drive right motor backwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.WriteLine("Going CNTCW");
        // Drive left motor backwards
        motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        // Drive right motor forwards, with strength adjuster
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    }
    while( ( encoderLeft.Counts() + encoderRight.Counts() ) / 2.0 < expectedEncoderCounts);
    motorLeft.Stop();
    motorRight.Stop();
    LCD.Write("L enc: ");
    LCD.WriteLine(encoderLeft.Counts());
    LCD.Write("R enc: ");
    LCD.WriteLine(encoderRight.Counts());
    LCD.WriteLine("--- Turn Done ---");
    return;
}

void turnForAngleAccelMap(float targetAngle, int motorPercent, TurnDirection direction) {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    float arcLength = (targetAngle / 360.0) * ROBOT_TURN_CIRC;
    LCD.Write("Turn arc length: ");
    LCD.WriteLine(arcLength);   
    float expectedEncoderCounts = arcLength * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionClockwise) {
        LCD.WriteLine("Going CW");
    } else {
        LCD.WriteLine("Going CNTCW");
        motorPercent *= MOTOR_SIDE_DIR_CORRECTOR;
    }
    float currentEncoderCounts = 0.0;
    float currentDistanceRatio = 0.0;
    float currentAccelMult = 0.0;
    while( currentEncoderCounts < expectedEncoderCounts) {
        // See how much of our journey we've completed so far
        currentDistanceRatio = ( currentEncoderCounts / expectedEncoderCounts );
        // Map above proportion value to a multiplier for smoother acceleration
        currentAccelMult = accelerationFunction(currentDistanceRatio);
        // Set motor percents according to above mapped value
        // Drive left motor
        motorLeft.SetPercent(motorPercent * currentAccelMult);
        // Drive right motor, with strength adjuster
        motorRight.SetPercent(motorPercent * currentAccelMult * MOTOR_SIDE_STR_CORRECTOR);
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( encoderLeft.Counts() + encoderRight.Counts() ) / 2.0;
    }
    motorLeft.Stop();
    motorRight.Stop();
    LCD.Write("L enc: ");
    LCD.WriteLine(encoderLeft.Counts());
    LCD.Write("R enc: ");
    LCD.WriteLine(encoderRight.Counts());
    LCD.WriteLine("--- Turn Done ---");
    return;
}

void turnForAngleProportion(float targetAngle, int motorPercent, TurnDirection direction) {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    float arcLength = (targetAngle / 360.0) * ROBOT_TURN_CIRC;
    LCD.Write("Turn arc length: ");
    LCD.WriteLine(arcLength);   
    float expectedEncoderCounts = arcLength * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    float turnEncoderRatio;
    if(direction == DirectionClockwise) {
        LCD.WriteLine("Going CW");
        turnEncoderRatio = 1.18;
        expectedEncoderCounts *= 1.02;
    } else {
        LCD.WriteLine("Going CNTCW");
        turnEncoderRatio = 0.98;
        motorPercent *= MOTOR_SIDE_DIR_CORRECTOR;
    }
    float leftEncoderCounts = 0.0;
    float rightEncoderCounts = 0.0;
    float encoderProportion = 0.0;
    float currentEncoderCounts = 0.0;
    float currentDistanceRatio = 0.0;
    float currentAccelMult = 0.0;
    while( currentEncoderCounts < expectedEncoderCounts) {
        // See how much of our journey we've completed so far
        currentDistanceRatio = ( currentEncoderCounts / expectedEncoderCounts );
        // Map above proportion value to a multiplier for smoother acceleration
        currentAccelMult = accelerationFunction(currentDistanceRatio);
        if( currentDistanceRatio > 0.1 ) {
            encoderProportion = (leftEncoderCounts / rightEncoderCounts) * turnEncoderRatio;
        } else {
            encoderProportion = 1.0;
        }
        // Set motor percents according to above mapped value
        // Drive left motor
        motorLeft.SetPercent(motorPercent * currentAccelMult);
        // Drive right motor, with strength adjuster
        motorRight.SetPercent(motorPercent * currentAccelMult * encoderProportion);
        leftEncoderCounts = encoderLeft.Counts();
        rightEncoderCounts = encoderRight.Counts();
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( leftEncoderCounts + rightEncoderCounts ) / 2.0;
    }
    motorLeft.Stop();
    motorRight.Stop();
    LCD.Write("L enc: ");
    LCD.WriteLine(encoderLeft.Counts());
    LCD.Write("R enc: ");
    LCD.WriteLine(encoderRight.Counts());
    LCD.WriteLine("--- Turn Done ---");
    return;
}

void turnToCourseAngle(float currentAngle, float targetAngle, MotorPower motorPercent) {
    if(currentAngle > targetAngle) {
        if( (currentAngle - targetAngle) < 180.0) {
            LCD.Write("Turn deg: ");
            LCD.WriteLine( (currentAngle - targetAngle) );
            turnForAngle( (currentAngle - targetAngle) , motorPercent, DirectionClockwise );
        } else {
            LCD.Write("Turn deg: ");
            LCD.WriteLine( 360.0 - (currentAngle - targetAngle) );
            turnForAngle( 360.0 - (currentAngle - targetAngle) , motorPercent, DirectionCounterClockwise );
        }
    } else {
        if( (targetAngle - currentAngle) < 180.0) {
            LCD.Write("Turn deg: ");
            LCD.WriteLine( (targetAngle - currentAngle) );
            turnForAngle( (targetAngle - currentAngle) , motorPercent, DirectionCounterClockwise );
        } else {
            LCD.Write("Turn deg: ");
            LCD.WriteLine( 360.0 - (targetAngle - currentAngle) );
            turnForAngle( 360.0 - (targetAngle - currentAngle) , motorPercent, DirectionClockwise );
        }
    }
    return;
}

//void driveUntil(int motorPower, special type for function pointer CALLBACKFUNC) {}

// DEPRECATED - BREAKS IN A BAD WAY WHEN RPS IS BROKEN
// Use RPS to get current heading, then calculate appropriate turn to reach target
void turnToCourseAngle(float targetAngle, MotorPower motorPercent) {
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    float currentAngle = currentHeading;
    turnToCourseAngle(currentAngle, targetAngle, motorPercent);
    return;
}

////////////////////////////////////////////////////////////
// RPS functions ///////////////////////////////////////////

void rpsCheckHeadingConstant(float targetHeading) {
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    float headingDifference = currentHeading - targetHeading;
    while( std::abs(headingDifference) > 3.0) {
        LCD.Write("Target angle diff: ");
        LCD.WriteLine( headingDifference );
        if(headingDifference > 0.0 && headingDifference < 180.0) {
            turnForAngle(1.0, MotorPercentWeak, DirectionClockwise);
        } else if(headingDifference < 0.0 && headingDifference > -180.0) {
            turnForAngle(1.0, MotorPercentWeak, DirectionCounterClockwise);
        } else if(headingDifference > 180.0) {
            turnForAngle(1.0, MotorPercentWeak, DirectionCounterClockwise);
        } else if(headingDifference < -180.0) {
            turnForAngle(1.0, MotorPercentWeak, DirectionClockwise);
        }
        Sleep(ACTION_SEP_PAUSE);
        currentHeading = rpsSampleHeading();
        if(currentHeading < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
        headingDifference = currentHeading - targetHeading;
    }
    return;
}

void rpsCheckXCoordConstant(float targetX) {
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    bool facingPlus;
    if(currentHeading < 180.0) {
        facingPlus = false;
    } else {
        facingPlus = true;
    }
    float currentXCoord = rpsSampleXCoord();
    if(currentXCoord < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    while( std::abs(currentXCoord - targetX) > 1.0 ) {
        if(currentXCoord < targetX && facingPlus) {
            driveForDistance(0.25, MotorPercentWeak, DirectionForward);
        } else if(currentXCoord > targetX && facingPlus) {
            driveForDistance(0.25, MotorPercentWeak, DirectionBackward);
        } else if(currentXCoord < targetX && !facingPlus) {
            driveForDistance(0.25, MotorPercentWeak, DirectionBackward);
        } else if(currentXCoord > targetX && !facingPlus) {
            driveForDistance(0.25, MotorPercentWeak, DirectionForward);
        }
        Sleep(ACTION_SEP_PAUSE);
        currentXCoord = rpsSampleXCoord();
        if(currentXCoord < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
    }
    return;
}

void rpsCheckYCoordConstant(float targetY) {
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    bool facingPlus;
    if(currentHeading > 90.0 && currentHeading < 270.0) {
        facingPlus = false;
    } else {
        facingPlus = true;
    }
    float currentYCoord = rpsSampleYCoord();
    if(currentYCoord < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    while( std::abs(currentYCoord - targetY) > 1.0 ) {
        if(currentYCoord < targetY && facingPlus) {
            driveForDistance(0.25, MotorPercentWeak, DirectionForward);
        } else if(currentYCoord > targetY && facingPlus) {
            driveForDistance(0.25, MotorPercentWeak, DirectionBackward);
        } else if(currentYCoord < targetY && !facingPlus) {
            driveForDistance(0.25, MotorPercentWeak, DirectionBackward);
        } else if(currentYCoord > targetY && !facingPlus) {
            driveForDistance(0.25, MotorPercentWeak, DirectionForward);
        }
        Sleep(ACTION_SEP_PAUSE);
        currentYCoord = rpsSampleYCoord();
        if(currentYCoord < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
    }
    return;
}

void rpsCheckHeadingDynamic(float targetHeading) {
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    float headingDifference = currentHeading - targetHeading;
    while( std::abs(headingDifference) > 3.0) {
        LCD.Write("Target angle diff: ");
        LCD.WriteLine( headingDifference );
        if(headingDifference > 0.0 && headingDifference < 180.0) {
            turnForAngle(std::abs(headingDifference), MotorPercentWeak, DirectionClockwise);
        } else if(headingDifference < 0.0 && headingDifference > -180.0) {
            turnForAngle(std::abs(headingDifference), MotorPercentWeak, DirectionCounterClockwise);
        } else if(headingDifference > 180.0) {
            turnForAngle(std::abs(headingDifference), MotorPercentWeak, DirectionCounterClockwise);
        } else if(headingDifference < -180.0) {
            turnForAngle(std::abs(headingDifference), MotorPercentWeak, DirectionClockwise);
        }
        Sleep(ACTION_SEP_PAUSE);
        currentHeading = rpsSampleHeading();
        if(currentHeading < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
        headingDifference = currentHeading - targetHeading;
    }
    return;
}

void rpsCheckXCoordDynamic(float targetX) {
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    bool facingPlus;
    if(currentHeading < 180.0) {
        facingPlus = false;
    } else {
        facingPlus = true;
    }
    float currentXCoord = rpsSampleXCoord();
    if(currentXCoord < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    float positionXDifference = std::abs(currentXCoord - targetX);
    while( std::abs(currentXCoord - targetX) > 1.0 ) {
        if(currentXCoord < targetX && facingPlus) {
            driveForDistance(positionXDifference, MotorPercentWeak, DirectionForward);
        } else if(currentXCoord > targetX && facingPlus) {
            driveForDistance(positionXDifference, MotorPercentWeak, DirectionBackward);
        } else if(currentXCoord < targetX && !facingPlus) {
            driveForDistance(positionXDifference, MotorPercentWeak, DirectionBackward);
        } else if(currentXCoord > targetX && !facingPlus) {
            driveForDistance(positionXDifference, MotorPercentWeak, DirectionForward);
        }
        Sleep(ACTION_SEP_PAUSE);
        currentXCoord = rpsSampleXCoord();
        if(currentXCoord < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
        positionXDifference = std::abs(currentXCoord - targetX);
    }
    return;
}

void rpsCheckYCoordDynamic(float targetY) {
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    bool facingPlus;
    if(currentHeading > 90.0 && currentHeading < 270.0) {
        facingPlus = false;
    } else {
        facingPlus = true;
    }
    float currentYCoord = rpsSampleYCoord();
    if(currentYCoord < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    float positionYDifference = std::abs(currentYCoord - targetY);
    while( std::abs(currentYCoord - targetY) > 1.0 ) {
        if(currentYCoord < targetY && facingPlus) {
            driveForDistance(positionYDifference, MotorPercentWeak, DirectionForward);
        } else if(currentYCoord > targetY && facingPlus) {
            driveForDistance(positionYDifference, MotorPercentWeak, DirectionBackward);
        } else if(currentYCoord < targetY && !facingPlus) {
            driveForDistance(positionYDifference, MotorPercentWeak, DirectionBackward);
        } else if(currentYCoord > targetY && !facingPlus) {
            driveForDistance(positionYDifference, MotorPercentWeak, DirectionForward);
        }
        Sleep(ACTION_SEP_PAUSE);
        currentYCoord = rpsSampleYCoord();
        if(currentYCoord < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
        positionYDifference = std::abs(currentYCoord - targetY);
    }
    return;
}

// Wrap RPS.Heading() for some automatic error detection
float rpsSampleHeading() {
    // Keep track of how many times we resample to avoid an incorrect value
    int extraAttempts = 0;
    float sampleOne, sampleTwo, sampleThree, sampleFinal;
    sampleOne = RPS.Heading();
    // If sample outside of range and we haven't exceeded the max number of
    //   incorrect values, try again
    while( (sampleOne < 0.0 || sampleOne > 360.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleOne = RPS.Heading();
    }
    // At this point, either the sample is good or we've exceeded the max number
    //   of incorrect values. If the sample is bad, quit trying and return an error value
    if(sampleOne < 0.0 || sampleOne > 360.0) {
        return -3.0;
    }
    sampleTwo = RPS.Heading();
    while((sampleTwo < 0.0 || sampleTwo > 360.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleTwo = RPS.Heading();
    }
    if(sampleTwo < 0.0 || sampleTwo > 360.0) {
        return -3.0;
    }
    sampleThree = RPS.Heading();
    while((sampleThree < 0.0 || sampleThree > 360.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleThree = RPS.Heading();
    }
    if(sampleThree < 0.0 || sampleThree > 360.0) {
        return -3.0;
    }
    // If RPS is returning values that are within range but vary wildly, return an error value
    if(std::abs(sampleOne - sampleTwo) > 5.0 || std::abs(sampleOne - sampleThree) > 5.0) {
        return -3.0;
    }
    // Otherwise, use valid samples to calculate an averaged value and return it
    sampleFinal = (sampleOne + sampleTwo + sampleThree) / 3.0;
    return sampleFinal;
}

// Wrap RPS.X() for some automatic error detection
float rpsSampleXCoord() {
    // Keep track of how many times we resample to avoid an incorrect value
    int extraAttempts = 0;
    float sampleOne, sampleTwo, sampleThree, sampleFinal;
    sampleOne = RPS.X();
    // If sample outside of range and we haven't exceeded the max number of
    //   incorrect values, try again
    while( (sampleOne < 0.0 || sampleOne > 36.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleOne = RPS.X();
    }
    // At this point, either the sample is good or we've exceeded the max number
    //   of incorrect values. If the sample is bad, quit trying and return an error value
    if(sampleOne < 0.0 || sampleOne > 36.0) {
        return -3.0;
    }
    sampleTwo = RPS.X();
    while((sampleTwo < 0.0 || sampleTwo > 36.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleTwo = RPS.X();
    }
    if(sampleTwo < 0.0 || sampleTwo > 36.0) {
        return -3.0;
    }
    sampleThree = RPS.X();
    while((sampleThree < 0.0 || sampleThree > 36.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleThree = RPS.X();
    }
    if(sampleThree < 0.0 || sampleThree > 36.0) {
        return -3.0;
    }
    // If RPS is returning values that are within range but vary wildly, return an error value
    if(std::abs(sampleOne - sampleTwo) > 1.0 || std::abs(sampleOne - sampleThree) > 1.0) {
        return -3.0;
    }
    // Otherwise, use valid samples to calculate an averaged value and return it
    sampleFinal = (sampleOne + sampleTwo + sampleThree) / 3.0;
    return sampleFinal;
}

// Wrap RPS.Y() for some automatic error detection
float rpsSampleYCoord() {
    // Keep track of how many times we resample to avoid an incorrect value
    int extraAttempts = 0;
    float sampleOne, sampleTwo, sampleThree, sampleFinal;
    sampleOne = RPS.Y();
    // If sample outside of range and we haven't exceeded the max number of
    //   incorrect values, try again
    while( (sampleOne < 0.0 || sampleOne > 72.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleOne = RPS.Y();
    }
    // At this point, either the sample is good or we've exceeded the max number
    //   of incorrect values. If the sample is bad, quit trying and return an error value
    if(sampleOne < 0.0 || sampleOne > 72.0) {
        return -3.0;
    }
    sampleTwo = RPS.Y();
    while((sampleTwo < 0.0 || sampleTwo > 72.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleTwo = RPS.Y();
    }
    if(sampleTwo < 0.0 || sampleTwo > 72.0) {
        return -3.0;
    }
    sampleThree = RPS.Y();
    while((sampleThree < 0.0 || sampleThree > 72.0) && extraAttempts < 3 ) {
        extraAttempts++;
        sampleThree = RPS.Y();
    }
    if(sampleThree < 0.0 || sampleThree > 72.0) {
        return -3.0;
    }
    // If RPS is returning values that are within range but vary wildly, return an error value
    if(std::abs(sampleOne - sampleTwo) > 1.0 || std::abs(sampleOne - sampleThree) > 1.0) {
        return -3.0;
    }
    // Otherwise, use valid samples to calculate an averaged value and return it
    sampleFinal = (sampleOne + sampleTwo + sampleThree) / 3.0;
    return sampleFinal;
}

////////////////////////////////////////////////////////////
