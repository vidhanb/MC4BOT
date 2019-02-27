// Required proteus firmware libraries
#include <FEHMotor.h>
#include <FEHServo.h>
#include <FEHLCD.h>
#include <FEHIO.h>

// Required personal libraries
#include "include/constants.h"
#include "include/globals.h"
#include "include/functions.h"

// Init functions //////////////////////////////////////////

void initRobot() {
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );
    servoLever.SetMin(SERVO_LEVER_MIN);
    servoLever.SetMax(SERVO_LEVER_MAX);
    flipLeverReset();
    return;
}

void printInit() {
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
    LCD.Write(",Thresh:");
    LCD.Write(CDS_CELL_START_THRESH);
    LCD.WriteLine("}");

    LCD.Write("Encoder left port: ");
    LCD.WriteLine(ENCODER_LEFT_PORT);
    LCD.Write("Encoder right port: ");
    LCD.WriteLine(ENCODER_RIGHT_PORT);
    LCD.Write("Encoder counts/inch: ");
    LCD.WriteLine(ENCODER_CTS_PER_INCH);

    LCD.WriteLine("Init complete.");
    return;
}

////////////////////////////////////////////////////////////
// Test functions //////////////////////////////////////////

void testDrive() {
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

void sensorsTest() {
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
    // 90degrees difference * 0.02seconds per degree = 1.8 secons 
    while(currAngle > SERVO_LEVER_POS_ACTIVE) {
        servoLever.SetDegree(currAngle);
        currAngle--;
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
    Sleep(SERVO_LEVER_RESET_PAUSE);
    while(currAngle < SERVO_LEVER_POS_NEUTRAL) {
        servoLever.SetDegree(currAngle);
        currAngle++;
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
    return;
}

////////////////////////////////////////////////////////////
// Drive functions /////////////////////////////////////////

// TODO: possibly make these actually return something so we can do fault tolerance/error detection?
void driveForDistance(double inches, MotorPower motorPercent, DriveDirection direction) {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    double expectedEncoderCounts = inches * ENCODER_CTS_PER_INCH;
    if(direction == DirectionForward) {
        motorLeft.SetPercent(motorPercent);
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
    } else {
        motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        motorRight.SetPercent(motorPercent);
    }
    // TODO: make sure this int to double comparison works properly
    while( ( encoderLeft.Counts() + encoderRight.Counts() ) / 2.0 < expectedEncoderCounts);
    motorLeft.Stop();
    motorRight.Stop();
    return;
}

void driveForTime(double seconds, MotorPower motorPercent, DriveDirection direction) {
    if(direction == DirectionForward) {
        motorLeft.SetPercent(motorPercent);
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
    } else {
        motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        motorRight.SetPercent(motorPercent);
    }
    Sleep(seconds);
    motorLeft.Stop();
    motorRight.Stop();
    return;
}

void turnForTime(double seconds, MotorPower motorPercent, TurnDirection direction) {
    if(direction == DirectionClockwise) {
        motorLeft.SetPercent(motorPercent);
        motorRight.SetPercent(motorPercent);
    } else {
        motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
    }
    Sleep(seconds);
    motorLeft.Stop();
    motorRight.Stop();
    return;
}

void turnForAngle(int targetAngle, MotorPower motorPercent, TurnDirection direction) {
    encoderLeft.ResetCounts();
    encoderRight.ResetCounts();
    double arcLength = (targetAngle / 360) * ROBOT_TURN_CIRC;
    double expectedEncoderCounts = arcLength * ENCODER_CTS_PER_INCH;
    if(direction == DirectionClockwise) {
        motorLeft.SetPercent(motorPercent);
        motorRight.SetPercent(motorPercent);
    } else {
        motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
    }
    // TODO: make sure this int to double comparison works properly
    while( ( encoderLeft.Counts() + encoderRight.Counts() ) / 2.0 < expectedEncoderCounts);
    motorLeft.Stop();
    motorRight.Stop();
    return;
}

void turnToCourseAngle(int currentAngle, int targetAngle, MotorPower motorPercent) {
    if(targetAngle > currentAngle) {
        if( (targetAngle - currentAngle) < 180) {
            turnForAngle( (targetAngle - currentAngle) , motorPercent, DirectionClockwise );
        } else {
            turnForAngle( 360 - (targetAngle - currentAngle) , motorPercent, DirectionCounterClockwise );
        }
    } else {
        if( (currentAngle - targetAngle) < 180) {
            turnForAngle( (currentAngle - targetAngle) , motorPercent, DirectionCounterClockwise );
        } else {
            turnForAngle( 360 - (currentAngle - targetAngle) , motorPercent, DirectionClockwise );
        }
    }
    return;
}

// Use RPS to get current heading, then calculate appropriate turn to reach target
void turnToCourseAngle(int targetAngle) {
    LCD.WriteLine("Not implemented yet.");
    return;
}

//void driveUntil(int motorPower, special type for function pointer CALLBACKFUNC) {}

////////////////////////////////////////////////////////////
