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

//// Setup hardware and logging for robot basic functionality
void initRobot() {
    // Setup screen
    LCD.Clear( FEHLCD::Black );
    LCD.SetFontColor( FEHLCD::White );

    // Calibrate servos and set to starting angle
    //// Claw game joystick
    g_servoLever.SetMin(SERVO_LEVER_MIN);
    g_servoLever.SetMax(SERVO_LEVER_MAX);
    flipLeverReset();
    //// Coin
    g_servoCoin.SetMin(SERVO_COIN_MIN);
    g_servoCoin.SetMax(SERVO_COIN_MAX);
    g_servoCoin.SetDegree(SERVO_COIN_POS_NEUTRAL);
    //// Foosball slider
    g_servoClaw.SetMin(SERVO_CLAW_MIN);
    g_servoClaw.SetMax(SERVO_CLAW_MAX);
    g_servoClaw.SetDegree(SERVO_CLAW_POS_NEUTRAL);

    // Encoder setup
    g_encoderLeft.ResetCounts();
    g_encoderRight.ResetCounts();

    // Connect to proper RPS course
    RPS.InitializeTouchMenu();

    // Start recording more detailed run log
    SD.OpenLog();
    SD.Printf("LOG STARTED AT TIME: %u", TimeNowSec);
    SD.Printf("ROBOT IN REGION: %c OF COURSE: %d", RPS.CurrentRegionLetter(), RPS.CurrentCourse());

    LCD.Clear();
    LCD.WriteLine("Init complete");

    SD.Printf("PRGRM-FUNC-EXIT:{Name: initRobot}");
    return;
}

//// Output how the robot was setup
void printInit() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: printInit}");
    // Print info for drive motors
    SD.Printf("INIT-HW-MOTOR:{Name: left, Port: %s, Volts: %f}", MOTOR_PORT_FL, MOTOR_VOLTS);
    SD.Printf("INIT-HW-MOTOR:{Name: right, Port: %s, Volts: %f}", MOTOR_PORT_FR, MOTOR_VOLTS);

    // Print info for servo motors
    SD.Printf("INIT-HW-SERVO:{Name: lever, Port: %s, Min: %d, Max: %d, Neutral pos: %d, Active pos: %d}", SERVO_PORT_LEVER, SERVO_LEVER_MIN, SERVO_LEVER_MAX, SERVO_LEVER_POS_NEUTRAL, SERVO_LEVER_POS_ACTIVE);
    SD.Printf("INIT-HW-SERVO:{Name: coin, Port: %s, Min: %d, Max: %d, Neutral pos: %d, Active pos: %d}", SERVO_PORT_COIN, SERVO_COIN_MIN, SERVO_COIN_MAX, SERVO_COIN_POS_NEUTRAL, SERVO_COIN_POS_ACTIVE);
    SD.Printf("INIT-HW-SERVO:{Name: claw, Port: %s, Min: %d, Max: %d, Neutral pos: %d, Active pos: %d}", SERVO_PORT_CLAW, SERVO_CLAW_MIN, SERVO_CLAW_MAX, SERVO_CLAW_POS_NEUTRAL, SERVO_CLAW_POS_ACTIVE);

    // Print info for sensors
    SD.Printf("INIT-HW-SENSOR:{Name: cds, Port: %s, Dark/blue threshold: %f, Blue/red threshold: %f}", CDS_CELL_PORT, CDS_CELL_DIV_DARK_BLUE, CDS_CELL_DIV_BLUE_RED);

    SD.Printf("INIT-HW-SENSOR:{Name: left encoder, Port: %s, Counts per inch: %f}", ENCODER_LEFT_PORT, ENCODER_CTS_PER_INCH);
    SD.Printf("INIT-HW-SENSOR:{Name: right encoder, Port: %s, Counts per inch: %f}", ENCODER_RIGHT_PORT, ENCODER_CTS_PER_INCH);

    LCD.WriteLine("Init log complete");
    SD.Printf("PRGM-FUNC-EXIT:{Name: printInit}");
    return;
}

//// Get the robot ready for official runs in competitions
void competitionStart() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: competitionStart}");
    // Prepare everything except "final action" so robot can start with minimum interaction

    rpsSampleCourse();

    LCD.Clear();
    LCD.WriteLine("");
    LCD.WriteLine("AWAITING FINAL ACTION");
    LCD.WriteLine("");
    LCD.WriteLine("PRESS ANYWHERE TO BEGIN");
    LCD.WriteLine("");
    float xTouch, yTouch;
    // Wait until screen is touched...
    while( !LCD.Touch(&xTouch, &yTouch) ) {};
    // ... then released
    while( LCD.Touch(&xTouch, &yTouch) ) {};
    LCD.Clear();
    unsigned int finalActionTime = TimeNowSec();
    SD.Printf("START-FINALACTION:{Time: %u}", finalActionTime);
    LCD.WriteLine("STARTED");
    LCD.WriteLine("AWAITING COURSE OR TIMEOUT");
    // Wait until the start light turns on, or somehow 30 seconds has passed and we missed it
    while ( (g_cdsCell.Value() > CDS_CELL_DIV_DARK_BLUE) && (TimeNowSec() < (finalActionTime + 30)) );
    SD.Printf("START-BEGIN:{Time: %u}", TimeNowSec());
    LCD.WriteLine("COMPETITION HAS BEGUN");
    // Exit this function and let the games begin!
    SD.Printf("PRGM-FUNC-EXIT:{Name: competitionStart}");
}

////////////////////////////////////////////////////////////
// Test functions //////////////////////////////////////////

//// Basic motor test
void testDriveStraight() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testDriveStraight}");
    // Just drive forward and backward at max speed and same
    // motor percentage for 3 seconds then quit
    g_motorLeft.SetPercent(100);
    g_motorRight.SetPercent(-100);
    Sleep(3.0);
    g_motorLeft.SetPercent(-100);
    g_motorRight.SetPercent(100);
    Sleep(3.0);
    g_motorLeft.Stop();
    g_motorRight.Stop();
    LCD.WriteLine("simple straight driving test done");
    SD.Printf("PRGM-FUNC-EXIT:{Name: testDriveStraight}");
    return;
}

//// Basic I/O test
void testSensors() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testSensors}");
    LCD.WriteLine("beginning sensors test");
    // Reset encoders
    g_encoderLeft.ResetCounts();
    g_encoderRight.ResetCounts();

    // Output labels
    LCD.WriteRC("CdS: ", 1, 1);
    LCD.WriteRC("Left enc: ", 2, 1);
    LCD.WriteRC("Right enc: ", 3, 1);

    while(true) {
        // Output updated values for CdS cell and encoders continually
        LCD.WriteRC(g_cdsCell.Value(), 1, 5);
        LCD.WriteRC(g_encoderLeft.Counts(), 2, 11);
        LCD.WriteRC(g_encoderRight.Counts(), 3, 12);
        Sleep(10); // 10ms
    }
    // Infinite loop will never reach here to exit
}

//// Output RPS values all around course
void testRPS() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testRPS}");
    float touch_x, touch_y;
    // Connect to proper RPS course
    RPS.InitializeTouchMenu();

    // Wait for screen to be pressed and released
    LCD.WriteLine("Press Screen to Start");
    while(!LCD.Touch(&touch_x, &touch_y));
    while(LCD.Touch(&touch_x, &touch_y));

    LCD.Clear();

    // Output labels
    LCD.WriteRC("RPS Test Program", 0, 0);
    LCD.WriteRC("X Position: ", 2, 0);
    LCD.WriteRC("Y Position: ", 3, 0);
    LCD.WriteRC("   Heading: ", 4, 0);

    while(true) {
        // Output updates values for X pos, Y pos, and heading continually
        LCD.WriteRC(RPS.X(), 2, 13);
        LCD.WriteRC(RPS.Y(), 3, 13);
        LCD.WriteRC(RPS.Heading(), 4, 13);

        Sleep(10); // 10ms
    }
    // Infinite loop will never reach here to exit
}

//// Visually see straightness of driving
void testDriveDistanceLong() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testDriveDistanceLong}");
    // Drive for over 4 ft to see how straight robot is over time
    driveForDistance(50.0, MotorPercentMedium, DirectionForward);
    // Then show encoder counts and quit
    LCD.Write("left: ");
    LCD.WriteLine(g_encoderLeft.Counts());
    LCD.Write("right: ");
    LCD.WriteLine(g_encoderRight.Counts());
    LCD.WriteLine("long distance straight test done");
    SD.Printf("PRGM-FUNC-EXIT:{Name: testDriveDistanceLong}");
}


//// Visually see straightness of driving - proportion-based
void testDriveDistanceLongProportion() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testDriveDistanceLongProportion}");
    // Drive for over 4 ft to see how straight robot is over time
    driveForDistanceProportion(50.0, MotorPercentMedium, DirectionForward);
    // Then show encoder counts and quit
    LCD.Write("left: ");
    LCD.WriteLine(g_encoderLeft.Counts());
    LCD.Write("right: ");
    LCD.WriteLine(g_encoderRight.Counts());
    LCD.WriteLine("long distance straight proportion test done");
    SD.Printf("PRGM-FUNC-EXIT:{Name: testDriveDistanceLongProportion}");
}

//// Test all directions robot can move
void testDriveDirections() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testDriveDirections}");
    // Show that motor percentage with directional multiplier works
    LCD.WriteLine(MotorPercentMedium * MOTOR_SIDE_DIR_CORRECTOR);
    LCD.WriteLine("turn right");
    turnForAngle(90.0, MotorPercentMedium, DirectionClockwise);
    Sleep(2.0);
    LCD.WriteLine("turn left");
    turnForAngle(90.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(2.0);
    LCD.WriteLine("forwards");
    driveForDistance(4.0, MotorPercentMedium, DirectionForward);
    Sleep(2.0);
    LCD.WriteLine("backwards");
    driveForDistance(4.0, MotorPercentMedium, DirectionBackward);
    Sleep(4.0);
    LCD.WriteLine("drive directions test done");
    SD.Printf("PRGM-FUNC-EXIT:{Name: testDriveDirections}");
}

//// Test all possible drive functions
void testDriveFunctions() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testDriveFunctions}");
    // Driving
    driveForTime(2.0, MotorPercentMedium, DirectionBackward);
    Sleep(1.0);
    driveForDistance(4.0, MotorPercentMedium, DirectionForward);
    Sleep(1.0);
    driveForDistanceAccelMap(4.0, MotorPercentMedium, DirectionBackward);
    Sleep(1.0);
    driveForDistanceProportion(4.0, MotorPercentMedium, DirectionForward);
    Sleep(1.0);

    // Blind turns
    turnForTime(2.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(1.0);
    turnForAngle(90.0, MotorPercentMedium, DirectionClockwise);
    Sleep(1.0);
    turnForAngleAccelMap(90.0, MotorPercentMedium, DirectionCounterClockwise);
    Sleep(1.0);
    turnForAngleProportion(90.0, MotorPercentMedium, DirectionClockwise);
    Sleep(1.0);
    turnForRatioTime(3.0, MotorPercentMedium, DirectionCounterClockwise, 0.6);
    Sleep(1.0);

    // Calculated turns
    turnToCourseAngle(90.0, 180, MotorPercentMedium);
    Sleep(1.0);
    turnToCourseAngle(90.0, 0, MotorPercentMedium);
    Sleep(1.0);

    LCD.WriteLine("drive functions test done");
    SD.Printf("PRGM-FUNC-EXIT:{Name: testDriveFunctions}");
}

//// Turn repeatedly to ensure treads stay on
void testTreadTurns() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testTreadTurns}");
    LCD.WriteLine("Entering tread turn test");
    int turns = 0;
    while(true) {
        LCD.Write("Turn ");
        LCD.WriteLine(turns);
        turnForAngle(180.0, MotorPercentMedium, DirectionClockwise);
        Sleep(1.0);
        turnForAngle(180.0, MotorPercentMedium, DirectionCounterClockwise);
        Sleep(1.0);
        turns++;
    }
    // Infinite loop will never reach here to exit
}

//// Make sure servo is attached and calibrated properly
void testServoRange() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testServoRange}");
    // Go to different positions in lever servo range, with pauses
    g_servoLever.SetDegree(180);
    LCD.WriteLine(180);
    Sleep(2.0);
    g_servoLever.SetDegree(135);
    LCD.WriteLine(135);
    Sleep(2.0);
    g_servoLever.SetDegree(90);
    LCD.WriteLine(90);
    Sleep(2.0);
    g_servoLever.SetDegree(45);
    LCD.WriteLine(45);
    Sleep(2.0);
    LCD.WriteLine("servo range test done");
    SD.Printf("PRGM-FUNC-EXIT:{Name: testServoRange}");
}

//// Test active and neutral position of all servos
void testServos() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: testServos}");
    LCD.WriteLine("servo time");
    // Uses lever servo
    rpsResetPress();
    Sleep(1.0);
    LCD.WriteLine("coin");
    coinRelease();
    Sleep(1.0);
    g_servoCoin.SetDegree(SERVO_COIN_POS_NEUTRAL);
    Sleep(1.0);
    LCD.WriteLine("claw");
    foosballDeploy();
    LCD.WriteLine("claw down");
    Sleep(1.0);
    foosballRetract();
    LCD.WriteLine("claw up");
    Sleep(1.0);
    LCD.WriteLine("servo tests done");
    SD.Printf("PRGM-FUNC-EXIT:{Name: testServos}");
}

////////////////////////////////////////////////////////////
// Mechanism functions /////////////////////////////////////

//// Engage lever servo
void flipLever() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: flipLever}");
    SD.Printf("SERVO-LEVER:{Degree: %d}", SERVO_LEVER_POS_ACTIVE);
    LCD.WriteLine("Lever flip deploying");
    g_servoLever.SetDegree(SERVO_LEVER_POS_ACTIVE);
    SD.Printf("PRGM-FUNC-EXIT:{Name: flipLever}");
    return;
}

//// Disengage lever servo
void flipLeverReset() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: flipLeverReset}");
    SD.Printf("SERVO-LEVER:{Degree: %d}", SERVO_LEVER_POS_NEUTRAL);
    LCD.WriteLine("Lever flip retracting");
    g_servoLever.SetDegree(SERVO_LEVER_POS_NEUTRAL);
    SD.Printf("PRGM-FUNC-EXIT:{Name: flipLeverReset}");
    return;
}

//// Slowly engage, then hold for >5 seconds, then slowly disengage lever servo
void rpsResetPress() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsResetPress}");
    SD.Printf("MECH-DDR:{RPS reset servo iter time: %f }", SERVO_LEVER_ITER_PAUSE);
    SD.Printf("MECH-DDR:{RPS reset button push time: %f}", SERVO_LEVER_RESET_PAUSE);
    LCD.WriteLine("Pressing RPS reset button");
    int currAngle = SERVO_LEVER_POS_NEUTRAL;
    // Slow down the servo motor's movement so that it has more torque
    while(currAngle > SERVO_LEVER_POS_ACTIVE) {
        // Move a small angle
        g_servoLever.SetDegree(currAngle);
        SD.Printf("SERVO-LEVER:{Degree: %d}", currAngle);
        currAngle-= 2;
        // Then wait a little bit
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
    Sleep(SERVO_LEVER_RESET_PAUSE);
    while(currAngle < SERVO_LEVER_POS_NEUTRAL) {
        g_servoLever.SetDegree(currAngle);
        SD.Printf("SERVO-LEVER:{Degree: %d}", currAngle);
        currAngle+= 2;
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsResetPress}");
    return;
}

//// Slowly engage foosball mechanism
void foosballDeploy() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: foosballDeploy}");
    SD.Printf("MECH-FOOSBALL:{Foosball servo iter time: %f }", SERVO_LEVER_ITER_PAUSE);
    LCD.WriteLine("Foosball arm deploying");
    int currAngle = SERVO_CLAW_POS_NEUTRAL;
    // Slow down the servo motor's movement so that it has more torque
    while(currAngle < SERVO_CLAW_POS_ACTIVE) {
        // Move a small angle
        g_servoClaw.SetDegree(currAngle);
        SD.Printf("SERVO-CLAW:{Degree: %d}", currAngle);
        currAngle+= 2;
        // Then wait a little bit
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: foosballDeploy}");
}

//// Slowly disengage foosball mechanism
void foosballRetract() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: foosballRetract}");
    SD.Printf("MECH-FOOSBALL:{Foosball servo iter time: %f }", SERVO_LEVER_ITER_PAUSE);
    LCD.WriteLine("Foosball arm retracting");
    int currAngle = SERVO_CLAW_POS_ACTIVE;
    // Slow down the servo motor's movement so that it has more torque
    while(currAngle > SERVO_CLAW_POS_NEUTRAL) {
        // Move a small angle
        g_servoClaw.SetDegree(currAngle);
        SD.Printf("SERVO-CLAW:{Degree: %d}", currAngle);
        currAngle-= 2;
        // Then wait a little bit
        Sleep(SERVO_LEVER_ITER_PAUSE);
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: foosballRetract}");
}

//// Lift up blocker to drop coin
void coinRelease() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: coinRelease}");
    LCD.WriteLine("Coin releasing");
    g_servoCoin.SetDegree(SERVO_COIN_POS_ACTIVE);
    SD.Printf("SERVO-COIN:{Degree: %d}", SERVO_COIN_POS_ACTIVE);
    SD.Printf("PRGM-FUNC-EXIT:{Name: coinRelease}");
}

////////////////////////////////////////////////////////////
// Drive functions /////////////////////////////////////////

// Note that all angles are expressed as floats

//// Map percentage of distance complete to a speed multiplier
////   Starts slow, max speed in middle, slows down again at end
float accelerationFunction(float ratio) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: accelerationFunction}");
    // Equivalent to function: -10(x-0.5)^4 + 1
    float result = (-10.0 * std::pow( (ratio - 0.5), 4.0) ) + 1.0;
    SD.Printf("ACCEL:{Input ratio: %f, Output multiplier: %f}", ratio, result);
    SD.Printf("PRGM-FUNC-EXIT:{Name: accelerationFunction}");
    return result;
}

//// Use average of encoders to move a set distance
void driveForDistance(float inches, int motorPercent, DriveDirection direction) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: driveForDistance}");
    // Reset encoders
    g_encoderLeft.ResetCounts();
    g_encoderRight.ResetCounts();
    // Calculate number of encoder counts for desired distance, then output
    float expectedEncoderCounts = inches * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionForward) {
        LCD.WriteLine("Going FW");
        // Drive left motor forwards
        g_motorLeft.SetPercent(motorPercent);
        // Drive right motor forwards, with strength adjuster and direction adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.WriteLine("Going BW");
        // Drive left motor backwards, with direction adjuster
        g_motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        // Drive right motor backwards, with strength adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_STR_CORRECTOR);
    }
    float currentEncoderCounts = 0.0;
    while( currentEncoderCounts < expectedEncoderCounts) {
        // Calculate how far we've gone for next loop
        //   Use an average of both encoders
        currentEncoderCounts = ( g_encoderLeft.Counts() + g_encoderRight.Counts() ) / 2.0;
    }
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("Left encoder: ");
    LCD.WriteLine(g_encoderLeft.Counts());
    LCD.Write("Right encoder: ");
    LCD.WriteLine(g_encoderRight.Counts());
    LCD.WriteLine("--- Drive Done ---");
    SD.Printf("PRGM-FUNC-EXIT:{Name: driveForDistance}");
    return;
}

//// Use average of encoders to move a set distance with speeding up and slowing down
void driveForDistanceAccelMap(float inches, int motorPercent, DriveDirection direction) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: driveForDistanceAccelMap}");
    // Reset encoders
    g_encoderLeft.ResetCounts();
    g_encoderRight.ResetCounts();
    // Calculate number of encoder counts for desired distance, then output
    float expectedEncoderCounts = inches * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionForward) {
        LCD.WriteLine("Going FW");
    } else {
        LCD.WriteLine("Going BW");
        // Make motor power negative to reverse both motors
        motorPercent *= MOTOR_SIDE_DIR_CORRECTOR;
    }
    // Initialize encoder values
    float currentEncoderCounts = 0.0;
    float currentDistanceRatio = 0.0;
    float currentAccelMult = 0.0;
    // Keep going until we've reached the expected counts for our distance
    while( currentEncoderCounts < expectedEncoderCounts) {
        // See what percentage of our journey we've completed so far
        currentDistanceRatio = ( currentEncoderCounts / expectedEncoderCounts );
        // Map above proportion value to a motor strength multiplier for smoother acceleration
        currentAccelMult = accelerationFunction(currentDistanceRatio);
        // Set motor percents according to above mapped value
        //// Drive left motor
        g_motorLeft.SetPercent(motorPercent * currentAccelMult);
        //// Drive right motor, with strength adjuster and direction adjuster
        g_motorRight.SetPercent(motorPercent * currentAccelMult * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( g_encoderLeft.Counts() + g_encoderRight.Counts() ) / 2.0;
    }
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("Left encoder: ");
    LCD.WriteLine(g_encoderLeft.Counts());
    LCD.Write("Right encoder: ");
    LCD.WriteLine(g_encoderRight.Counts());
    LCD.WriteLine("--- Drive Done ---");
    SD.Printf("PRGM-FUNC-EXIT:{Name: driveForDistanceAccelMap}");
    return;
}

//// Use average of encoders to move a set distance, using advanced encoder logic to stay straight
void driveForDistanceProportion(float inches, int motorPercent, DriveDirection direction) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: driveForDistanceProportion}");
    // Reset encoders
    g_encoderLeft.ResetCounts();
    g_encoderRight.ResetCounts();
    // Calculate number of encoder counts for desired distance, then output
    float expectedEncoderCounts = inches * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionForward) {
        LCD.WriteLine("Going FW");
    } else {
        LCD.WriteLine("Going BW");
        // Make motor power negative to reverse both motors
        motorPercent *= MOTOR_SIDE_DIR_CORRECTOR;
    }
    // Initialize encoder values
    float leftEncoderCounts = 0.0;
    float rightEncoderCounts = 0.0;
    float encoderProportion = 0.0;
    float currentEncoderCounts = 0.0;
    float currentDistanceRatio = 0.0;
    float currentAccelMult = 0.0;
    // Keep going until we've reached the expected counts for our distance
    while( currentEncoderCounts < expectedEncoderCounts) {
        // See what percentage of our journey we've completed so far
        currentDistanceRatio = ( currentEncoderCounts / expectedEncoderCounts );
        // Map above proportion value to a motor strength multiplier for smoother acceleration
        currentAccelMult = accelerationFunction(currentDistanceRatio);
        // Only apply proportion adjustment after encoders have had time to settle in, and only
        //   if the ratio isn't too different (which would indicate an error with the encoders)
        if( currentEncoderCounts > 20.0 && encoderProportion > 0.5 && encoderProportion < 2.0) {
            encoderProportion = (leftEncoderCounts / rightEncoderCounts) * IDEAL_RTOL_ENCODER_RATIO;
            // If encoder proportion is more than a little bit off, output this info
            if( std::abs(encoderProportion - IDEAL_RTOL_ENCODER_RATIO) > 0.1 ) {
                LCD.Write("enc ratio: ");
                LCD.WriteLine(encoderProportion);
            }
        } else {
            encoderProportion = 1.0;
        }
        // Set motor percents according to above mapped value
        //// Drive left motor
        g_motorLeft.SetPercent(motorPercent * currentAccelMult);
        //// Drive right motor, with strength adjuster
        g_motorRight.SetPercent(motorPercent * currentAccelMult * encoderProportion * MOTOR_SIDE_DIR_CORRECTOR);
        // Update counts for next loop
        leftEncoderCounts = g_encoderLeft.Counts();
        rightEncoderCounts = g_encoderRight.Counts();
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( leftEncoderCounts + rightEncoderCounts ) / 2.0;
    }
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("Left encoder: ");
    LCD.WriteLine(g_encoderLeft.Counts());
    LCD.Write("Right encoder: ");
    LCD.WriteLine(g_encoderRight.Counts());
    LCD.WriteLine("--- Drive Done ---");
    SD.Printf("PRGM-FUNC-ENTER:{Name: driveForDistanceProportion}");
    return;
}

//// Drive blindly for a set time
void driveForTime(float seconds, int motorPercent, DriveDirection direction) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: driveForTime}");
    if(direction == DirectionForward) {
        LCD.WriteLine("Going FW");
        // Drive left motor forwards
        g_motorLeft.SetPercent(motorPercent);
        // Drive right motor forwards, with strength adjuster and direction adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.WriteLine("Going BW");
        // Drive left motor backwards, with direction adjuster
        g_motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        // Drive right motor backwards, with strength adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_STR_CORRECTOR);
    }
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("Drive time: ");
    LCD.WriteLine(seconds);
    Sleep(seconds);
    LCD.WriteLine("--- Drive Done ---");
    SD.Printf("PRGM-FUNC-EXIT:{Name: driveForTime}");
    return;
}

//// Turn blindly for a set time
void turnForTime(float seconds, int motorPercent, TurnDirection direction) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: turnForTime}");
    if(direction == DirectionClockwise) {
        LCD.WriteLine("Going CW");
        // Drive left motor forwards
        g_motorLeft.SetPercent(motorPercent);
        // Drive right motor backwards, with strength adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.WriteLine("Going CNTCW");
        // Drive left motor backwards, with direction adjuster
        g_motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        // Drive right motor forwards, with strength adjuster and direction adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    }
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("Turn time: ");
    LCD.WriteLine(seconds);
    Sleep(seconds);
    LCD.WriteLine("--- Turn Done ---");
    SD.Printf("PRGM-FUNC-EXIT:{Name: turnForTime}");
    return;
}

//// Turn blindly for a set time, driving the motors at different rates
void turnForRatioTime(float seconds, int motorPercent, TurnDirection direction, float motorRatio) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: turnForRatioTime}");
    if(direction == DirectionClockwise) {
        LCD.Write("Going CW, ratio ");
        LCD.WriteLine(motorRatio);
        // Drive left motor forwards
        g_motorLeft.SetPercent(motorPercent);
        // Drive right motor, at a proportional lesser value, forwards, with strength adjuster
        g_motorRight.SetPercent(motorPercent * motorRatio * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.Write("Going CNTCW, ratio ");
        LCD.WriteLine(motorRatio);
        // Drive left motor forwards, at a proportional lesser value
        g_motorLeft.SetPercent(motorPercent * motorRatio);
        // Drive right motor, forwards, with strength adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    }
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("Turn time: ");
    LCD.WriteLine(seconds);
    Sleep(seconds);
    LCD.WriteLine("--- Turn Done ---");
    SD.Printf("PRGM-FUNC-EXIT:{Name: turnForRatioTime}");
    return;
}

//// Use average of encoders to turn a set angle
void turnForAngle(float targetAngle, int motorPercent, TurnDirection direction) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: turnForAngle}");
    // Reset encoders
    g_encoderLeft.ResetCounts();
    g_encoderRight.ResetCounts();
    // Calculate expected distance of treads based on robot geometry and output
    float arcLength = (targetAngle / 360.0) * ROBOT_TURN_CIRC;
    LCD.Write("Turn arc length: ");
    LCD.WriteLine(arcLength);   
    // Calculate number of encoder counts for desired arc length, then output
    float expectedEncoderCounts = arcLength * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionClockwise) {
        LCD.WriteLine("Going CW");
        // Drive left motor forwards
        g_motorLeft.SetPercent(motorPercent);
        // Drive right motor backwards, with strength adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_STR_CORRECTOR);
    } else {
        LCD.WriteLine("Going CNTCW");
        // Drive left motor backwards, with direction adjuster
        g_motorLeft.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR);
        // Drive right motor forwards, with strength adjuster and direction adjuster
        g_motorRight.SetPercent(motorPercent * MOTOR_SIDE_DIR_CORRECTOR * MOTOR_SIDE_STR_CORRECTOR);
    }
    // Use an average of the encoders to drive until we've reached the expected counts
    while( ( g_encoderLeft.Counts() + g_encoderRight.Counts() ) / 2.0 < expectedEncoderCounts);
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("L enc: ");
    LCD.WriteLine(g_encoderLeft.Counts());
    LCD.Write("R enc: ");
    LCD.WriteLine(g_encoderRight.Counts());
    LCD.WriteLine("--- Turn Done ---");
    SD.Printf("PRGM-FUNC-EXIT:{Name: turnForAngle}");
    return;
}

//// Use average of encoders to move a set distance with speeding up and slowing down
void turnForAngleAccelMap(float targetAngle, int motorPercent, TurnDirection direction) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: turnForAngleAccelMap}");
    // Reset encoders
    g_encoderLeft.ResetCounts();
    g_encoderRight.ResetCounts();
    // Calculate expected distance of treads based on robot geometry and output
    float arcLength = (targetAngle / 360.0) * ROBOT_TURN_CIRC;
    LCD.Write("Turn arc length: ");
    LCD.WriteLine(arcLength);   
    // Calculate number of encoder counts for desired arc length, then output
    float expectedEncoderCounts = arcLength * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionClockwise) {
        LCD.WriteLine("Going CW");
    } else {
        LCD.WriteLine("Going CNTCW");
        // Make motor power negative to reverse both motors
        motorPercent *= MOTOR_SIDE_DIR_CORRECTOR;
    }
    // Initialize encoer values
    float currentEncoderCounts = 0.0;
    float currentDistanceRatio = 0.0;
    float currentAccelMult = 0.0;
    // Keep going until we've reached the expected counts for our distance
    while( currentEncoderCounts < expectedEncoderCounts) {
        // See what percentage of our journey we've completed so far
        currentDistanceRatio = ( currentEncoderCounts / expectedEncoderCounts );
        // Map above proportion value to a motor strength multiplier for smoother acceleration
        currentAccelMult = accelerationFunction(currentDistanceRatio);
        // Set motor percents according to above mapped value
        //// Drive left motor
        g_motorLeft.SetPercent(motorPercent * currentAccelMult);
        //// Drive right motor, with strength adjuster
        g_motorRight.SetPercent(motorPercent * currentAccelMult * MOTOR_SIDE_STR_CORRECTOR);
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( g_encoderLeft.Counts() + g_encoderRight.Counts() ) / 2.0;
    }
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("L enc: ");
    LCD.WriteLine(g_encoderLeft.Counts());
    LCD.Write("R enc: ");
    LCD.WriteLine(g_encoderRight.Counts());
    LCD.WriteLine("--- Turn Done ---");
    SD.Printf("PRGM-FUNC-EXIT:{Name: turnForAngleAccelMap}");
    return;
}

//// Use average of encoders to move a set distance, using advanced encoder logic to stay straight
void turnForAngleProportion(float targetAngle, int motorPercent, TurnDirection direction) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: turnForAngleProportion}");
    // Reset encoders
    g_encoderLeft.ResetCounts();
    g_encoderRight.ResetCounts();
    // Calculate expected distance of treads based on robot geometry and output
    float arcLength = (targetAngle / 360.0) * ROBOT_TURN_CIRC;
    LCD.Write("Turn arc length: ");
    LCD.WriteLine(arcLength);
    // Calculate number of encoder counts for desired distance, then output
    float expectedEncoderCounts = arcLength * ENCODER_CTS_PER_INCH;
    LCD.Write("Exp enc counts: ");
    LCD.WriteLine(expectedEncoderCounts);
    if(direction == DirectionClockwise) {
        LCD.WriteLine("Going CW");
    } else {
        LCD.WriteLine("Going CNTCW");
        // Make motor power negative to reverse both motors
        motorPercent *= MOTOR_SIDE_DIR_CORRECTOR;
    }

    // Initialize encoeder values
    float leftEncoderCounts = 0.0;
    float rightEncoderCounts = 0.0;
    float encoderProportion = 0.0;
    float currentEncoderCounts = 0.0;
    float currentDistanceRatio = 0.0;
    float currentAccelMult = 0.0;
    // Keep going until we've reached the expected counts for our distance
    while( currentEncoderCounts < expectedEncoderCounts) {
        // See what percentage of our journey we've completed so far
        currentDistanceRatio = ( currentEncoderCounts / expectedEncoderCounts );
        // Map above proportion value to a motor strength multiplier for smoother acceleration
        currentAccelMult = accelerationFunction(currentDistanceRatio);
        // Only apply proportion adjustment after encoders have had time to settle in, and only
        //   if the ratio isn't too different (which would indicate an error with the encoders)
        if( currentEncoderCounts > 20.0 && encoderProportion > 0.5 && encoderProportion < 2.0) {
            encoderProportion = (leftEncoderCounts / rightEncoderCounts) * IDEAL_RTOL_ENCODER_RATIO;
            // If encoder proportion is more than a little bit off, output this info
            if( std::abs(encoderProportion - 1.08) > 0.1 ) {
                LCD.Write("enc ratio: ");
                LCD.WriteLine(encoderProportion);
            }
        } else {
            encoderProportion = 1.0;
        }
        // Set motor percents according to above mapped value
        //// Drive left motor
        g_motorLeft.SetPercent(motorPercent * currentAccelMult);
        //// Drive right motor, with strength adjuster
        g_motorRight.SetPercent(motorPercent * currentAccelMult * encoderProportion);
        leftEncoderCounts = g_encoderLeft.Counts();
        rightEncoderCounts = g_encoderRight.Counts();
        // Calculate how far we've gone for next loop
        currentEncoderCounts = ( leftEncoderCounts + rightEncoderCounts ) / 2.0;
    }
    // Stop moving
    g_motorLeft.Stop();
    g_motorRight.Stop();
    // Output final counts
    LCD.Write("L enc: ");
    LCD.WriteLine(g_encoderLeft.Counts());
    LCD.Write("R enc: ");
    LCD.WriteLine(g_encoderRight.Counts());
    LCD.WriteLine("--- Turn Done ---");
    SD.Printf("PRGM-FUNC-EXIT:{Name: turnForAngleProportion}");
    return;
}

//// Calculates and calls turn function to get to course heading
void turnToCourseAngle(float currentAngle, float targetAngle, int motorPercent) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: turnToCourseAngle}");
    if(currentAngle > targetAngle) {
        // If we're past the angle, but by less than a half rotation, turn clockwise to reach it
        if( (currentAngle - targetAngle) < 180.0) {
            LCD.Write("Turn deg: ");
            // Output # degrees to turn, then do it
            LCD.WriteLine( (currentAngle - targetAngle) );
            turnForAngle( (currentAngle - targetAngle) , motorPercent, DirectionClockwise );
        } else {
            // If we're past the angle, but by more than a half rotation, it's shorter to turn counterclockwise to reach it
            LCD.Write("Turn deg: ");
            LCD.WriteLine( 360.0 - (currentAngle - targetAngle) );
            turnForAngle( 360.0 - (currentAngle - targetAngle) , motorPercent, DirectionCounterClockwise );
        }
    } else {
        // If we're short of the angle, and by less than a half rotatoin, turn counterclockwise to reach it
        if( (targetAngle - currentAngle) < 180.0) {
            LCD.Write("Turn deg: ");
            LCD.WriteLine( (targetAngle - currentAngle) );
            turnForAngle( (targetAngle - currentAngle) , motorPercent, DirectionCounterClockwise );
        } else {
            // If we're short of the angle, and by more than a half rotation, it's shorter to turn clockwise to reach it
            LCD.Write("Turn deg: ");
            LCD.WriteLine( 360.0 - (targetAngle - currentAngle) );
            turnForAngle( 360.0 - (targetAngle - currentAngle) , motorPercent, DirectionClockwise );
        }
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: turnToCourseAngle}");
    return;
}

// DEPRECATED - BREAKS IN A BAD WAY WHEN RPS IS BROKEN/INACCURATE
// Use RPS to get current heading, then calculate appropriate turn to reach target
/*
void turnToCourseAngle(float targetAngle, int motorPercent) {
    // Use RPS to get the current heading, then pass the two angles to the other turnToCourseAngle function
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    float currentAngle = currentHeading;
    turnToCourseAngle(currentAngle, targetAngle, motorPercent);
    return;
}
*/

//void driveUntil(int motorPercent, special type for function pointer CALLBACKFUNC) {}


////////////////////////////////////////////////////////////
// RPS functions ///////////////////////////////////////////

//// Adjusts heading with pulses until RPS heading is accurate
void rpsCheckHeadingConstant(float targetHeading) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsCheckHeadingConstant}");
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    // Calculate difference between where we are and where we need to be
    float headingDifference = currentHeading - targetHeading;
    // Loop until we're within desired accuracy
    while( std::abs(headingDifference) > 3.0) {
        LCD.Write("Target angle diff: ");
        LCD.WriteLine( headingDifference );
        if(headingDifference > 0.0 && headingDifference < 180.0) {
            // If we're past desired target, but not by more than a half rotation, pulse CW
            turnForAngle(1.0, MotorPercentWeak, DirectionClockwise);
        } else if(headingDifference < 0.0 && headingDifference > -180.0) {
            // If we're behind desired target, but by less than a half rotation, pulse CCW
            turnForAngle(1.0, MotorPercentWeak, DirectionCounterClockwise);
        } else if(headingDifference > 180.0) {
            // If we're past desired target by more than a half rotation, it's shorter to pulse CCW
            turnForAngle(1.0, MotorPercentWeak, DirectionCounterClockwise);
        } else if(headingDifference < -180.0) {
            // If we're behind desired target by more than a half rotation, it's shorter to pulse CW
            turnForAngle(1.0, MotorPercentWeak, DirectionClockwise);
        }
        // Wait for robot to stop moving and RPS position to catch up
        Sleep(ACTION_SEP_PAUSE);
        // Update values for next loop
        currentHeading = rpsSampleHeading();
        if(currentHeading < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
        headingDifference = currentHeading - targetHeading;
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsCheckHeadingConstant}");
    return;
}

//// Adjusts X position with pulses until RPS X position is accurate
void rpsCheckXCoordConstant(float targetX) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsCheckXCoordConstant}");
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    // First, use heading to figure out whether we're facing east (towards plus X) or west
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
    // Loop until we're within desired accuracy
    while( std::abs(currentXCoord - targetX) > 1.0 ) {
        if(currentXCoord < targetX && facingPlus) {
            // If we're west of target and facing east, pulse forwards
            driveForDistance(0.25, MotorPercentWeak, DirectionForward);
        } else if(currentXCoord > targetX && facingPlus) {
            // If we're east of target and facing east, pulse backwards
            driveForDistance(0.25, MotorPercentWeak, DirectionBackward);
        } else if(currentXCoord < targetX && !facingPlus) {
            // If we're west of target and facing west, pulse backwards
            driveForDistance(0.25, MotorPercentWeak, DirectionBackward);
        } else if(currentXCoord > targetX && !facingPlus) {
            // If we're east of target and facing west, pulse forwards
            driveForDistance(0.25, MotorPercentWeak, DirectionForward);
        }
        // Wait for robot to stop moving and RPS position to catch up
        Sleep(ACTION_SEP_PAUSE);
        // Update values for next loop
        currentXCoord = rpsSampleXCoord();
        if(currentXCoord < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsCheckXCoordConstant}");
    return;
}

//// Adjusts Y position with pulses until RPS Y position is accurate
void rpsCheckYCoordConstant(float targetY) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsCheckYCoordConstant}");
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    // First, use heading to figure out whether we're facing north (towards plus Y) or south
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
    // Loop until we're within desired accuracy
    while( std::abs(currentYCoord - targetY) > 1.0 ) {
        if(currentYCoord < targetY && facingPlus) {
            // If we're south of target and facing north, pulse fowards
            driveForDistance(0.25, MotorPercentWeak, DirectionForward);
        } else if(currentYCoord > targetY && facingPlus) {
            // If we're north of target and facing north, pulse backwards
            driveForDistance(0.25, MotorPercentWeak, DirectionBackward);
        } else if(currentYCoord < targetY && !facingPlus) {
            // If we're south of target and facing south, pulse backwards
            driveForDistance(0.25, MotorPercentWeak, DirectionBackward);
        } else if(currentYCoord > targetY && !facingPlus) {
            // If we're north of target and facing south, pulse forwards
            driveForDistance(0.25, MotorPercentWeak, DirectionForward);
        }
        // Wait for robot to stop moving and RPS position to catch up
        Sleep(ACTION_SEP_PAUSE);
        // Update values for next loop
        currentYCoord = rpsSampleYCoord();
        if(currentYCoord < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsCheckYCoordConstant}");
    return;
}

//// Adjusts heading with calculated turns until RPS heading is accurate
void rpsCheckHeadingDynamic(float targetHeading) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsCheckHeadingDynamic}");
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    // Calculate difference between where we are and where we need to be
    float headingDifference = currentHeading - targetHeading;
    // Loop until we're within desired accuracy
    while( std::abs(headingDifference) > 3.0) {
        LCD.Write("Target angle diff: ");
        LCD.WriteLine( headingDifference );
        LCD.Write("current: ");
        LCD.WriteLine(currentHeading);
        LCD.Write("target: ");
        LCD.WriteLine(targetHeading);
        // Note that turnToCourseAngle uses turns without acceleration or proportion adjustment internally
        turnToCourseAngle(currentHeading, targetHeading, MotorPercentMedium);
        // Wait for robot to stop moving and RPS position to catch up
        Sleep(ACTION_SEP_PAUSE);
        // Update values for next loop
        currentHeading = rpsSampleHeading();
        if(currentHeading < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
        headingDifference = currentHeading - targetHeading;
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsCheckHeadingDynamic}");
    return;
}

//// Adjusts X position with calculated drives until RPS X position is accurate
void rpsCheckXCoordDynamic(float targetX) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsCheckXCoordDynamic}");
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    // First, use heading to figure out whether we're facing east (towards plus X) or west
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
    // Loop until we're within desired accuracy
    while( std::abs(currentXCoord - targetX) > 1.0 ) {
        if(currentXCoord < targetX && facingPlus) {
            // If we're west of target and facing east, pulse forwards
            driveForDistanceProportion(positionXDifference, MotorPercentMedium, DirectionForward);
        } else if(currentXCoord > targetX && facingPlus) {
            // If we're east of target and facing east, pulse backwards
            driveForDistanceProportion(positionXDifference, MotorPercentMedium, DirectionBackward);
        } else if(currentXCoord < targetX && !facingPlus) {
            // If we're west of target and facing west, pulse backwards
            driveForDistanceProportion(positionXDifference, MotorPercentMedium, DirectionBackward);
        } else if(currentXCoord > targetX && !facingPlus) {
            // If we're east of target and facing west, pulse forwards
            driveForDistanceProportion(positionXDifference, MotorPercentMedium, DirectionForward);
        }
        // Wait for robot to stop moving and RPS position to catch up
        Sleep(ACTION_SEP_PAUSE);
        // Update values for next loop
        currentXCoord = rpsSampleXCoord();
        if(currentXCoord < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
        positionXDifference = std::abs(currentXCoord - targetX);
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsCheckXCoordDynamic}");
    return;
}

//// Adjusts Y position with calculated drives until RPS Y position is accurate
void rpsCheckYCoordDynamic(float targetY) {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsCheckYCoordDynamic}");
    float currentHeading = rpsSampleHeading();
    if(currentHeading < 0.0) {
        // RPS is having issues right now, we can't perform this function accurately, so just quit
        return;
    }
    // First, use heading to figure out whether we're facing north (towards plus Y) or south
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
    // Loop until we're within desired accuracy
    while( std::abs(currentYCoord - targetY) > 1.0 ) {
        if(currentYCoord < targetY && facingPlus) {
            // If we're south of target and facing north, pulse fowards
            driveForDistanceProportion(positionYDifference, MotorPercentMedium, DirectionForward);
        } else if(currentYCoord > targetY && facingPlus) {
            // If we're north of target and facing north, pulse backwards
            driveForDistanceProportion(positionYDifference, MotorPercentMedium, DirectionBackward);
        } else if(currentYCoord < targetY && !facingPlus) {
            // If we're south of target and facing south, pulse backwards
            driveForDistanceProportion(positionYDifference, MotorPercentMedium, DirectionBackward);
        } else if(currentYCoord > targetY && !facingPlus) {
            // If we're north of target and facing south, pulse forwards
            driveForDistanceProportion(positionYDifference, MotorPercentMedium, DirectionForward);
        }
        // Wait for robot to stop moving and RPS position to catch up
        Sleep(ACTION_SEP_PAUSE);
        // Update values for next loop
        currentYCoord = rpsSampleYCoord();
        if(currentYCoord < 0.0) {
            // RPS is having issues right now, we can't perform this function accurately, so just quit
            return;
        }
        positionYDifference = std::abs(currentYCoord - targetY);
    }
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsCheckYCoordDynamic}");
    return;
}

//// Wrap RPS.Heading() for some automatic error detection
float rpsSampleHeading() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsSampleHeading}");
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
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsSampleHeading}");
    return sampleFinal;
}

//// Wrap RPS.X() for some automatic error detection
float rpsSampleXCoord() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsSampleXCoord}");
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
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsSampleXCoord}");
    return sampleFinal;
}

//// Wrap RPS.Y() for some automatic error detection
float rpsSampleYCoord() {
    SD.Printf("PRGM-FUNC-ENTER:{Name: rpsSampleYCoord}");
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
    SD.Printf("PRGM-FUNC-EXIT:{Name: rpsSampleYCoord}");
    return sampleFinal;
}

// Adjust RPS checks for this specific course's RPS errors
void rpsSampleCourse() {
    // Initialize sampling values
    float sampleXDDR, sampleYDDR, sampleXCoin, sampleYCoin;
    float touch_x, touch_y;
    // Output instructions
    LCD.Clear();
    LCD.WriteLine("Move robot to red DDR button");
    LCD.WriteLine("Make sure front is pushing red button, lever will push down rps reset button, and current rps heading is 177.0");
    LCD.WriteLine("Current RPS: ");
    LCD.WriteRC("Press screen to set point", 13, 2);
    // Output heading information while waiting for screen to be pressed
    while(!LCD.Touch(&touch_x, &touch_y)) {
        LCD.WriteRC(rpsSampleHeading(), 10, 14);
    }
    // Wait for release to debounce push
    while(LCD.Touch(&touch_x, &touch_y)) {
        LCD.WriteRC(rpsSampleHeading(), 10, 14);
    }
    // Sample coordinates
    sampleXDDR = rpsSampleXCoord();
    sampleYDDR = rpsSampleYCoord();

    // Output instructions for next point
    LCD.Clear();
    LCD.WriteLine("Move robot to upper-level coin deposit spot");
    LCD.WriteLine("Make sure coin is centered on slot, robot is backed up completely to machine, and current rps heading is 357.0");
    LCD.WriteLine("Current RPS: ");
    LCD.WriteRC("Press screen to set point", 13, 2);
    // Output heading information while waiting for screen to be pressed
    while(!LCD.Touch(&touch_x, &touch_y)) {
        LCD.WriteRC(rpsSampleHeading, 10, 14);
    }
    // Wait for release to debounce push
    while(LCD.Touch(&touch_x, &touch_y)) {
        LCD.WriteRC(rpsSampleHeading(), 10, 14);
    }
    // Sample coordinates
    sampleXCoin = rpsSampleXCoord();
    sampleYCoin = rpsSampleYCoord();

    // Normalize samples to expected values
    sampleXDDR -= 24.8;
    sampleYDDR -= 9.1;
    sampleXCoin -= 15.0;
    sampleYCoin -= 44.3;

    // Calculate coordinate translations for this course
    g_adjustX = ( (sampleXDDR + sampleXCoin) / 2.0 );
    g_adjustY = ( (sampleYDDR + sampleYCoin) / 2.0 );
}

////////////////////////////////////////////////////////////
