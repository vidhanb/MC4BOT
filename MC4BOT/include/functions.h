#ifndef FUNCTIONS_H
#define FUNCTIONS_H

// Initialization
//// Setup hardware and logging for robot basic functionality
void initRobot();
//// Output how the robot was setup to the screen
void printInit();
//// Get the robot ready for official runs in competitions
void competitionStart();

// Testing various parts of software and hardware
//// Basic motor test
void testDriveStraight();
//// Basic I/O test
void testSensors();
//// Output RPS values all around course
void testRPS();
//// Experiment with the change in RPS X and Y values as
////   the heading changes
void testRPSTurnChange();
//// Visually see straightness of driving
void testDriveDistanceLong();
//// Visually see straightness of driving - proportion-based
void testDriveDistanceLongProportion();
//// Test all directions robot can move
void testDriveDirections();
//// Test all possible drive functions
void testDriveFunctions();
//// Turn repeatedly to ensure treads stay on
void testTreadTurns();
//// Make sure servo is attached and calibrated properly
void testServoRange();
//// Test active and neutral position of all servos
void testServos();

// Mechanisms
//// Engage lever servo
void flipLever();
//// Disengage lever servo
void flipLeverReset();
//// Slowly engage, then hold for >5 seconds, then slowly disengage lever servo
void rpsResetPress();
//// Slowly engage foosball mechanism
void foosballDeploy();
//// Slowly disengage foosball mechanism
void foosballRetract();
//// Lift up blocker to drop coin
void coinRelease();

// Mathematical
//// Map percentage of distance complete to a speed multiplier
////   Starts slow, max speed in middle, slows down again at end
float accelerationFunction(float ratio);

// Movement
//   Note that all angles are expressed as floats
//// Use average of encoders to move a set distance
void driveForDistance(float inches, int motorPercent, DriveDirection direction);
//// Use average of encoders to move a set distance with speeding up and slowing down
void driveForDistanceAccelMap(float inches, int motorPercent, DriveDirection direction);
//// Use average of encoders to move a set distance, using advanced encoder logic to stay straight
void driveForDistanceProportion(float inches, int motorPercent, DriveDirection direction);
//// Drive blindly for a set time
void driveForTime(float seconds, int motorPercent, DriveDirection direction);
//// Turn blindly for a set time
void turnForTime(float seconds, int motorPercent, TurnDirection direction);
//// Turn blindly for a set time, driving the motors at different rates
void turnForRatioTime(float seconds, int motorPercent, TurnDirection direction, float motorRatio);
//// Use average of encoders to turn a set angle
void turnForAngle(float targetAngle, int motorPercent, TurnDirection direction);
//// Use average of encoders to move a set distance with speeding up and slowing down
void turnForAngleAccelMap(float inches, int motorPercent, TurnDirection direction);
//// Use average of encoders to move a set distance, using advanced encoder logic to stay straight
void turnForAngleProportion(float inches, int motorPercent, TurnDirection direction);
//// Calculates and calls turn function to get to course heading
void turnToCourseAngle(float currentAngle, float targetAngle, int motorPercent);

//void turnToCourseAngle(float targetAngle, int motorPercent);

// RPS
//   Note that all angles are expressed as floats
//// Adjusts heading with pulses until RPS heading is accurate
void rpsCheckHeadingConstant(float targetHeading);
//// Adjusts X position with pulses until RPS X position is accurate
void rpsCheckXCoordConstant(float targetX);
//// Adjusts Y position with pulses until RPS Y position is accurate
void rpsCheckYCoordConstant(float targetY);
//// Adjusts heading with calculated turns until RPS heading is accurate
void rpsCheckHeadingDynamic(float targetHeading);
//// Adjusts X position with calculated drives until RPS X position is accurate
void rpsCheckXCoordDynamic(float targetX);
//// Adjusts Y position with calculated drives until RPS Y position is accurate
void rpsCheckYCoordDynamic(float targetY);
//// Wrap RPS.Heading() for some automatic error detection
float rpsSampleHeading();
//// Wrap RPS.X() for some automatic error detection
float rpsSampleXCoord();
//// Wrap RPS.Y() for some automatic error detection
float rpsSampleYCoord();
//// Adjust RPS checks for this specific course's RPS errors
void rpsSampleCourse();

// Specific events
void finalComp();

#endif // FUNCTIONS_H
