#ifndef FUNCTIONS_H
#define FUNCTIONS_H

// Initialization
void initRobot();
void printInit();
void competitionStart();

// Testing various parts of software and hardware
void testDriveStraight();
void testSensors();
void testRPS();
void testDriveDistanceLong();
void testDriveDirections();
void testDriveFunctions();
void testTreadTurns();
void testServoRange();
void testServos();

// Mechanisms
void flipLever();
void flipLeverReset();
void rpsResetPress();
void foosballDeploy();
void foosballRetract();
void coinRelease();

// Mathematical
float accelerationFunction(float ratio);

// Movement
//   Note that all angles are expressed as floats
void driveForDistance(float inches, int motorPercent, DriveDirection direction);
void driveForDistanceAccelMap(float inches, int motorPercent, DriveDirection direction);
void driveForDistanceProportion(float inches, int motorPercent, DriveDirection direction);
void driveForTime(float seconds, int motorPercent, DriveDirection direction);
void turnForTime(float seconds, int motorPercent, TurnDirection direction);
void turnForRatioTime(float seconds, int motorPercent, TurnDirection direction, float motorRatio);
void turnForAngle(float targetAngle, int motorPercent, TurnDirection direction);
void turnForAngleAccelMap(float inches, int motorPercent, TurnDirection direction);
void turnForAngleProportion(float inches, int motorPercent, TurnDirection direction);
void turnToCourseAngle(float currentAngle, float targetAngle, int motorPercent);
void turnToCourseAngle(float targetAngle, int motorPercent);

// RPS
//   Note that all angles are expressed as floats
void rpsCheckHeadingConstant(float targetHeading);
void rpsCheckXCoordConstant(float targetX);
void rpsCheckYCoordConstant(float targetY);
void rpsCheckHeadingDynamic(float targetHeading);
void rpsCheckXCoordDynamic(float targetX);
void rpsCheckYCoordDynamic(float targetY);
float rpsSampleHeading();
float rpsSampleXCoord();
float rpsSampleYCoord();

// Specific events
void finalComp();

#endif // FUNCTIONS_H
