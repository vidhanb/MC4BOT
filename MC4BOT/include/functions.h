#ifndef FUNCTIONS_H
#define FUNCTIONS_H

void initRobot();
void printInit();
void competitionStart();

void testDriveStraight();
void testSensors();
void testRPS();
void testDriveDistanceLong();
void testDriveDirections();
void testDriveFunctions();
void testTreadTurns();
void testServoRange();
void testServos();

void flipLever();
void flipLeverReset();
void rpsResetPress();
void foosballDeploy();
void foosballRetract();
void coinRelease();

float accelerationFunction(float ratio);

void driveForDistance(float inches, MotorPower motorPercent, DriveDirection direction);
void driveForDistanceAccelMap(float inches, int motorPercent, DriveDirection direction);
void driveForDistanceProportion(float inches, int motorPercent, DriveDirection direction);
void driveForTime(float seconds, MotorPower motorPercent, DriveDirection direction);
void turnForTime(float seconds, MotorPower motorPercent, TurnDirection direction);
void turnForAngle(float targetAngle, MotorPower motorPercent, TurnDirection direction);
void turnToCourseAngle(float currentAngle, int targetAngle, MotorPower motorPercent);
void turnToCourseAngle(float targetAngle, MotorPower motorPercent);

void rpsCheckHeadingConstant(float targetHeading);
void rpsCheckXCoordConstant(float targetX);
void rpsCheckYCoordConstant(float targetY);
void rpsCheckHeadingDynamic(float targetHeading);
void rpsCheckXCoordDynamic(float targetX);
void rpsCheckYCoordDynamic(float targetY);
float rpsSampleHeading();
float rpsSampleXCoord();
float rpsSampleYCoord();

void finalComp();

#endif // FUNCTIONS_H
