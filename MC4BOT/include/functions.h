#ifndef FUNCTIONS_H
#define FUNCTIONS_H

void initRobot();
void printInit();

void testDrive();
void testSensors();
void testRPS();
void testDistance();
void testDirections();
void testFunctions();
void testTreadTurns();
void testServos();

void flipLever();
void flipLeverReset();

void driveForDistance(double inches, MotorPower motorPercent, DriveDirection direction);
void driveForTime(double seconds, MotorPower motorPercent, DriveDirection direction);
void turnForTime(double seconds, MotorPower motorPercent, TurnDirection direction);
void turnForAngle(int targetAngle, MotorPower motorPercent, TurnDirection direction);
void turnToCourseAngle(int currentAngle, int targetAngle, MotorPower motorPercent);
void turnToCourseAngle(int targetAngle, MotorPower motorPercent);

void finalComp();

#endif // FUNCTIONS_H
