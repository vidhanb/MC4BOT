#ifndef FUNCTIONS_H
#define FUNCTIONS_H

void initRobot();
void printInit();
void testDrive();
void sensorsTest();
void flipLever();
void flipLeverReset();

// TODO: figure out where to put function descriptions
void driveForDistance(double inches, MotorPower motorPercent, DriveDirection direction);
void driveForTime(double seconds, MotorPower motorPercent, DriveDirection direction);
void turnForTime(double seconds, MotorPower motorPercent, TurnDirection direction);
void turnForAngle(int targetAngle, MotorPower motorPercent, TurnDirection direction);
void turnToCourseAngle(int currentAngle, int targetAngle, MotorPower motorPercent);
void turnToCourseAngle(int targetAngle);

void pt02();

#endif // FUNCTIONS_H
