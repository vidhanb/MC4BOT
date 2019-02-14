#ifndef CONSTANTS_H
#define CONSTANTS_H

// Preprocesser macro to allow outputting defined constants
#define DEFID(x) #x
#define EXPAND_DEFID(x) DEFID(x)

// Drive motor ports
#define MOTOR_FL FEHMotor::Motor0
#define MOTOR_FR FEHMotor::Motor3
#define MOTOR_TEST EXPAND_DEFID(MOTOR_FR)

// Servo motor ports
#define SERVO_LEVER FEHServo::Servo0
#define SERVO_COIN FEHServo::Servo3
#define SERVO_CLAW FEHServo::Servo7

//TODO: adjust this value
// Motor strength percentages
#define MOTOR_PERCENT 50
#define SERVO_PERCENT 50

//TODO: adjust this voltage so as to not damage motors!
// Drive motor voltage
#define DRIVE_VOLTS 9.0

//TODO: Calibrate servos and adjust these values
// Servo max and min values
#define SERVO_LEVER_MIN 500
#define SERVO_LEVER_MAX 2500
#define SERVO_COIN_MIN 500
#define SERVO_COIN_MAX 2500
#define SERVO_CLAW_MIN 500
#define SERVO_CLAW_MAX 2500

#endif // CONSTANTS_H
