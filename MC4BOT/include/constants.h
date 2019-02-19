#ifndef CONSTANTS_H
#define CONSTANTS_H

// Drive motor ports and voltages
#define MOTOR_PORT_FL FEHMotor::Motor3
#define MOTOR_PORT_FR FEHMotor::Motor2
#define MOTOR_VOLTS 7.2

// Servo motor ports
#define SERVO_PORT_LEVER FEHServo::Servo0
#define SERVO_PORT_COIN FEHServo::Servo3
#define SERVO_PORT_CLAW FEHServo::Servo7

//TODO: adjust this value (have multiple?)
// Motor strength percentages
#define MOTOR_PERCENT 25
#define SERVO_PERCENT 25

// Servo max and min values
#define SERVO_LEVER_MIN 755
#define SERVO_LEVER_MAX 2495
//TODO: Calibrate servos and adjust these values
#define SERVO_COIN_MIN 500
#define SERVO_COIN_MAX 2500
#define SERVO_CLAW_MIN 500
#define SERVO_CLAW_MAX 2500

#endif // CONSTANTS_H
