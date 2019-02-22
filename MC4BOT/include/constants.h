#ifndef CONSTANTS_H
#define CONSTANTS_H

// Drive motor ports, voltages, and percentages
// Left - negative goes forward
#define MOTOR_PORT_FL FEHMotor::Motor3
#define MOTOR_PORT_FR FEHMotor::Motor2
#define MOTOR_VOLTS 7.2
#define MOTOR_PERCENT_STRONG 70
#define MOTOR_PERCENT_MEDIUM 50
#define MOTOR_PERCENT_WEAK 30

// Servo motor ports
#define SERVO_PORT_LEVER FEHServo::Servo0
#define SERVO_PORT_COIN FEHServo::Servo3
#define SERVO_PORT_CLAW FEHServo::Servo7

// Sensor ports
#define CDS_CELL_PORT FEHIO::P0_0
#define ENCODER_LEFT_PORT FEHIO::P1_0
#define ENCODER_RIGHT_PORT FEHIO::P1_2

//TODO: adjust this value (have multiple?)
// Motor strength percentages
#define MOTOR_PERCENT 25
#define SERVO_PERCENT 25

// Servo calibration values
#define SERVO_LEVER_MIN 755
#define SERVO_LEVER_MAX 2495
#define SERVO_LEVER_POS_NEUTRAL 180
#define SERVO_LEVER_POS_ACTIVE 90
//TODO: Calibrate servos and adjust these values
#define SERVO_COIN_MIN 500
#define SERVO_COIN_MAX 2500
#define SERVO_CLAW_MIN 500
#define SERVO_CLAW_MAX 2500

#endif // CONSTANTS_H
