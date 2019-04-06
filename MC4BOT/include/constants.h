#ifndef CONSTANTS_H
#define CONSTANTS_H

// Used for calculating turn distance
#define PI 3.14159265358979

/////////////////////////////////////////////////////////////////////////

// Drive motor settings

//// Hardware settings
#define MOTOR_PORT_FL FEHMotor::Motor3
#define MOTOR_PORT_FR FEHMotor::Motor2
#define MOTOR_VOLTS 7.2

//// Note: As motors are oriented differently, right motor should be set negative
////       in order to move forward
#define MOTOR_SIDE_DIR_CORRECTOR -1
//// Multiplier for right motor since it is weaker than the left motor
////   This is approximately the value that results in the robot driving straight
#define MOTOR_SIDE_STR_CORRECTOR 1.042

//// Distance calculations
#define WHEEL_DIAM 1.625 // inches (including treads)
#define WHEEL_CIRC (WHEEL_DIAM * PI)

#define ROBOT_TURN_DIAM 10.3 //inches
#define ROBOT_TURN_CIRC (ROBOT_TURN_DIAM * PI)

//// Just some recommended levels
//// All drive functions accept int parameters, so the full range of percentages
////   is still allowed
enum MotorPower {
    MotorPercentStrong = 70,
    MotorPercentMedium = 50,
    MotorPercentWeak = 30
};

//// Makes function calls more readable
enum DriveDirection {
    DirectionForward,
    DirectionBackward
};

//// Makes function calls more readable
enum TurnDirection {
    DirectionClockwise,
    DirectionCounterClockwise
};

/////////////////////////////////////////////////////////////////////////

// Servo motor settings

//// Ports
#define SERVO_PORT_LEVER FEHServo::Servo0
#define SERVO_PORT_COIN FEHServo::Servo3
#define SERVO_PORT_CLAW FEHServo::Servo7

//// Calibration constants and timings
////// Lever
#define SERVO_LEVER_MIN 760
#define SERVO_LEVER_MAX 2480
#define SERVO_LEVER_POS_NEUTRAL 170
#define SERVO_LEVER_POS_ACTIVE 70
#define SERVO_LEVER_RED_ACTIVE 71
#define SERVO_LEVER_BLUE_ACTIVE 61
#define SERVO_LEVER_ITER_PAUSE 0.01
#define SERVO_LEVER_RESET_PAUSE 5.5

////// Coin
#define SERVO_COIN_MIN 520
#define SERVO_COIN_MAX 2450
#define SERVO_COIN_POS_NEUTRAL 180
#define SERVO_COIN_POS_ACTIVE 85

////// Foosball
#define SERVO_CLAW_MIN 530
#define SERVO_CLAW_MAX 2430
#define SERVO_CLAW_POS_NEUTRAL 0
#define SERVO_CLAW_POS_ACTIVE 90

/////////////////////////////////////////////////////////////////////////

// Sensor settings

//// Ports
#define CDS_CELL_PORT FEHIO::P2_7
#define ENCODER_LEFT_PORT FEHIO::P1_0
#define ENCODER_RIGHT_PORT FEHIO::P0_0

//// CdS Volts
#define CDS_CELL_DIV_DARK_BLUE 1.7
#define CDS_CELL_DIV_BLUE_RED 1.0

//// Encoders
#define ENCODER_CTS_PER_ROT 48 // for mechanical rotary encoder
#define ENCODER_CTS_PER_INCH ( (ENCODER_CTS_PER_ROT / WHEEL_CIRC) )
////// Proportion which approximately results in driving straight
#define IDEAL_RTOL_ENCODER_RATIO 1.08

/////////////////////////////////////////////////////////////////////////

// Slowdown between actions to allow RPS to update
#define ACTION_SEP_PAUSE 0.4

#endif // CONSTANTS_H
