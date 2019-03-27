#ifndef CONSTANTS_H
#define CONSTANTS_H

#define PI 3.14159265358979

/////////////////////////////////////////////////////////////////////////

// Drive motor settings

#define MOTOR_PORT_FL FEHMotor::Motor3
#define MOTOR_PORT_FR FEHMotor::Motor2
#define MOTOR_VOLTS 7.2
// Note: As motors are oriented differently, right motor should be set negative
//       in order to move forward
#define MOTOR_SIDE_DIR_CORRECTOR -1
// Multiplier for right motor since it is weaker than the left motor
//  This is approximately the value that results in the robot driving straight
#define MOTOR_SIDE_STR_CORRECTOR 1.042

enum MotorPower {
    MotorPercentStrong = 70,
    MotorPercentMedium = 50,
    MotorPercentWeak = 30
};

enum DriveDirection {
    DirectionForward,
    DirectionBackward
};

enum TurnDirection {
    DirectionClockwise,
    DirectionCounterClockwise
};

/////////////////////////////////////////////////////////////////////////

// Servo motor settings
#define SERVO_PORT_LEVER FEHServo::Servo0
#define SERVO_PORT_COIN FEHServo::Servo3
#define SERVO_PORT_CLAW FEHServo::Servo7

#define SERVO_LEVER_MIN 755
#define SERVO_LEVER_MAX 2495
#define SERVO_LEVER_POS_NEUTRAL 180
#define SERVO_LEVER_POS_ACTIVE 90
#define SERVO_LEVER_ITER_PAUSE 0.02
#define SERVO_LEVER_RESET_PAUSE 5.5

#define SERVO_COIN_MIN 520
#define SERVO_COIN_MAX 2455
#define SERVO_COIN_POS_NEUTRAL 180
#define SERVO_COIN_POS_ACTIVE 85

#define SERVO_CLAW_MIN 540
#define SERVO_CLAW_MAX 2450
#define SERVO_CLAW_POS_NEUTRAL 0
#define SERVO_CLAW_POS_ACTIVE 75

/////////////////////////////////////////////////////////////////////////

// Sensor settings
#define CDS_CELL_PORT FEHIO::P2_7
#define ENCODER_LEFT_PORT FEHIO::P3_7
#define ENCODER_RIGHT_PORT FEHIO::P0_0

// Volts
#define CDS_CELL_DIV_DARK_BLUE 2.0
#define CDS_CELL_DIV_BLUE_RED 1.0

#define WHEEL_DIAM 1.625 // inches (including treads)
#define WHEEL_CIRC (WHEEL_DIAM * PI)

#define ROBOT_TURN_DIAM 10.5 //inches
#define ROBOT_TURN_CIRC (ROBOT_TURN_DIAM * PI)

#define ENCODER_CTS_PER_ROT 24 // for mechanical rotary encoder
#define ENCODER_CTS_PER_INCH ( (ENCODER_CTS_PER_ROT / WHEEL_CIRC) * 2.050 )
// Proportion which approximately results in driving straight
#define IDEAL_RTOL_ENCODER_RATIO 1.06

/////////////////////////////////////////////////////////////////////////

#define ACTION_SEP_PAUSE 0.5

#endif // CONSTANTS_H
