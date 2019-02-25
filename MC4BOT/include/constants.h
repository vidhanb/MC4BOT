#ifndef CONSTANTS_H
#define CONSTANTS_H

#define PI 3.14159265358979

/////////////////////////////////////////////////////////////////////////

// Drive motor settings
// TODO: determine which motor requires negative to move forwards
#define MOTOR_PORT_FL FEHMotor::Motor3
#define MOTOR_PORT_FR FEHMotor::Motor2
#define MOTOR_VOLTS 7.2
#define MOTOR_SIDE_DIR_CORRECTOR -1
// #define MOTOR_SIDE_STR_CORRECTOR (if one motor is weaker than the other)

enum MotorPower {
    MotorPercentStrong = 70,
    MotorPercentMedium = 50,
    MotorPercentWeak = 30
};

enum DriveDirection : bool {
    DirectionForward,
    DirectionBackward
};

enum TurnDirection : bool {
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
//TODO: Calibrate servos and adjust these values
#define SERVO_COIN_MIN 500
#define SERVO_COIN_MAX 2500
#define SERVO_CLAW_MIN 500
#define SERVO_CLAW_MAX 2500

#define SERVO_LEVER_ITER_PAUSE 0.02
#define SERVO_LEVER_RESET_PAUSE 5.5

/////////////////////////////////////////////////////////////////////////

// Sensor settings
#define CDS_CELL_PORT FEHIO::P0_0
#define ENCODER_LEFT_PORT FEHIO::P1_0
#define ENCODER_RIGHT_PORT FEHIO::P1_2

#define CDS_CELL_START_THRESH 1.6 // volts

#define WHEEL_DIAM 1.625 // inches (including treads)
#define WHEEL_CIRC WHEEL_DIAM * PI

#define ROBOT_TURN_DIAM 9.0 //inches
#define ROBOT_TURN_CIRC ROBOT_TURN_DIAM * PI

#define ENCODER_CTS_PER_ROT 24 // for mechanical rotary encoder
#define ENCODER_CTS_PER_INCH ENCODER_CTS_PER_ROT / WHEEL_CIRC

/////////////////////////////////////////////////////////////////////////

#endif // CONSTANTS_H
