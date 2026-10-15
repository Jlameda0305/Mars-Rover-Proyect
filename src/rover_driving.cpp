#include "esp_system.h"
#include "rover_driving.h"
#include "rover_kinematics.h"
#include "rover_config.h"
#include "rover_servo.h"
#include <ESP32Servo.h>

enum MotorDirection {
  MOTOR_IDLE,
  MOTOR_FORWARD,
  MOTOR_BACKWARD
};

static Servo motors_left;
static Servo motors_right;

static MotorDirection motor_state = MOTOR_IDLE;
static RoverMode current_rover_mode = DRIVE_TURN_NORMAL;

void rover_driving_init(void)
{
    motors_left.attach(ROVER_MOTORS_LEFT_PIN);
    motors_left.writeMicroseconds(RC_CENTER);
    motors_right.attach(ROVER_MOTORS_RIGHT_PIN);
    motors_right.writeMicroseconds(RC_CENTER);

    // Steering servos
    rover_servo_write(SERVO_FRONT_LEFT, RC_CENTER);
    rover_servo_write(SERVO_FRONT_RIGHT, RC_CENTER);
    rover_servo_write(SERVO_BACK_LEFT, RC_CENTER);
    rover_servo_write(SERVO_BACK_RIGHT, RC_CENTER);
}

static void setMotorInReverseMode(Servo motor) {
  motor.writeMicroseconds(1400);
  delay(100);
  motor.writeMicroseconds(RC_CENTER);
  delay(100);
}

void rover_driving_set_drive_mode(RoverMode mode)
{
    current_rover_mode = mode;
}

void rover_driving_move(uint16_t signal) {
  MotorOutputs outputs;
  if (current_rover_mode == DRIVE_TURN_NORMAL) {
    outputs = calculate_motors(DRIVE_TURN_NORMAL, signal, RC_CENTER);
    if (outputs.left < RC_CENTER && outputs.left > 0) {
      if (motor_state == MOTOR_IDLE) {
        setMotorInReverseMode(motors_left);
        setMotorInReverseMode(motors_right);
        motor_state = MOTOR_BACKWARD;
      }
    } else if (outputs.left > RC_CENTER) {
      motor_state = MOTOR_FORWARD;
    } else {
      motor_state = MOTOR_IDLE;
    }
  } else if (current_rover_mode == DRIVE_TURN_SPIN) {
    // In spin mode, signal is steering/volante signal
    outputs = calculate_motors(DRIVE_TURN_SPIN, RC_CENTER, signal);
    if (outputs.left < RC_CENTER && outputs.left > 0) {
      if (motor_state == MOTOR_IDLE) {
        setMotorInReverseMode(motors_left);
        motor_state = MOTOR_FORWARD;
      }
    } else if (outputs.right < RC_CENTER && outputs.right > 0) {
      if (motor_state == MOTOR_IDLE) {
        setMotorInReverseMode(motors_right);
        motor_state = MOTOR_FORWARD;
      }
    } else {
      motor_state = MOTOR_IDLE;
    }
  }

  motors_left.writeMicroseconds(outputs.left);
  motors_right.writeMicroseconds(outputs.right);
}

void rover_driving_steer(uint16_t signal) {
  SteeringAngles angles = calculate_steering(current_rover_mode, signal);
  rover_servo_write(SERVO_FRONT_LEFT, angles.front_left);
  rover_servo_write(SERVO_FRONT_RIGHT, angles.front_right);
  rover_servo_write(SERVO_BACK_LEFT, angles.back_left);
  rover_servo_write(SERVO_BACK_RIGHT, angles.back_right);
}
