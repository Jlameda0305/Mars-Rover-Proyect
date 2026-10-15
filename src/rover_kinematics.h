#pragma once

#include <stdint.h>
#include <stdlib.h>
#include "rover_driving.h"

#ifndef RC_LOW
#define RC_LOW 1000
#endif
#ifndef RC_CENTER
#define RC_CENTER 1500
#endif
#ifndef RC_HIGH
#define RC_HIGH 2000
#endif

#ifndef SERVOMIN
#define SERVOMIN 250
#endif
#ifndef SERVOMAX
#define SERVOMAX 500
#endif
#ifndef SERVOMIN_FULL_RANGE
#define SERVOMIN_FULL_RANGE 150
#endif
#ifndef SERVOMAX_FULL_RANGE
#define SERVOMAX_FULL_RANGE 600
#endif

struct SteeringAngles {
    uint16_t front_left;
    uint16_t front_right;
    uint16_t back_left;
    uint16_t back_right;
};

struct MotorOutputs {
    uint16_t left;
    uint16_t right;
};

SteeringAngles calculate_steering(RoverMode mode, uint16_t steer_signal);
MotorOutputs calculate_motors(RoverMode mode, uint16_t throttle_signal, uint16_t steer_signal);
uint16_t filter_rc_signal(const uint16_t* samples, uint8_t sample_count);
uint16_t calculate_servo_ticks(uint8_t servo_id, uint16_t us_signal, int16_t offset);
