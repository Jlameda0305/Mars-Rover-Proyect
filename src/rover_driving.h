#pragma once
#include <stdint.h>

typedef enum RoverMode {
  DRIVE_TURN_NORMAL,
  DRIVE_TURN_SPIN
} RoverMode;

void rover_driving_init(void);
void rover_driving_set_drive_mode(RoverMode mode);
void rover_driving_move(uint16_t signal);
void rover_driving_steer(uint16_t signal);