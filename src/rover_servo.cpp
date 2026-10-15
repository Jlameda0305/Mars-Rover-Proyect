#include <Arduino.h>
#include "rover_servo.h"
#include "rover_kinematics.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ESP32Servo.h"
#include "rover_config.h"
#include "esp_timer.h"
#include "esp_err.h"
#include "assert.h"

#define SERVO_UPDATE_INTERVAL_MS  10

typedef enum ServoDirection {
  DIRECTION_POSITIVE,
  DIRECTION_NEGATIVE
} ServoDirection;

typedef struct ServoState {
  uint16_t      min;
  uint16_t      max;
  uint16_t      current_pos;
  uint16_t      start_pos;
  uint16_t      end_pos;
  uint8_t       speed;
  uint32_t      num_moves;
  uint32_t      steps_per_move;
  uint32_t      num_moves_finished;
  ServoDirection direction;
  bool          isPaused;
  xList         path;
} ServoState;

static void update_servo_positions_timer_cb(void* arg);
static void reset_moving_servo(RoverServoId servoId);
static int16_t get_servo_offset(RoverServoId servoId);

static Servo steering_servos[SERVO_LAST];
static xSemaphoreHandle position_update_sem_handle = NULL;
static bool isInitialized = false;
static ServoState servo_states[SERVO_LAST];

void rover_servo_init(void) {
  TaskHandle_t xHandle = NULL;
  if (!isInitialized) {
    isInitialized = true;
    memset(&servo_states, 0, sizeof(servo_states));
    for (uint8_t i = 0; i < SERVO_LAST; i++) {
      servo_states[i].current_pos = RC_CENTER;
    }

    steering_servos[SERVO_FRONT_LEFT].attach(ROVER_SERVO_FRONT_LEFT_PIN);
    steering_servos[SERVO_FRONT_RIGHT].attach(ROVER_SERVO_FRONT_RIGHT_PIN);
    steering_servos[SERVO_BACK_LEFT].attach(ROVER_SERVO_BACK_LEFT_PIN);
    steering_servos[SERVO_BACK_RIGHT].attach(ROVER_SERVO_BACK_RIGHT_PIN);

    for (uint8_t i = 0; i < SERVO_LAST; i++) {
      steering_servos[i].writeMicroseconds(RC_CENTER);
    }

    position_update_sem_handle = xSemaphoreCreateBinary();
    assert(position_update_sem_handle);
    xSemaphoreGive(position_update_sem_handle);

    assert(xTaskCreate(update_servo_positions_timer_cb, "ServoUpdater", 2048, NULL, tskIDLE_PRIORITY, &xHandle) == pdPASS);
  }
}

static int16_t get_servo_offset(RoverServoId servoId) {
  switch (servoId) {
    case SERVO_FRONT_LEFT:
      return SERVO_FRONT_LEFT_OFFSET;
    case SERVO_FRONT_RIGHT:
      return SERVO_FRONT_RIGHT_OFFSET;
    case SERVO_BACK_LEFT:
      return SERVO_BACK_LEFT_OFFSET;
    case SERVO_BACK_RIGHT:
      return SERVO_BACK_RIGHT_OFFSET;
    default:
      return 0;
  }
}

void rover_servo_write(RoverServoId servoId, uint16_t us, bool full_range)
{
  (void)full_range;
  assert(servoId < SERVO_LAST);
  assert(us >= RC_LOW);
  assert(us <= RC_HIGH);
  xSemaphoreTake(position_update_sem_handle, portMAX_DELAY);

  int16_t offset = get_servo_offset(servoId);
  int32_t target_us = static_cast<int32_t>(us) + offset;
  if (target_us < 600) {
    target_us = 600;
  } else if (target_us > 2400) {
    target_us = 2400;
  }

  reset_moving_servo(servoId);
  servo_states[servoId].current_pos = us;

  steering_servos[servoId].writeMicroseconds(static_cast<uint16_t>(target_us));
  xSemaphoreGive(position_update_sem_handle);
}

void rover_servo_move(RoverServoId servoId, uint16_t pos, uint8_t speed)
{
  assert(servoId < SERVO_LAST);
  ServoState* servo_state = &servo_states[servoId];
  uint32_t steps_to_move;
  uint32_t internal_speed;
  assert(pos >= RC_LOW);
  assert(pos <= RC_HIGH);
  assert(speed >= SERVO_MIN_SPEED);
  assert(speed <= SERVO_MAX_SPEED);
  internal_speed = SERVO_MAX_SPEED + 1 - speed; // Speed is inverted internally 1 fastest, 10 slowest

  if (pos == servo_state->current_pos || (pos == servo_state->end_pos && internal_speed == servo_state->speed)) return;
  xSemaphoreTake(position_update_sem_handle, portMAX_DELAY);
  servo_state->isPaused = false;
  servo_state->end_pos = pos;
  servo_state->start_pos = servo_state->current_pos;
  servo_state->speed = internal_speed;
  steps_to_move = abs((int32_t)servo_state->end_pos - (int32_t)servo_state->current_pos);
  servo_state->num_moves = (steps_to_move / SERVO_UPDATE_INTERVAL_MS);
  if (servo_state->num_moves == 0) {
    servo_state->num_moves = 1;
  }
  servo_state->steps_per_move = steps_to_move / servo_state->num_moves;
  servo_state->num_moves_finished = 0;
  servo_state->direction = (servo_state->end_pos > servo_state->current_pos) ? DIRECTION_POSITIVE : DIRECTION_NEGATIVE;
  xSemaphoreGive(position_update_sem_handle);
}

void rover_servo_pause(RoverServoId servoId)
{
  assert(servoId < SERVO_LAST);
  servo_states[servoId].isPaused = true;
}

void rover_servo_resume(RoverServoId servoId)
{
  assert(servoId < SERVO_LAST);
  servo_states[servoId].isPaused = false;
}

static void update_servo_positions_timer_cb(void* arg)
{
  while (true) {
    ServoState* axis;
    uint32_t next_servo_pos;
    for (uint8_t i = 0; i < SERVO_LAST; i++) {
      axis = &servo_states[i];
      if (axis->num_moves == 0) continue;
      if (axis->isPaused) continue;
      xSemaphoreTake(position_update_sem_handle, portMAX_DELAY);
      axis->num_moves_finished++;
      if (axis->num_moves_finished % axis->speed == 0) {
        // Check if last move
        if (abs((int32_t)axis->current_pos - (int32_t)axis->end_pos) <= axis->steps_per_move) {
          axis->current_pos = axis->end_pos;
          next_servo_pos = axis->end_pos;
          axis->num_moves = 0;
          axis->num_moves_finished = 0;
          axis->start_pos = axis->current_pos;
        } else if (axis->direction == DIRECTION_POSITIVE) {
          next_servo_pos = axis->current_pos + axis->steps_per_move;
          axis->current_pos = axis->current_pos + axis->steps_per_move;
        } else {
          next_servo_pos = axis->current_pos - axis->steps_per_move;
          axis->current_pos = axis->current_pos - axis->steps_per_move;
        }
        xSemaphoreGive(position_update_sem_handle);
        rover_servo_write((RoverServoId)i, next_servo_pos, true);
      } else {
        xSemaphoreGive(position_update_sem_handle);
      }
    }
    vTaskDelay(pdMS_TO_TICKS(SERVO_UPDATE_INTERVAL_MS));
  }
}

static void reset_moving_servo(RoverServoId servoId)
{
  assert(servoId < SERVO_LAST);
  memset(&servo_states[servoId], 0 , sizeof(ServoState));
  servo_states[servoId].isPaused = true;
}
