/*
 * Simplified Rover RC Firmware
 * Controlled exclusively as an RC car via WiFi Access Point ("Rover") using WebSockets.
 * Hardware features such as arm, camera head, LoRa and RMT receiver are disabled.
 */

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <esp_log.h>
#include <Wire.h>
#include "rover_config.h"
#include "wifi_controller.h"
#include "rover_servo.h"
#include "rover_driving.h"
#include "switch_checker.h"

#define DEBUG

#ifdef DEBUG
#define LOG Serial.print
#define LOGF Serial.printf
#define LOGLN Serial.println
#else
#define LOG(msg)
#define LOGF(...)
#define LOGLN(msg)
#endif

static const char* TAG = "ROVER_MAIN";

static uint16_t rc_values[RC_NUM_CHANNELS][RC_FILTER_SAMPLES];
static uint8_t rc_value_index = 0;

static RoverMode current_rover_mode = DRIVE_TURN_NORMAL;
static bool wifi_control_enabled = false;

static void handle_wifi_controller_status(WifiControllerStatus status);
static void handle_controller_disconnected(uint16_t last_sampled_signal);
static uint16_t filter_signal(uint16_t* signals);

void setup() {
    Serial.begin(SERIAL_PORT_SPEED);
    Wire.begin(ROVER_SDA_PIN, ROVER_SCL_PIN);
    Wire.setClock(400000);

    xSemaphoreHandle i2cSemaphoreHandle = xSemaphoreCreateBinary();
    assert(i2cSemaphoreHandle != NULL);
    xSemaphoreGive(i2cSemaphoreHandle);

    rover_servo_init(i2cSemaphoreHandle);
    rover_driving_init();
    handle_controller_disconnected(0);

    wifi_controller_init("Rover", NULL, WIFI_CONTROLLER_AP);
    wifi_controller_register_connection_callback(&handle_wifi_controller_status);

    LOGF("Rover Ready (Simplified RC)! Core: %d\n", xPortGetCoreID());
}

void loop() {
    uint16_t steer_raw = wifi_controller_get_ch_val(RC_STEER_CHANNEL);
    uint16_t motor_raw = wifi_controller_get_ch_val(RC_MOTOR_CHANNEL);
    uint16_t mode_raw  = wifi_controller_get_ch_val(RC_ROVER_MODE_ROVER_CHANNEL);

    rc_values[RC_STEER_CHANNEL][rc_value_index] = steer_raw;
    rc_values[RC_MOTOR_CHANNEL][rc_value_index] = motor_raw;

    RoverMode mode = (mode_raw >= 1400 && mode_raw <= 1600) ? DRIVE_TURN_SPIN : DRIVE_TURN_NORMAL;
    if (mode != current_rover_mode) {
        current_rover_mode = mode;
        rover_driving_set_drive_mode(current_rover_mode);
    }

    uint16_t motor_signal = RC_CENTER;
    if (current_rover_mode == DRIVE_TURN_NORMAL) {
        motor_signal = filter_signal(rc_values[RC_MOTOR_CHANNEL]);
    } else if (current_rover_mode == DRIVE_TURN_SPIN) {
        motor_signal = filter_signal(rc_values[RC_STEER_CHANNEL]);
    }

    rover_driving_move(motor_signal);
    rover_driving_steer(filter_signal(rc_values[RC_STEER_CHANNEL]));

    rc_value_index = (rc_value_index + 1) % RC_FILTER_SAMPLES;
    vTaskDelay(pdMS_TO_TICKS(20));
}

static void handle_wifi_controller_status(WifiControllerStatus status) {
    LOGF("Wifi Controller status: %d\n", status);
    if (status != WIFI_CONTROLLER_CONNECTED) {
        handle_controller_disconnected(0);
        wifi_control_enabled = false;
    } else {
        wifi_control_enabled = true;
    }
}

static void handle_controller_disconnected(uint16_t last_sampled_signal) {
    // Set all RC values to mid position if signal is 0 (controller disconnected)
    if (last_sampled_signal == 0) {
        for (uint8_t channel = 0; channel < RC_NUM_CHANNELS; channel++) {
            for (uint8_t sample = 0; sample < RC_FILTER_SAMPLES; sample++) {
                rc_values[channel][sample] = RC_CENTER;
            }
        }
    }
}

static uint16_t filter_signal(uint16_t* signals) {
    uint32_t signal = 0;
    for (uint8_t i = 0; i < RC_FILTER_SAMPLES; i++) {
        signal += signals[i];
    }
    signal = signal / RC_FILTER_SAMPLES;

    if (signal <= 800) return RC_CENTER; // If controller disconnected or invalid signal
    if (signal > 800 && signal < 1010) return RC_LOW;
    if (signal > 1990) return RC_HIGH;
    if (signal > 1480 && signal < 1520) return RC_CENTER;
    return signal;
}
