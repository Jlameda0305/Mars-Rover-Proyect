#include "rover_kinematics.h"
#include <algorithm>
#include <cstdlib>

template <typename T>
static inline T clamp_val(T val, T low, T high) {
    return (val < low) ? low : ((val > high) ? high : val);
}

SteeringAngles calculate_steering(RoverMode mode, uint16_t steer_signal) {
    if (mode == DRIVE_TURN_SPIN) {
        // Fixed tangential angles for 360 spin
        return {
            static_cast<uint16_t>(RC_HIGH),  // FL = 2000
            static_cast<uint16_t>(RC_LOW),   // FR = 1000
            static_cast<uint16_t>(RC_HIGH),  // BL = 2000
            static_cast<uint16_t>(RC_LOW)    // BR = 1000
        };
    }

    // DRIVE_TURN_NORMAL: 4WS Ackermann-like opposite steering
    if (steer_signal == 0) {
        return { RC_CENTER, RC_CENTER, RC_CENTER, RC_CENTER };
    }

    uint16_t clamped = clamp_val<uint16_t>(steer_signal, RC_LOW, RC_HIGH);
    uint16_t diff = static_cast<uint16_t>(std::abs(static_cast<int32_t>(RC_CENTER) - static_cast<int32_t>(clamped)));

    if (clamped < RC_CENTER) {
        // Left turn: front wheels turn left (1500 - diff), rear wheels turn opposite (1500 + diff)
        return {
            static_cast<uint16_t>(RC_CENTER - diff),
            static_cast<uint16_t>(RC_CENTER - diff),
            static_cast<uint16_t>(RC_CENTER + diff),
            static_cast<uint16_t>(RC_CENTER + diff)
        };
    } else if (clamped > RC_CENTER) {
        // Right turn: front wheels turn right (1500 + diff), rear wheels turn opposite (1500 - diff)
        return {
            static_cast<uint16_t>(RC_CENTER + diff),
            static_cast<uint16_t>(RC_CENTER + diff),
            static_cast<uint16_t>(RC_CENTER - diff),
            static_cast<uint16_t>(RC_CENTER - diff)
        };
    } else {
        // Neutral: centered straight ahead
        return { RC_CENTER, RC_CENTER, RC_CENTER, RC_CENTER };
    }
}

MotorOutputs calculate_motors(RoverMode mode, uint16_t throttle_signal, uint16_t steer_signal) {
    if (mode == DRIVE_TURN_NORMAL) {
        // Normal mode: identical forward/reverse/idle signal on both left and right sides
        uint16_t throttle = (throttle_signal == 0) ? RC_CENTER : clamp_val<uint16_t>(throttle_signal, RC_LOW, RC_HIGH);
        return { throttle, throttle };
    }

    // DRIVE_TURN_SPIN: differential inversion proportional to steering wheel (horario vs antihorario)
    if (steer_signal == 0) {
        return { RC_CENTER, RC_CENTER };
    }

    uint16_t steer = clamp_val<uint16_t>(steer_signal, RC_LOW, RC_HIGH);
    uint16_t diff = static_cast<uint16_t>(std::abs(static_cast<int32_t>(RC_CENTER) - static_cast<int32_t>(steer)));

    if (steer < RC_CENTER) {
        // Spin Anticlockwise: left backwards, right forwards
        return {
            static_cast<uint16_t>(RC_CENTER - diff),
            static_cast<uint16_t>(RC_CENTER + diff)
        };
    } else if (steer > RC_CENTER) {
        // Spin Clockwise: left forwards, right backwards
        return {
            static_cast<uint16_t>(RC_CENTER + diff),
            static_cast<uint16_t>(RC_CENTER - diff)
        };
    } else {
        // Neutral spin: idle motors
        return { RC_CENTER, RC_CENTER };
    }
}

uint16_t filter_rc_signal(const uint16_t* samples, uint8_t sample_count) {
    if (samples == nullptr || sample_count == 0) {
        return RC_CENTER;
    }

    uint32_t sum = 0;
    for (uint8_t i = 0; i < sample_count; ++i) {
        sum += samples[i];
    }
    uint16_t avg = static_cast<uint16_t>(sum / sample_count);

    // Fail-safe: disconnected controller or invalid signal
    if (avg <= 800) {
        return RC_CENTER;
    }
    // Extremo inferior
    if (avg < 1010) {
        return RC_LOW;
    }
    // Extremo superior
    if (avg > 1990) {
        return RC_HIGH;
    }
    // Deadzone (zona muerta): 1480-1520 µs -> 1500
    if (avg >= 1480 && avg <= 1520) {
        return RC_CENTER;
    }

    return avg;
}

uint16_t calculate_servo_ticks(uint8_t servo_id, uint16_t us_signal, int16_t offset) {
    (void)servo_id;
    // Map us_signal from [RC_LOW (1000), RC_HIGH (2000)] to [SERVOMIN (250), SERVOMAX (500)]
    int32_t val = static_cast<int32_t>(us_signal - RC_LOW) * (SERVOMAX - SERVOMIN) / (RC_HIGH - RC_LOW) + SERVOMIN;
    val += offset;

    // Safety clamping: [SERVOMIN_FULL_RANGE (150), SERVOMAX_FULL_RANGE (600)]
    if (val < SERVOMIN_FULL_RANGE) {
        val = SERVOMIN_FULL_RANGE;
    } else if (val > SERVOMAX_FULL_RANGE) {
        val = SERVOMAX_FULL_RANGE;
    }

    return static_cast<uint16_t>(val);
}
