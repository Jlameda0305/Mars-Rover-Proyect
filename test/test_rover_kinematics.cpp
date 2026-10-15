#include <iostream>
#include <iomanip>
#include <string>
#include <vector>
#include <cmath>
#include "rover_kinematics.h"

// Simple zero-dependency test framework
static int g_total_assertions = 0;
static int g_passed_assertions = 0;
static int g_failed_assertions = 0;

static int g_total_suites = 0;
static int g_passed_suites = 0;

#define ASSERT_TRUE(condition, message) do { \
    g_total_assertions++; \
    if (condition) { \
        g_passed_assertions++; \
    } else { \
        g_failed_assertions++; \
        std::cerr << "  [FAIL] " << message << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
    } \
} while (0)

#define ASSERT_EQ(actual, expected, message) do { \
    g_total_assertions++; \
    if ((actual) == (expected)) { \
        g_passed_assertions++; \
    } else { \
        g_failed_assertions++; \
        std::cerr << "  [FAIL] " << message << " | Expected: " << (expected) \
                  << ", Got: " << (actual) << " (" << __FILE__ << ":" << __LINE__ << ")\n"; \
    } \
} while (0)

void run_suite(const std::string& suite_name, void (*suite_func)()) {
    g_total_suites++;
    int initial_failures = g_failed_assertions;
    std::cout << "\n========================================\n";
    std::cout << "SUITE: " << suite_name << "\n";
    std::cout << "========================================\n";
    suite_func();
    if (g_failed_assertions == initial_failures) {
        g_passed_suites++;
        std::cout << "--> SUITE PASSED: " << suite_name << "\n";
    } else {
        std::cerr << "--> SUITE FAILED: " << suite_name << "\n";
    }
}

// ---------------------------------------------------------
// 1. Suite: filter_rc_signal
// ---------------------------------------------------------
void test_suite_filter_rc_signal() {
    // 1.1 Deadzone (1480-1520 µs -> 1500)
    {
        uint16_t s1[] = { 1500 };
        ASSERT_EQ(filter_rc_signal(s1, 1), 1500, "Deadzone center 1500us -> 1500");

        uint16_t s_low_edge[] = { 1480 };
        ASSERT_EQ(filter_rc_signal(s_low_edge, 1), 1500, "Deadzone lower bound 1480us -> 1500");

        uint16_t s_high_edge[] = { 1520 };
        ASSERT_EQ(filter_rc_signal(s_high_edge, 1), 1500, "Deadzone upper bound 1520us -> 1500");

        uint16_t s_in_1[] = { 1485 };
        ASSERT_EQ(filter_rc_signal(s_in_1, 1), 1500, "Deadzone inside 1485us -> 1500");

        uint16_t s_in_2[] = { 1515 };
        ASSERT_EQ(filter_rc_signal(s_in_2, 1), 1500, "Deadzone inside 1515us -> 1500");
    }

    // 1.2 Fail-safe (<= 800 µs -> 1500)
    {
        uint16_t s_zero[] = { 0 };
        ASSERT_EQ(filter_rc_signal(s_zero, 1), 1500, "Fail-safe 0us -> 1500");

        uint16_t s_500[] = { 500 };
        ASSERT_EQ(filter_rc_signal(s_500, 1), 1500, "Fail-safe 500us -> 1500");

        uint16_t s_800[] = { 800 };
        ASSERT_EQ(filter_rc_signal(s_800, 1), 1500, "Fail-safe boundary 800us -> 1500");

        ASSERT_EQ(filter_rc_signal(nullptr, 0), 1500, "Fail-safe null/empty samples -> 1500");
    }

    // 1.3 Extremos (< 1010 -> 1000, > 1990 -> 2000)
    {
        uint16_t s_850[] = { 850 };
        ASSERT_EQ(filter_rc_signal(s_850, 1), 1000, "Extremo inferior 850us -> 1000");

        uint16_t s_1000[] = { 1000 };
        ASSERT_EQ(filter_rc_signal(s_1000, 1), 1000, "Extremo inferior 1000us -> 1000");

        uint16_t s_1009[] = { 1009 };
        ASSERT_EQ(filter_rc_signal(s_1009, 1), 1000, "Extremo inferior 1009us boundary -> 1000");

        uint16_t s_1991[] = { 1991 };
        ASSERT_EQ(filter_rc_signal(s_1991, 1), 2000, "Extremo superior 1991us boundary -> 2000");

        uint16_t s_2000[] = { 2000 };
        ASSERT_EQ(filter_rc_signal(s_2000, 1), 2000, "Extremo superior 2000us -> 2000");

        uint16_t s_2100[] = { 2100 };
        ASSERT_EQ(filter_rc_signal(s_2100, 1), 2000, "Extremo superior 2100us -> 2000");
    }

    // 1.4 Rangos normales
    {
        uint16_t s_1010[] = { 1010 };
        ASSERT_EQ(filter_rc_signal(s_1010, 1), 1010, "Normal range edge 1010us -> 1010");

        uint16_t s_1200[] = { 1200 };
        ASSERT_EQ(filter_rc_signal(s_1200, 1), 1200, "Normal range 1200us -> 1200");

        uint16_t s_1479[] = { 1479 };
        ASSERT_EQ(filter_rc_signal(s_1479, 1), 1479, "Normal range just below deadzone 1479us -> 1479");

        uint16_t s_1521[] = { 1521 };
        ASSERT_EQ(filter_rc_signal(s_1521, 1), 1521, "Normal range just above deadzone 1521us -> 1521");

        uint16_t s_1750[] = { 1750 };
        ASSERT_EQ(filter_rc_signal(s_1750, 1), 1750, "Normal range 1750us -> 1750");

        uint16_t s_1990[] = { 1990 };
        ASSERT_EQ(filter_rc_signal(s_1990, 1), 1990, "Normal range edge 1990us -> 1990");
    }

    // 1.5 Promedio de muestras múltiples
    {
        uint16_t s_multi1[] = { 1400, 1600 }; // avg = 1500 (deadzone)
        ASSERT_EQ(filter_rc_signal(s_multi1, 2), 1500, "Multi-sample avg 1500 -> 1500");

        uint16_t s_multi2[] = { 1200, 1300 }; // avg = 1250 (normal)
        ASSERT_EQ(filter_rc_signal(s_multi2, 2), 1250, "Multi-sample avg 1250 -> 1250");

        uint16_t s_multi3[] = { 700, 800 }; // avg = 750 (fail-safe)
        ASSERT_EQ(filter_rc_signal(s_multi3, 2), 1500, "Multi-sample avg 750 -> 1500");
    }
}

// ---------------------------------------------------------
// 2. Suite: calculate_steering en DRIVE_TURN_NORMAL
// ---------------------------------------------------------
void test_suite_calculate_steering_normal() {
    // 2.1 Neutro (1500us o 0)
    {
        SteeringAngles a_neutral = calculate_steering(DRIVE_TURN_NORMAL, 1500);
        ASSERT_EQ(a_neutral.front_left, 1500, "Neutral FL = 1500");
        ASSERT_EQ(a_neutral.front_right, 1500, "Neutral FR = 1500");
        ASSERT_EQ(a_neutral.back_left, 1500, "Neutral BL = 1500");
        ASSERT_EQ(a_neutral.back_right, 1500, "Neutral BR = 1500");

        SteeringAngles a_zero = calculate_steering(DRIVE_TURN_NORMAL, 0);
        ASSERT_EQ(a_zero.front_left, 1500, "Zero-signal FL = 1500");
        ASSERT_EQ(a_zero.front_right, 1500, "Zero-signal FR = 1500");
        ASSERT_EQ(a_zero.back_left, 1500, "Zero-signal BL = 1500");
        ASSERT_EQ(a_zero.back_right, 1500, "Zero-signal BR = 1500");
    }

    // 2.2 Giro a la izquierda (steer < 1500): delanteras a 1500-diff, traseras a 1500+diff
    {
        // diff = 200 (steer = 1300)
        SteeringAngles a_left1 = calculate_steering(DRIVE_TURN_NORMAL, 1300);
        ASSERT_EQ(a_left1.front_left, 1300, "Turn Left diff=200 FL = 1300");
        ASSERT_EQ(a_left1.front_right, 1300, "Turn Left diff=200 FR = 1300");
        ASSERT_EQ(a_left1.back_left, 1700, "Turn Left diff=200 BL = 1700");
        ASSERT_EQ(a_left1.back_right, 1700, "Turn Left diff=200 BR = 1700");

        // diff = 500 (steer = 1000, max left)
        SteeringAngles a_left_max = calculate_steering(DRIVE_TURN_NORMAL, 1000);
        ASSERT_EQ(a_left_max.front_left, 1000, "Turn Left max FL = 1000");
        ASSERT_EQ(a_left_max.front_right, 1000, "Turn Left max FR = 1000");
        ASSERT_EQ(a_left_max.back_left, 2000, "Turn Left max BL = 2000");
        ASSERT_EQ(a_left_max.back_right, 2000, "Turn Left max BR = 2000");
    }

    // 2.3 Giro a la derecha (steer > 1500): delanteras a 1500+diff, traseras a 1500-diff
    {
        // diff = 200 (steer = 1700)
        SteeringAngles a_right1 = calculate_steering(DRIVE_TURN_NORMAL, 1700);
        ASSERT_EQ(a_right1.front_left, 1700, "Turn Right diff=200 FL = 1700");
        ASSERT_EQ(a_right1.front_right, 1700, "Turn Right diff=200 FR = 1700");
        ASSERT_EQ(a_right1.back_left, 1300, "Turn Right diff=200 BL = 1300");
        ASSERT_EQ(a_right1.back_right, 1300, "Turn Right diff=200 BR = 1300");

        // diff = 500 (steer = 2000, max right)
        SteeringAngles a_right_max = calculate_steering(DRIVE_TURN_NORMAL, 2000);
        ASSERT_EQ(a_right_max.front_left, 2000, "Turn Right max FL = 2000");
        ASSERT_EQ(a_right_max.front_right, 2000, "Turn Right max FR = 2000");
        ASSERT_EQ(a_right_max.back_left, 1000, "Turn Right max BL = 1000");
        ASSERT_EQ(a_right_max.back_right, 1000, "Turn Right max BR = 1000");
    }

    // 2.4 Simetría y límites
    {
        // Simetría para varias desviaciones
        std::vector<uint16_t> diffs = { 50, 100, 250, 350, 500 };
        for (uint16_t d : diffs) {
            SteeringAngles l = calculate_steering(DRIVE_TURN_NORMAL, 1500 - d);
            SteeringAngles r = calculate_steering(DRIVE_TURN_NORMAL, 1500 + d);

            ASSERT_EQ(l.front_left + r.front_left, 3000, "Symmetry FL sum = 3000");
            ASSERT_EQ(l.front_right + r.front_right, 3000, "Symmetry FR sum = 3000");
            ASSERT_EQ(l.back_left + r.back_left, 3000, "Symmetry BL sum = 3000");
            ASSERT_EQ(l.back_right + r.back_right, 3000, "Symmetry BR sum = 3000");

            ASSERT_EQ(l.front_left, r.back_left, "Opposite symmetry FL == BR");
            ASSERT_EQ(l.back_left, r.front_left, "Opposite symmetry BL == FR");
        }

        // Límites extremos fuera de [1000, 2000]
        SteeringAngles under = calculate_steering(DRIVE_TURN_NORMAL, 800);
        ASSERT_EQ(under.front_left, 1000, "Underflow steer clamped to max left FL=1000");
        ASSERT_EQ(under.back_left, 2000, "Underflow steer clamped to max left BL=2000");

        SteeringAngles over = calculate_steering(DRIVE_TURN_NORMAL, 2200);
        ASSERT_EQ(over.front_left, 2000, "Overflow steer clamped to max right FL=2000");
        ASSERT_EQ(over.back_left, 1000, "Overflow steer clamped to max right BL=1000");
    }
}

// ---------------------------------------------------------
// 3. Suite: calculate_steering en DRIVE_TURN_SPIN
// ---------------------------------------------------------
void test_suite_calculate_steering_spin() {
    // Ángulos fijos tangenciales: FL=2000, FR=1000, BL=2000, BR=1000 independientemente del volante
    std::vector<uint16_t> test_signals = { 0, 1000, 1200, 1500, 1800, 2000, 2200 };
    for (uint16_t sig : test_signals) {
        SteeringAngles a = calculate_steering(DRIVE_TURN_SPIN, sig);
        ASSERT_EQ(a.front_left, 2000, "Spin tangential FL = 2000");
        ASSERT_EQ(a.front_right, 1000, "Spin tangential FR = 1000");
        ASSERT_EQ(a.back_left, 2000, "Spin tangential BL = 2000");
        ASSERT_EQ(a.back_right, 1000, "Spin tangential BR = 1000");
    }
}

// ---------------------------------------------------------
// 4. Suite: calculate_motors en DRIVE_TURN_NORMAL
// ---------------------------------------------------------
void test_suite_calculate_motors_normal() {
    // 4.1 Reposo (1500us o 0)
    {
        MotorOutputs idle1 = calculate_motors(DRIVE_TURN_NORMAL, 1500, 1500);
        ASSERT_EQ(idle1.left, 1500, "Normal idle left = 1500");
        ASSERT_EQ(idle1.right, 1500, "Normal idle right = 1500");

        MotorOutputs idle0 = calculate_motors(DRIVE_TURN_NORMAL, 0, 1500);
        ASSERT_EQ(idle0.left, 1500, "Normal zero-throttle left = 1500");
        ASSERT_EQ(idle0.right, 1500, "Normal zero-throttle right = 1500");
    }

    // 4.2 Avance (throttle > 1500)
    {
        MotorOutputs fwd1 = calculate_motors(DRIVE_TURN_NORMAL, 1650, 1500);
        ASSERT_EQ(fwd1.left, 1650, "Normal forward left = 1650");
        ASSERT_EQ(fwd1.right, 1650, "Normal forward right = 1650");

        MotorOutputs fwd_max = calculate_motors(DRIVE_TURN_NORMAL, 2000, 1500);
        ASSERT_EQ(fwd_max.left, 2000, "Normal forward max left = 2000");
        ASSERT_EQ(fwd_max.right, 2000, "Normal forward max right = 2000");
    }

    // 4.3 Retroceso (throttle < 1500)
    {
        MotorOutputs rev1 = calculate_motors(DRIVE_TURN_NORMAL, 1350, 1500);
        ASSERT_EQ(rev1.left, 1350, "Normal reverse left = 1350");
        ASSERT_EQ(rev1.right, 1350, "Normal reverse right = 1350");

        MotorOutputs rev_max = calculate_motors(DRIVE_TURN_NORMAL, 1000, 1500);
        ASSERT_EQ(rev_max.left, 1000, "Normal reverse max left = 1000");
        ASSERT_EQ(rev_max.right, 1000, "Normal reverse max right = 1000");
    }

    // 4.4 Invarianza respecto a la dirección en modo normal
    {
        MotorOutputs m1 = calculate_motors(DRIVE_TURN_NORMAL, 1700, 1200);
        ASSERT_EQ(m1.left, 1700, "Steer invariant left = 1700");
        ASSERT_EQ(m1.right, 1700, "Steer invariant right = 1700");

        MotorOutputs m2 = calculate_motors(DRIVE_TURN_NORMAL, 1700, 1800);
        ASSERT_EQ(m2.left, 1700, "Steer invariant left = 1700");
        ASSERT_EQ(m2.right, 1700, "Steer invariant right = 1700");
    }

    // 4.5 Clamping de seguridad
    {
        MotorOutputs under = calculate_motors(DRIVE_TURN_NORMAL, 850, 1500);
        ASSERT_EQ(under.left, 1000, "Normal under-throttle clamped left = 1000");
        ASSERT_EQ(under.right, 1000, "Normal under-throttle clamped right = 1000");

        MotorOutputs over = calculate_motors(DRIVE_TURN_NORMAL, 2150, 1500);
        ASSERT_EQ(over.left, 2000, "Normal over-throttle clamped left = 2000");
        ASSERT_EQ(over.right, 2000, "Normal over-throttle clamped right = 2000");
    }
}

// ---------------------------------------------------------
// 5. Suite: calculate_motors en DRIVE_TURN_SPIN
// ---------------------------------------------------------
void test_suite_calculate_motors_spin() {
    // 5.1 Reposo / Neutro (steer = 1500 o 0)
    {
        MotorOutputs idle1 = calculate_motors(DRIVE_TURN_SPIN, 1500, 1500);
        ASSERT_EQ(idle1.left, 1500, "Spin neutral left = 1500");
        ASSERT_EQ(idle1.right, 1500, "Spin neutral right = 1500");

        MotorOutputs idle0 = calculate_motors(DRIVE_TURN_SPIN, 1500, 0);
        ASSERT_EQ(idle0.left, 1500, "Spin zero-steer left = 1500");
        ASSERT_EQ(idle0.right, 1500, "Spin zero-steer right = 1500");
    }

    // 5.2 Giro Horario (Clockwise, steer > 1500): left fwd (+diff), right rev (-diff)
    {
        // diff = 200 (steer = 1700)
        MotorOutputs cw1 = calculate_motors(DRIVE_TURN_SPIN, 1500, 1700);
        ASSERT_EQ(cw1.left, 1700, "Spin Clockwise diff=200 left = 1700");
        ASSERT_EQ(cw1.right, 1300, "Spin Clockwise diff=200 right = 1300");

        // diff = 500 (steer = 2000, max)
        MotorOutputs cw_max = calculate_motors(DRIVE_TURN_SPIN, 1500, 2000);
        ASSERT_EQ(cw_max.left, 2000, "Spin Clockwise max left = 2000");
        ASSERT_EQ(cw_max.right, 1000, "Spin Clockwise max right = 1000");
    }

    // 5.3 Giro Antihorario (Anticlockwise, steer < 1500): left rev (-diff), right fwd (+diff)
    {
        // diff = 200 (steer = 1300)
        MotorOutputs ccw1 = calculate_motors(DRIVE_TURN_SPIN, 1500, 1300);
        ASSERT_EQ(ccw1.left, 1300, "Spin Anticlockwise diff=200 left = 1300");
        ASSERT_EQ(ccw1.right, 1700, "Spin Anticlockwise diff=200 right = 1700");

        // diff = 500 (steer = 1000, max)
        MotorOutputs ccw_max = calculate_motors(DRIVE_TURN_SPIN, 1500, 1000);
        ASSERT_EQ(ccw_max.left, 1000, "Spin Anticlockwise max left = 1000");
        ASSERT_EQ(ccw_max.right, 2000, "Spin Anticlockwise max right = 2000");
    }

    // 5.4 Simetría diferencial
    {
        std::vector<uint16_t> diffs = { 50, 150, 250, 400, 500 };
        for (uint16_t d : diffs) {
            MotorOutputs cw = calculate_motors(DRIVE_TURN_SPIN, 1500, 1500 + d);
            MotorOutputs ccw = calculate_motors(DRIVE_TURN_SPIN, 1500, 1500 - d);

            ASSERT_EQ(cw.left, ccw.right, "Spin differential symmetry cw.left == ccw.right");
            ASSERT_EQ(cw.right, ccw.left, "Spin differential symmetry cw.right == ccw.left");
            ASSERT_EQ(cw.left + cw.right, 3000, "Spin differential complementary sum = 3000");
            ASSERT_EQ(ccw.left + ccw.right, 3000, "Spin differential complementary sum = 3000");
        }
    }

    // 5.5 Clamping en extremos
    {
        MotorOutputs under = calculate_motors(DRIVE_TURN_SPIN, 1500, 800);
        ASSERT_EQ(under.left, 1000, "Spin steer underflow clamped left = 1000");
        ASSERT_EQ(under.right, 2000, "Spin steer underflow clamped right = 2000");

        MotorOutputs over = calculate_motors(DRIVE_TURN_SPIN, 1500, 2250);
        ASSERT_EQ(over.left, 2000, "Spin steer overflow clamped left = 2000");
        ASSERT_EQ(over.right, 1000, "Spin steer overflow clamped right = 1000");
    }
}

// ---------------------------------------------------------
// 6. Suite: calculate_servo_ticks
// ---------------------------------------------------------
void test_suite_calculate_servo_ticks() {
    // 6.1 Calibración con offsets en posición central (1500us -> base 375 ticks)
    // Offsets de rover_config:
    // SERVO_FRONT_LEFT_OFFSET:  +20 -> 395
    // SERVO_FRONT_RIGHT_OFFSET: -15 -> 360
    // SERVO_BACK_LEFT_OFFSET:   +5  -> 380
    // SERVO_BACK_RIGHT_OFFSET:  -25 -> 350
    {
        ASSERT_EQ(calculate_servo_ticks(0, 1500, +20), 395, "Center FL (+20) = 395 ticks");
        ASSERT_EQ(calculate_servo_ticks(1, 1500, -15), 360, "Center FR (-15) = 360 ticks");
        ASSERT_EQ(calculate_servo_ticks(2, 1500, +5),  380, "Center BL (+5)  = 380 ticks");
        ASSERT_EQ(calculate_servo_ticks(3, 1500, -25), 350, "Center BR (-25) = 350 ticks");
    }

    // 6.2 Calibración en mínimo normal (1000us -> base 250 ticks)
    {
        ASSERT_EQ(calculate_servo_ticks(0, 1000, +20), 270, "Min FL (+20) = 270 ticks");
        ASSERT_EQ(calculate_servo_ticks(1, 1000, -15), 235, "Min FR (-15) = 235 ticks");
        ASSERT_EQ(calculate_servo_ticks(2, 1000, +5),  255, "Min BL (+5)  = 255 ticks");
        ASSERT_EQ(calculate_servo_ticks(3, 1000, -25), 225, "Min BR (-25) = 225 ticks");
    }

    // 6.3 Calibración en máximo normal (2000us -> base 500 ticks)
    {
        ASSERT_EQ(calculate_servo_ticks(0, 2000, +20), 520, "Max FL (+20) = 520 ticks");
        ASSERT_EQ(calculate_servo_ticks(1, 2000, -15), 485, "Max FR (-15) = 485 ticks");
        ASSERT_EQ(calculate_servo_ticks(2, 2000, +5),  505, "Max BL (+5)  = 505 ticks");
        ASSERT_EQ(calculate_servo_ticks(3, 2000, -25), 475, "Max BR (-25) = 475 ticks");
    }

    // 6.4 Clamping de seguridad (150 a 600 ticks)
    {
        // Límite inferior: 1000us (250) con offset negativo grande
        ASSERT_EQ(calculate_servo_ticks(0, 1000, -100), 150, "Exact min clamp boundary 250-100 = 150 ticks");
        ASSERT_EQ(calculate_servo_ticks(0, 1000, -150), 150, "Underflow clamped to 150 ticks");
        ASSERT_EQ(calculate_servo_ticks(0, 1000, -300), 150, "Extreme underflow clamped to 150 ticks");

        // Límite superior: 2000us (500) con offset positivo grande
        ASSERT_EQ(calculate_servo_ticks(0, 2000, +100), 600, "Exact max clamp boundary 500+100 = 600 ticks");
        ASSERT_EQ(calculate_servo_ticks(0, 2000, +150), 600, "Overflow clamped to 600 ticks");
        ASSERT_EQ(calculate_servo_ticks(0, 2000, +300), 600, "Extreme overflow clamped to 600 ticks");
    }
}

// ---------------------------------------------------------
// Main Entrypoint
// ---------------------------------------------------------
int main() {
    std::cout << "====================================================\n";
    std::cout << " MARS ROVER KINEMATICS NATIVE TEST SUITE (C++17)   \n";
    std::cout << "====================================================\n";

    run_suite("filter_rc_signal (Deadzone, Fail-safe, Extremos, Normal)", test_suite_filter_rc_signal);
    run_suite("calculate_steering (DRIVE_TURN_NORMAL - 4WS)", test_suite_calculate_steering_normal);
    run_suite("calculate_steering (DRIVE_TURN_SPIN - Tangencial)", test_suite_calculate_steering_spin);
    run_suite("calculate_motors (DRIVE_TURN_NORMAL - Traccion)", test_suite_calculate_motors_normal);
    run_suite("calculate_motors (DRIVE_TURN_SPIN - Diferencial)", test_suite_calculate_motors_spin);
    run_suite("calculate_servo_ticks (Offsets & Clamping 150-600)", test_suite_calculate_servo_ticks);

    std::cout << "\n====================================================\n";
    std::cout << "                 TEST RESULTS SUMMARY               \n";
    std::cout << "====================================================\n";
    std::cout << "  Total Suites:      " << g_total_suites << "\n";
    std::cout << "  Passed Suites:     " << g_passed_suites << "\n";
    std::cout << "  Failed Suites:     " << (g_total_suites - g_passed_suites) << "\n";
    std::cout << "  Total Assertions:  " << g_total_assertions << "\n";
    std::cout << "  Passed Assertions: " << g_passed_assertions << "\n";
    std::cout << "  Failed Assertions: " << g_failed_assertions << "\n";
    std::cout << "====================================================\n";

    if (g_failed_assertions == 0) {
        std::cout << ">>> OVERALL STATUS: ALL TESTS PASSED (100%) <<<\n";
        return 0;
    } else {
        std::cerr << ">>> OVERALL STATUS: SOME TESTS FAILED <<<\n";
        return 1;
    }
}
