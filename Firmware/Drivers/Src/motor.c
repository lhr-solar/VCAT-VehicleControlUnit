/**
 * @file motor.c
 * @brief Motor command calculations (current mapping, ramp, rollover limit)
 * @copyright Copyright (c) 2026 UT Longhorn Racing Solar
 */

#include "motor.h"
#include "rollover_speed_table.h"
#include "FaultBits.h"

#include <stdlib.h>
#include <math.h>

static bool rollover_limit_active = false;
static float last_sent_current = 0.0f;
static bool performance_mode_active = false;

#if MOTOR_MAX_CURRENT_MODE == MOTOR_MAX_CURRENT_MODE_STEPS
/* At/above each speed_mph (inclusive), max_current applies; last match wins.
 * Below first step → MOTOR_MAX_CURRENT_PERCENT; above last → last max_current. */
static const motor_max_current_step_t MAX_CURRENT_STEPS[] = {
    {7.0f, 0.80f}, {17.0f, 0.75f}, {20.0f, 0.70f},
    {23.0f, 0.60f}, {25.0f, 0.50f}, {28.5f, 0.45f}
};
static const size_t NUM_MAX_CURRENT_STEPS =
    sizeof(MAX_CURRENT_STEPS) / sizeof(MAX_CURRENT_STEPS[0]);

#elif MOTOR_MAX_CURRENT_MODE == MOTOR_MAX_CURRENT_MODE_PIECEWISE
/*
 * Linear segments in motor mechanical Hz (rps = |rpm| / 60).
 *
 * Inclusivity (closed–closed per segment, earlier wins at shared knots):
 *   - Each segment covers [start_rps, end_rps] inclusive.
 *   - If adjacent segments share a knot (end_i == start_{i+1}), the earlier
 *     segment owns that exact point. Keep end_current[i] == start_current[i+1]
 *     so the join is continuous.
 *
 * Outside the table:
 *   - Below first start_rps → first start_current
 *   - Above last end_rps    → last end_current
 * Table currents are 0–1 of soft limit (1 = full soft).
 */
static const motor_max_current_segment_t MAX_CURRENT_SEGMENTS[] = {
    /* start_hz, end_hz, start_current, end_current */
    {0.0f,  2.0f,  1.00f, 1.00f},
    {2.0f,  2.5f,  1.00f, 0.75f},
    {2.5f,  3.75f, 0.75f, 0.78f},
    {3.75f, 4.0f,  0.78f, 0.85f},
};
static const size_t NUM_MAX_CURRENT_SEGMENTS =
    sizeof(MAX_CURRENT_SEGMENTS) / sizeof(MAX_CURRENT_SEGMENTS[0]);
#endif

/**.
 * @brief Rollover limit (0–1 of soft limit): 0 when over limit, else 1.
 * @note Gated by APPLY_ROLLOVER. LUT indexed by abs(LWS_Angle / 10).
 */
static float get_rollover_limit(float vehicle_velocity_mps, int16_t lws_angle) {
#if APPLY_ROLLOVER
    int deg = abs((int)lws_angle) / 10;
    if (deg > (int)ROLLOVER_TABLE_MAX_DEG) deg = (int)ROLLOVER_TABLE_MAX_DEG;

    uint16_t v_max_cms = rollover_speed_table[deg];
    uint16_t v_now_cms = (uint16_t)(fabsf(vehicle_velocity_mps) * 100.0f);

    if (v_max_cms != ROLLOVER_TABLE_NO_LIMIT && v_now_cms > v_max_cms) {
        rollover_limit_active = true;
        warning_set(WARNING_ID_TIPPING_LIMIT_ACTIVE);
        return 0.0f;
    }

    warning_clear(WARNING_ID_TIPPING_LIMIT_ACTIVE);
    rollover_limit_active = false;
    return 1.0f;
#else
    (void)vehicle_velocity_mps;
    (void)lws_angle;
    rollover_limit_active = false;
    return 1.0f;
#endif
}

float motor_get_max_current(float motor_rpm, float vehicle_velocity_mps) {
    if (performance_mode_active) {
        return 1.0f;
    }

#if MOTOR_MAX_CURRENT_MODE == MOTOR_MAX_CURRENT_MODE_STEPS
    (void)motor_rpm;
    float speed_mph = fabsf(vehicle_velocity_mps) * MOTOR_METERS_SEC_TO_MPH;
    float cap = 1.0f;
    for (size_t i = 0; i < NUM_MAX_CURRENT_STEPS; ++i) {
        if (speed_mph >= MAX_CURRENT_STEPS[i].speed_mph) {
            cap = MAX_CURRENT_STEPS[i].max_current;
        }
    }
    return cap;

#elif MOTOR_MAX_CURRENT_MODE == MOTOR_MAX_CURRENT_MODE_PIECEWISE
    (void)vehicle_velocity_mps;
    float rps = fabsf(motor_rpm) / 60.0f;

    if (NUM_MAX_CURRENT_SEGMENTS == 0) {
        return 1.0f;
    }

    if (rps <= MAX_CURRENT_SEGMENTS[0].start_rps) {
        return MAX_CURRENT_SEGMENTS[0].start_current;
    }

    for (size_t i = 0; i < NUM_MAX_CURRENT_SEGMENTS; ++i) {
        const motor_max_current_segment_t *s = &MAX_CURRENT_SEGMENTS[i];
        if (rps > s->end_rps) continue;

        /* Gap before this segment: hold previous segment's end current */
        if (rps < s->start_rps) {
            return (i == 0) ? s->start_current
                            : MAX_CURRENT_SEGMENTS[i - 1].end_current;
        }

        float span = s->end_rps - s->start_rps;
        if (span <= 0.0f) return s->end_current;
        float t = (rps - s->start_rps) / span;
        return s->start_current + t * (s->end_current - s->start_current);
    }

    return MAX_CURRENT_SEGMENTS[NUM_MAX_CURRENT_SEGMENTS - 1].end_current;

#elif MOTOR_MAX_CURRENT_MODE == MOTOR_MAX_CURRENT_MODE_DIRECT_PEDAL
    (void)motor_rpm;
    (void)vehicle_velocity_mps;
    return 1.0f;

#else
#error "MOTOR_MAX_CURRENT_MODE must be STEPS, PIECEWISE, or DIRECT_PEDAL"
#endif
}

float motor_get_drive_current(float motor_rpm, float vehicle_velocity_mps,
                              int16_t lws_angle, uint8_t accel_percent_0_100) {
    float pedal = (accel_percent_0_100 <= MOTOR_ACCEL_DEADZONE_MIN)
                      ? 0.0f
                      : (float)accel_percent_0_100 / 100.0f;
#if MOTOR_MAX_CURRENT_MODE == MOTOR_MAX_CURRENT_MODE_DIRECT_PEDAL
    (void)motor_rpm;
    float rollover = get_rollover_limit(vehicle_velocity_mps, lws_angle);
    return MOTOR_MAX_CURRENT_PERCENT * fminf(pedal, rollover);
#else
    float rollover = get_rollover_limit(vehicle_velocity_mps, lws_angle);
    float max_curr = motor_get_max_current(motor_rpm, vehicle_velocity_mps);
    return MOTOR_MAX_CURRENT_PERCENT * fminf(pedal, fminf(rollover, max_curr));
#endif
}

float motor_get_pwr_current(uint8_t accel_percent_0_100) {
    float pedal = (accel_percent_0_100 <= MOTOR_ACCEL_DEADZONE_MIN)
                      ? 0.0f
                      : (float)accel_percent_0_100 / 100.0f;
    return pedal * (MOTOR_SOFT_LIM_CURR_PWR / MOTOR_HARD_LIM_CURR_PWR);
}

float motor_ramp_current(float target_current) {
#if MOTOR_RAMP_ENABLED
    last_sent_current = fminf(target_current, last_sent_current + MOTOR_CURRENT_RAMP_PER_TICK);
#else
    last_sent_current = target_current;
#endif
    return last_sent_current;
}

void motor_reset_current_ramp(void) {
    last_sent_current = 0.0f;
}

bool motor_is_over_rollover_speed(void) {
    return rollover_limit_active;
}

void motor_set_performance_mode(bool enabled) {
    performance_mode_active = enabled;
}
