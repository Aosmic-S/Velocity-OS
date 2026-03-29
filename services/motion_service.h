/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║         AOSMIC HYPERKERNEL OS — VelocityOS v1.0          ║
 * ║         Motion Service — Differential Drive Engine       ║
 * ╚══════════════════════════════════════════════════════════╝
 */

#pragma once
#include "../kernel/hyperkernel.h"
#include "../hal/hal.h"

// ─── Motion Constants ─────────────────────────────────────
#define MOTION_MAX_SPEED         100.0f    // percent
#define MOTION_MIN_SPEED         0.0f
#define MOTION_ACCEL_RATE        2.5f      // %/tick
#define MOTION_DECEL_RATE        5.0f      // %/tick fast brake
#define MOTION_OBSTACLE_DIST_CM  20.0f     // Emergency stop distance
#define MOTION_WARN_DIST_CM      40.0f     // Warning distance
#define MOTION_TICK_MS           20        // 50Hz control loop

// ─── Motion Commands ──────────────────────────────────────
typedef enum {
    MOTION_CMD_STOP        = 0,
    MOTION_CMD_FORWARD     = 1,
    MOTION_CMD_BACKWARD    = 2,
    MOTION_CMD_TURN_LEFT   = 3,
    MOTION_CMD_TURN_RIGHT  = 4,
    MOTION_CMD_PIVOT_LEFT  = 5,
    MOTION_CMD_PIVOT_RIGHT = 6,
    MOTION_CMD_STRAFE_LEFT = 7,
    MOTION_CMD_STRAFE_RIGHT= 8,
    MOTION_CMD_CUSTOM      = 9,
    MOTION_CMD_ESTOP       = 99,
} motion_cmd_t;

// ─── Wheel IDs ────────────────────────────────────────────
typedef enum {
    WHEEL_FL = 0,
    WHEEL_FR = 1,
    WHEEL_RL = 2,
    WHEEL_RR = 3,
    WHEEL_COUNT
} wheel_id_t;

// ─── Acceleration Profile ─────────────────────────────────
typedef enum {
    ACCEL_LINEAR    = 0,
    ACCEL_SMOOTH    = 1,   // Sigmoid curve
    ACCEL_STEP      = 2,   // Instant
    ACCEL_EXPO      = 3,   // Exponential
} accel_profile_t;

// ─── Motion Command Message ───────────────────────────────
typedef struct {
    motion_cmd_t    cmd;
    float           speed;          // 0-100%
    float           turn_bias;      // -1.0 left, +1.0 right
    accel_profile_t profile;
    uint32_t        duration_ms;    // 0 = continuous
    bool            override_safety;
} motion_msg_t;

// ─── Wheel State ──────────────────────────────────────────
typedef struct {
    float       target_speed;   // Desired
    float       current_speed;  // Actual (after ramp)
    bool        forward;
    ledc_channel_t pwm_ch;
    gpio_num_t  dir_a;
    gpio_num_t  dir_b;
    float       traction_factor; // 0.0-1.0 balance
} wheel_state_t;

// ─── Motion Service State ─────────────────────────────────
typedef struct {
    wheel_state_t   wheels[WHEEL_COUNT];
    motion_cmd_t    current_cmd;
    float           speed;
    float           turn_bias;
    accel_profile_t accel_profile;
    bool            estop_active;
    bool            obstacle_front;
    bool            obstacle_rear;
    float           obstacle_dist_cm;
    uint64_t        cmd_count;
    uint64_t        estop_count;
    int64_t         last_cmd_time;
    QueueHandle_t   cmd_queue;
    QueueHandle_t   obstacle_sub;
    QueueHandle_t   estop_sub;
} motion_state_t;

extern motion_state_t g_motion;

// ─── Motion Service API ───────────────────────────────────
hk_status_t motion_service_init(void);
hk_status_t motion_service_start(void);
hk_status_t motion_service_stop(void);
hk_status_t motion_send_cmd(motion_msg_t* cmd);
hk_status_t motion_set_wheel(wheel_id_t w, float speed, bool forward);
void        motion_emergency_stop(const char* reason);
float       motion_get_speed(void);
