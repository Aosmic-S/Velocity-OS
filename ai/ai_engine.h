/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║         AOSMIC HYPERKERNEL OS — VelocityOS v1.0          ║
 * ║         AI Decision Engine — Rule-Based Behavior FSM     ║
 * ╚══════════════════════════════════════════════════════════╝
 */

#pragma once
#include "../kernel/hyperkernel.h"
#include "motion_service.h"

// ─── AI Behavior States ───────────────────────────────────
typedef enum {
    AI_STATE_BOOT          = 0,
    AI_STATE_IDLE          = 1,
    AI_STATE_PATROL        = 2,
    AI_STATE_ALERT         = 3,
    AI_STATE_MANUAL        = 4,   // Remote override
    AI_STATE_EMERGENCY     = 5,
    AI_STATE_AVOID         = 6,
    AI_STATE_FIRE_RESPONSE = 7,
    AI_STATE_GAS_RESPONSE  = 8,
    AI_STATE_LOW_BATTERY   = 9,
} ai_state_t;

// ─── Sensor Data Snapshot ─────────────────────────────────
typedef struct {
    float   obstacle_front_cm;
    float   obstacle_rear_cm;
    float   obstacle_left_cm;
    float   gas_level;          // 0.0 - 1.0 normalized
    bool    flame_detected;
    bool    smoke_detected;
    float   battery_voltage;
    float   temperature_c;
    int64_t timestamp;
} ai_sensor_snapshot_t;

// ─── AI Decision Output ───────────────────────────────────
typedef struct {
    ai_state_t  new_state;
    motion_cmd_t motion_cmd;
    float        speed;
    bool         activate_pump;
    bool         activate_buzzer;
    bool         send_alert;
    char         reason[64];
} ai_decision_t;

// ─── AI Engine State ──────────────────────────────────────
typedef struct {
    ai_state_t              current_state;
    ai_state_t              prev_state;
    ai_sensor_snapshot_t    sensors;
    ai_decision_t           last_decision;
    uint64_t                decision_count;
    uint64_t                state_changes;
    int64_t                 state_entered_at;
    bool                    manual_override;
    float                   patrol_speed;
    int                     patrol_direction;
    int                     patrol_step;
    QueueHandle_t           sensor_sub;
    QueueHandle_t           power_sub;
    QueueHandle_t           manual_sub;
} ai_engine_t;

extern ai_engine_t g_ai;

// ─── AI Service API ───────────────────────────────────────
hk_status_t ai_service_init(void);
hk_status_t ai_service_start(void);
hk_status_t ai_service_stop(void);
const char* ai_state_name(ai_state_t state);
