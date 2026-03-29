/**
 * AI Decision Engine Implementation
 * Rule-Based Behavior Finite State Machine
 */

#include "ai_engine.h"
#include "../bus/message_bus.h"
#include "../hal/hal.h"
#include "esp_log.h"
#include "math.h"

static const char* TAG = "AI_Engine";
ai_engine_t g_ai = {0};

static const char* STATE_NAMES[] = {
    "BOOT", "IDLE", "PATROL", "ALERT", "MANUAL",
    "EMERGENCY", "AVOID", "FIRE_RESPONSE", "GAS_RESPONSE", "LOW_BATTERY"
};

const char* ai_state_name(ai_state_t s) {
    if (s >= 10) return "UNKNOWN";
    return STATE_NAMES[s];
}

// ─── State Transition ─────────────────────────────────────
static void ai_transition(ai_state_t new_state, const char* reason) {
    if (new_state == g_ai.current_state) return;

    ESP_LOGI(TAG, "STATE: %s → %s [%s]",
             ai_state_name(g_ai.current_state),
             ai_state_name(new_state), reason);

    g_ai.prev_state       = g_ai.current_state;
    g_ai.current_state    = new_state;
    g_ai.state_entered_at = esp_timer_get_time();
    g_ai.state_changes++;

    // Publish state change
    typedef struct { uint8_t state; char reason[48]; } state_msg_t;
    state_msg_t sm = { .state = new_state };
    strncpy(sm.reason, reason, 47);
    bus_publish(TOPIC_AI_DECISION, &sm, sizeof(sm));
}

// ─── Rule Engine ──────────────────────────────────────────
/**
 * Core AI rule table:
 * 
 *  IF gas_level > 0.7       → GAS_RESPONSE: stop + pump
 *  IF flame_detected        → FIRE_RESPONSE: retreat + pump
 *  IF battery < 6.5V        → LOW_BATTERY: slow patrol
 *  IF obstacle < 20cm       → AVOID: turn away
 *  IF manual_override       → MANUAL: pass-through
 *  IF idle > 5s             → PATROL
 *  ELSE                     → IDLE
 */
static ai_decision_t ai_evaluate_rules(void) {
    ai_decision_t d = {0};
    ai_sensor_snapshot_t* s = &g_ai.sensors;

    d.motion_cmd    = MOTION_CMD_STOP;
    d.speed         = 0;
    d.activate_pump = false;
    d.activate_buzzer = false;

    // ── RULE 1: Manual Override (highest priority)
    if (g_ai.manual_override) {
        d.new_state  = AI_STATE_MANUAL;
        snprintf(d.reason, 64, "Remote control active");
        return d;
    }

    // ── RULE 2: Emergency (ESTOP active)
    if (g_ai.current_state == AI_STATE_EMERGENCY) {
        d.new_state   = AI_STATE_EMERGENCY;
        d.motion_cmd  = MOTION_CMD_ESTOP;
        d.activate_buzzer = true;
        snprintf(d.reason, 64, "Emergency lockout");
        return d;
    }

    // ── RULE 3: Gas Detection (PRIORITY: CRITICAL)
    if (s->gas_level > 0.70f) {
        d.new_state     = AI_STATE_GAS_RESPONSE;
        d.motion_cmd    = MOTION_CMD_STOP;
        d.activate_pump = true;
        d.activate_buzzer = true;
        d.send_alert    = true;
        d.speed         = 0;
        snprintf(d.reason, 64, "Gas: %.0f%%", s->gas_level * 100);
        return d;
    }

    // ── RULE 4: Fire Detection
    if (s->flame_detected || s->smoke_detected) {
        d.new_state     = AI_STATE_FIRE_RESPONSE;
        d.motion_cmd    = MOTION_CMD_BACKWARD;
        d.activate_pump = true;
        d.activate_buzzer = true;
        d.send_alert    = true;
        d.speed         = 40.0f;
        snprintf(d.reason, 64, "Fire/smoke detected!");
        return d;
    }

    // ── RULE 5: Low Battery
    if (s->battery_voltage < 6.5f && s->battery_voltage > 1.0f) {
        d.new_state  = AI_STATE_LOW_BATTERY;
        d.motion_cmd = MOTION_CMD_STOP;
        d.send_alert = true;
        snprintf(d.reason, 64, "Battery: %.2fV", s->battery_voltage);
        return d;
    }

    // ── RULE 6: Obstacle Avoidance
    if (s->obstacle_front_cm > 0 && s->obstacle_front_cm < 20.0f) {
        d.new_state  = AI_STATE_AVOID;
        // Choose turn direction based on side distances
        if (s->obstacle_left_cm > s->obstacle_rear_cm) {
            d.motion_cmd = MOTION_CMD_PIVOT_LEFT;
        } else {
            d.motion_cmd = MOTION_CMD_PIVOT_RIGHT;
        }
        d.speed = 30.0f;
        snprintf(d.reason, 64, "Obstacle: %.1fcm", s->obstacle_front_cm);
        return d;
    }

    // ── RULE 7: Moderate obstacle — slow down with warning
    if (s->obstacle_front_cm < 40.0f && s->obstacle_front_cm > 0) {
        d.new_state  = AI_STATE_PATROL;
        d.motion_cmd = MOTION_CMD_FORWARD;
        d.speed      = 20.0f;
        snprintf(d.reason, 64, "Approaching obstacle");
        return d;
    }

    // ── RULE 8: Default patrol behavior
    int64_t time_in_state = (esp_timer_get_time() - g_ai.state_entered_at) / 1000000;
    if (g_ai.current_state == AI_STATE_IDLE && time_in_state > 5) {
        d.new_state  = AI_STATE_PATROL;
        d.motion_cmd = MOTION_CMD_FORWARD;
        d.speed      = g_ai.patrol_speed;
        snprintf(d.reason, 64, "Idle timeout → patrol");
        return d;
    }

    // ── Patrol maneuver sequencing
    if (g_ai.current_state == AI_STATE_PATROL) {
        g_ai.patrol_step++;
        if (g_ai.patrol_step < 60) {  // 60 ticks forward ~3s
            d.new_state  = AI_STATE_PATROL;
            d.motion_cmd = MOTION_CMD_FORWARD;
            d.speed      = g_ai.patrol_speed;
        } else if (g_ai.patrol_step < 75) {  // 15 ticks pivot
            d.new_state  = AI_STATE_PATROL;
            d.motion_cmd = (g_ai.patrol_direction % 2 == 0) ?
                           MOTION_CMD_PIVOT_LEFT : MOTION_CMD_PIVOT_RIGHT;
            d.speed = 35.0f;
        } else {
            g_ai.patrol_step = 0;
            g_ai.patrol_direction++;
            d.new_state  = AI_STATE_IDLE;
            d.motion_cmd = MOTION_CMD_STOP;
        }
        snprintf(d.reason, 64, "Patrol step %d", g_ai.patrol_step);
        return d;
    }

    // Default: stay idle
    d.new_state  = AI_STATE_IDLE;
    d.motion_cmd = MOTION_CMD_STOP;
    snprintf(d.reason, 64, "No condition triggered");
    return d;
}

// ─── Execute Decision ─────────────────────────────────────
static void ai_execute(ai_decision_t* d) {
    // State transition
    ai_transition(d->new_state, d->reason);

    // Motor command
    if (!g_ai.manual_override) {
        motion_msg_t mcmd = {
            .cmd     = d->motion_cmd,
            .speed   = d->speed,
            .profile = ACCEL_SMOOTH,
        };
        motion_send_cmd(&mcmd);
    }

    // Pump control
    hal_gpio_set(HAL_PUMP_RELAY, d->activate_pump);

    // Buzzer
    hal_gpio_set(HAL_BUZZER, d->activate_buzzer);

    // Alert
    if (d->send_alert) {
        bus_publish(TOPIC_SYSTEM_ALERT, d->reason, strlen(d->reason) + 1);
    }

    g_ai.last_decision = *d;
    g_ai.decision_count++;
}

// ─── AI Task (20Hz) ───────────────────────────────────────
static void ai_task(void* arg) {
    TickType_t last_wake = xTaskGetTickCount();
    ESP_LOGI(TAG, "AI Decision Engine online");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(50)); // 20Hz

        // Consume sensor updates
        hk_message_t msg;
        while (xQueueReceive(g_ai.sensor_sub, &msg, 0) == pdTRUE) {
            typedef struct {
                float front_cm, rear_cm, left_cm;
                float gas, battery, temp;
                bool  flame, smoke;
            } sensor_update_t;
            if (msg.payload_len >= sizeof(sensor_update_t)) {
                sensor_update_t* su = (sensor_update_t*)msg.payload;
                g_ai.sensors.obstacle_front_cm = su->front_cm;
                g_ai.sensors.obstacle_rear_cm  = su->rear_cm;
                g_ai.sensors.obstacle_left_cm  = su->left_cm;
                g_ai.sensors.gas_level         = su->gas;
                g_ai.sensors.battery_voltage   = su->battery;
                g_ai.sensors.temperature_c     = su->temp;
                g_ai.sensors.flame_detected    = su->flame;
                g_ai.sensors.smoke_detected    = su->smoke;
            }
        }

        // Check manual override from comm service
        while (xQueueReceive(g_ai.manual_sub, &msg, 0) == pdTRUE) {
            g_ai.manual_override = (msg.payload[0] != 0);
        }

        // Run rule engine
        ai_decision_t decision = ai_evaluate_rules();
        ai_execute(&decision);
    }
}

// ─── Service Lifecycle ────────────────────────────────────
hk_status_t ai_service_init(void) {
    g_ai.current_state   = AI_STATE_BOOT;
    g_ai.patrol_speed    = 50.0f;
    g_ai.patrol_direction = 0;

    bus_subscribe(TOPIC_SENSOR_DATA, "ai_sensor", &g_ai.sensor_sub);
    bus_subscribe(TOPIC_MOTOR_CMD,   "ai_manual", &g_ai.manual_sub);

    return HK_OK;
}

hk_status_t ai_service_start(void) {
    ai_transition(AI_STATE_IDLE, "Service start");
    hk_task_descriptor_t* td;
    return hk_task_create("svc_ai", ai_task, STACK_AI, NULL,
                           PRIORITY_AI, CORE_SERVICES, &td);
}

hk_status_t ai_service_stop(void) {
    g_ai.current_state = AI_STATE_IDLE;
    return HK_OK;
}

