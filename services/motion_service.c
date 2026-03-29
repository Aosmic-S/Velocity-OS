/**
 * Motion Service Implementation
 * Differential Drive + Speed Curves + Traction Balancing
 */

#include "motion_service.h"
#include "../bus/message_bus.h"
#include "math.h"
#include "esp_log.h"

static const char* TAG = "MotionSvc";
motion_state_t g_motion = {0};

// ─── Wheel Hardware Map ───────────────────────────────────
static const struct {
    ledc_channel_t pwm_ch;
    gpio_num_t     dir_a;
    gpio_num_t     dir_b;
} WHEEL_HW[WHEEL_COUNT] = {
    { LEDC_CHANNEL_0, HAL_MOTOR_FL_DIR_A, HAL_MOTOR_FL_DIR_B },
    { LEDC_CHANNEL_1, HAL_MOTOR_FR_DIR_A, HAL_MOTOR_FR_DIR_B },
    { LEDC_CHANNEL_2, HAL_MOTOR_RL_DIR_A, HAL_MOTOR_RL_DIR_B },
    { LEDC_CHANNEL_3, HAL_MOTOR_RR_DIR_A, HAL_MOTOR_RR_DIR_B },
};

// ─── Acceleration Curves ──────────────────────────────────
static float apply_accel_profile(float current, float target, accel_profile_t profile) {
    float diff = target - current;
    float step;

    switch (profile) {
        case ACCEL_SMOOTH: {
            // Sigmoid / S-curve
            float t = (current / 100.0f);
            float sigmoid = 1.0f / (1.0f + expf(-10.0f * (t - 0.5f)));
            step = diff * (0.1f + sigmoid * 0.15f);
            break;
        }
        case ACCEL_EXPO:
            step = diff * 0.2f * (1.0f + fabsf(diff) / 100.0f);
            break;
        case ACCEL_STEP:
            return target;
        case ACCEL_LINEAR:
        default: {
            float rate = (diff > 0) ? MOTION_ACCEL_RATE : MOTION_DECEL_RATE;
            step = (diff > 0) ? fminf(diff, rate) : fmaxf(diff, -rate);
            break;
        }
    }

    float result = current + step;
    if (result < 0.0f) result = 0.0f;
    if (result > 100.0f) result = 100.0f;
    return result;
}

// ─── Low-Level Wheel Control ──────────────────────────────
hk_status_t motion_set_wheel(wheel_id_t w, float speed, bool forward) {
    if (w >= WHEEL_COUNT) return HK_ERR_INVALID;

    wheel_state_t* ws = &g_motion.wheels[w];
    ws->target_speed = (speed < 0) ? 0 : (speed > 100) ? 100 : speed;
    ws->forward      = forward;

    // Apply traction factor (balance uneven surfaces)
    float effective = ws->current_speed * ws->traction_factor;

    hal_gpio_set(WHEEL_HW[w].dir_a, forward ? 1 : 0);
    hal_gpio_set(WHEEL_HW[w].dir_b, forward ? 0 : 1);
    hal_pwm_set_duty_pct(WHEEL_HW[w].pwm_ch, effective);
    return HK_OK;
}

// ─── Ramp All Wheels ──────────────────────────────────────
static void motion_ramp_step(void) {
    for (int i = 0; i < WHEEL_COUNT; i++) {
        wheel_state_t* ws = &g_motion.wheels[i];
        ws->current_speed = apply_accel_profile(ws->current_speed,
                                                 ws->target_speed,
                                                 g_motion.accel_profile);
        float effective = ws->current_speed * ws->traction_factor;
        hal_gpio_set(WHEEL_HW[i].dir_a, ws->forward ? 1 : 0);
        hal_gpio_set(WHEEL_HW[i].dir_b, ws->forward ? 0 : 1);
        hal_pwm_set_duty_pct(WHEEL_HW[i].pwm_ch, effective);
    }
}

// ─── Apply Motion Command ─────────────────────────────────
static void motion_apply_cmd(motion_msg_t* cmd) {
    if (g_motion.estop_active && !cmd->override_safety) {
        ESP_LOGW(TAG, "Command blocked: ESTOP active");
        return;
    }

    float speed = cmd->speed;
    float bias  = cmd->turn_bias;  // -1.0 to +1.0
    g_motion.accel_profile = cmd->profile;

    switch (cmd->cmd) {
        case MOTION_CMD_FORWARD:
            g_motion.wheels[WHEEL_FL].target_speed = speed * (1.0f - fmaxf(0, bias));
            g_motion.wheels[WHEEL_FR].target_speed = speed * (1.0f + fminf(0, bias));
            g_motion.wheels[WHEEL_RL].target_speed = speed * (1.0f - fmaxf(0, bias));
            g_motion.wheels[WHEEL_RR].target_speed = speed * (1.0f + fminf(0, bias));
            for (int i = 0; i < WHEEL_COUNT; i++) g_motion.wheels[i].forward = true;
            break;

        case MOTION_CMD_BACKWARD:
            g_motion.wheels[WHEEL_FL].target_speed = speed * (1.0f + fminf(0, bias));
            g_motion.wheels[WHEEL_FR].target_speed = speed * (1.0f - fmaxf(0, bias));
            g_motion.wheels[WHEEL_RL].target_speed = speed * (1.0f + fminf(0, bias));
            g_motion.wheels[WHEEL_RR].target_speed = speed * (1.0f - fmaxf(0, bias));
            for (int i = 0; i < WHEEL_COUNT; i++) g_motion.wheels[i].forward = false;
            break;

        case MOTION_CMD_PIVOT_LEFT:
            g_motion.wheels[WHEEL_FL].target_speed = speed;
            g_motion.wheels[WHEEL_RL].target_speed = speed;
            g_motion.wheels[WHEEL_FR].target_speed = speed;
            g_motion.wheels[WHEEL_RR].target_speed = speed;
            g_motion.wheels[WHEEL_FL].forward = false;
            g_motion.wheels[WHEEL_RL].forward = false;
            g_motion.wheels[WHEEL_FR].forward = true;
            g_motion.wheels[WHEEL_RR].forward = true;
            break;

        case MOTION_CMD_PIVOT_RIGHT:
            g_motion.wheels[WHEEL_FL].target_speed = speed;
            g_motion.wheels[WHEEL_RL].target_speed = speed;
            g_motion.wheels[WHEEL_FR].target_speed = speed;
            g_motion.wheels[WHEEL_RR].target_speed = speed;
            g_motion.wheels[WHEEL_FL].forward = true;
            g_motion.wheels[WHEEL_RL].forward = true;
            g_motion.wheels[WHEEL_FR].forward = false;
            g_motion.wheels[WHEEL_RR].forward = false;
            break;

        case MOTION_CMD_STOP:
        case MOTION_CMD_ESTOP:
            for (int i = 0; i < WHEEL_COUNT; i++) {
                g_motion.wheels[i].target_speed = 0;
            }
            break;

        case MOTION_CMD_CUSTOM:
            // Custom wheel speeds encoded in turn_bias as bitmask offset
            break;

        default: break;
    }

    g_motion.current_cmd  = cmd->cmd;
    g_motion.speed        = cmd->speed;
    g_motion.cmd_count++;
    g_motion.last_cmd_time = esp_timer_get_time();
}

// ─── Emergency Stop ───────────────────────────────────────
void motion_emergency_stop(const char* reason) {
    g_motion.estop_active = true;
    g_motion.estop_count++;

    // Hard stop — no ramping
    for (int i = 0; i < WHEEL_COUNT; i++) {
        g_motion.wheels[i].target_speed  = 0;
        g_motion.wheels[i].current_speed = 0;
        hal_pwm_set_duty(WHEEL_HW[i].pwm_ch, 0);
    }

    hal_gpio_set(HAL_LED_WARN, true);
    hk_estop(reason);
    ESP_LOGE(TAG, "[ESTOP] %s", reason);
}

// ─── Main Control Loop Task ───────────────────────────────
static void motion_task(void* arg) {
    motion_msg_t msg;
    TickType_t   last_wake = xTaskGetTickCount();

    // Subscribe to obstacle and estop events
    QueueHandle_t obs_q, estop_q;
    bus_subscribe(TOPIC_OBSTACLE, "motion_obs",   &obs_q);
    bus_subscribe(TOPIC_ESTOP,    "motion_estop", &estop_q);

    ESP_LOGI(TAG, "Motion service running at 50Hz");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(MOTION_TICK_MS));

        // Check obstacle events
        hk_message_t bus_msg;
        if (xQueueReceive(obs_q, &bus_msg, 0) == pdTRUE) {
            float dist;
            memcpy(&dist, bus_msg.payload, sizeof(float));
            g_motion.obstacle_dist_cm = dist;
            if (dist < MOTION_OBSTACLE_DIST_CM) {
                g_motion.obstacle_front = true;
                // Kernel-level override: stop all forward motion
                if (g_motion.current_cmd == MOTION_CMD_FORWARD) {
                    motion_emergency_stop("Obstacle detected");
                }
            } else {
                g_motion.obstacle_front = false;
                g_motion.estop_active   = false;
                hal_gpio_set(HAL_LED_WARN, false);
            }
        }

        // Check ESTOP bus events
        if (xQueueReceive(estop_q, &bus_msg, 0) == pdTRUE) {
            motion_emergency_stop("ESTOP from system bus");
        }

        // Process command queue
        if (xQueueReceive(g_motion.cmd_queue, &msg, 0) == pdTRUE) {
            motion_apply_cmd(&msg);
        }

        // Ramp wheels each tick
        motion_ramp_step();

        // Publish motor status
        typedef struct { float speeds[4]; motion_cmd_t cmd; } motor_status_t;
        motor_status_t status = {
            .cmd = g_motion.current_cmd,
            .speeds = {
                g_motion.wheels[0].current_speed,
                g_motion.wheels[1].current_speed,
                g_motion.wheels[2].current_speed,
                g_motion.wheels[3].current_speed,
            }
        };
        bus_publish(TOPIC_MOTOR_STATUS, &status, sizeof(status));
    }
}

// ─── Service Lifecycle ────────────────────────────────────
hk_status_t motion_service_init(void) {
    // Initialize traction factors
    for (int i = 0; i < WHEEL_COUNT; i++) {
        g_motion.wheels[i].traction_factor = 1.0f;
        g_motion.wheels[i].current_speed   = 0.0f;
        g_motion.wheels[i].target_speed    = 0.0f;
        g_motion.wheels[i].pwm_ch          = WHEEL_HW[i].pwm_ch;
        g_motion.wheels[i].dir_a           = WHEEL_HW[i].dir_a;
        g_motion.wheels[i].dir_b           = WHEEL_HW[i].dir_b;
    }
    g_motion.cmd_queue = xQueueCreate(16, sizeof(motion_msg_t));
    g_motion.accel_profile = ACCEL_SMOOTH;
    return g_motion.cmd_queue ? HK_OK : HK_ERR_NOMEM;
}

hk_status_t motion_service_start(void) {
    hk_task_descriptor_t* td;
    return hk_task_create("svc_motion", motion_task,
                           STACK_MOTION, NULL, PRIORITY_MOTION,
                           CORE_KERNEL, &td);
}

hk_status_t motion_service_stop(void) {
    motion_emergency_stop("Service stopping");
    return HK_OK;
}

hk_status_t motion_send_cmd(motion_msg_t* cmd) {
    if (!cmd) return HK_ERR_INVALID;
    if (xQueueSend(g_motion.cmd_queue, cmd, pdMS_TO_TICKS(10)) == pdTRUE)
        return HK_OK;
    return HK_ERR_BUSY;
}

float motion_get_speed(void) { return g_motion.speed; }
