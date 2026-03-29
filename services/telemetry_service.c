/**
 * Real-Time Telemetry Engine — VelocityOS
 * High-frequency data streaming with compression
 */

#include "../kernel/hyperkernel.h"
#include "../bus/message_bus.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>

static const char* TAG = "TelemetrySvc";

#define TELEM_JSON_SIZE   1024

typedef struct {
    // System
    uint32_t uptime_s;
    uint32_t free_heap;
    float    cpu0_pct;
    float    cpu1_pct;
    // Motion
    float    wheel_speeds[4];
    uint8_t  motion_state;
    float    obstacle_front;
    // Sensors
    float    gas_level;
    float    battery_v;
    float    temperature_c;
    bool     flame_detected;
    bool     smoke_detected;
    // AI
    uint8_t  ai_state;
    uint64_t decision_count;
    // Power
    float    runtime_min;
    float    power_mw;
    // Bus stats
    uint32_t msgs_published;
    uint32_t msgs_dropped;
    // Timestamp
    int64_t  timestamp_us;
} telemetry_frame_t;

static telemetry_frame_t s_frame = {0};
static QueueHandle_t s_sensor_sub;
static QueueHandle_t s_power_sub;
static QueueHandle_t s_motor_sub;
static QueueHandle_t s_ai_sub;

// Serialize to compact JSON for WebSocket
static int telem_to_json(char* buf, size_t maxlen) {
    return snprintf(buf, maxlen,
        "{"
        "\"ts\":%lld,"
        "\"up\":%u,"
        "\"heap\":%u,"
        "\"cpu0\":%.1f,"
        "\"cpu1\":%.1f,"
        "\"ws\":[%.1f,%.1f,%.1f,%.1f],"
        "\"ms\":%u,"
        "\"obs\":%.1f,"
        "\"gas\":%.3f,"
        "\"bv\":%.2f,"
        "\"tmp\":%.1f,"
        "\"flm\":%d,"
        "\"smk\":%d,"
        "\"ai\":%u,"
        "\"dc\":%llu,"
        "\"rt\":%.1f,"
        "\"pw\":%.1f,"
        "\"mp\":%u,"
        "\"md\":%u"
        "}",
        s_frame.timestamp_us,
        s_frame.uptime_s,
        s_frame.free_heap,
        s_frame.cpu0_pct,
        s_frame.cpu1_pct,
        s_frame.wheel_speeds[0], s_frame.wheel_speeds[1],
        s_frame.wheel_speeds[2], s_frame.wheel_speeds[3],
        s_frame.motion_state,
        s_frame.obstacle_front,
        s_frame.gas_level,
        s_frame.battery_v,
        s_frame.temperature_c,
        s_frame.flame_detected ? 1 : 0,
        s_frame.smoke_detected ? 1 : 0,
        s_frame.ai_state,
        s_frame.decision_count,
        s_frame.runtime_min,
        s_frame.power_mw,
        s_frame.msgs_published,
        s_frame.msgs_dropped
    );
}

static void telemetry_task(void* arg) {
    TickType_t last_wake = xTaskGetTickCount();
    static char json_buf[TELEM_JSON_SIZE];

    ESP_LOGI(TAG, "Telemetry Engine running at %dHz", HK_TELEMETRY_RATE_HZ);

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000 / HK_TELEMETRY_RATE_HZ));

        s_frame.timestamp_us = esp_timer_get_time();
        s_frame.uptime_s     = g_kernel.uptime_seconds;
        s_frame.free_heap    = g_kernel.free_heap;
        s_frame.cpu0_pct     = g_kernel.cpu_usage_core0;
        s_frame.cpu1_pct     = g_kernel.cpu_usage_core1;

        // Consume bus updates
        hk_message_t msg;

        // Sensor data
        typedef struct { float f,r,l,gas,bv,tmp; bool flame,smoke; } sd_t;
        if (xQueueReceive(s_sensor_sub, &msg, 0) == pdTRUE && msg.payload_len >= sizeof(sd_t)) {
            sd_t* sd = (sd_t*)msg.payload;
            s_frame.obstacle_front = sd->f;
            s_frame.gas_level      = sd->gas;
            s_frame.battery_v      = sd->bv;
            s_frame.temperature_c  = sd->tmp;
            s_frame.flame_detected = sd->flame;
            s_frame.smoke_detected = sd->smoke;
        }

        // Motor status
        typedef struct { float speeds[4]; uint8_t cmd; } mst_t;
        if (xQueueReceive(s_motor_sub, &msg, 0) == pdTRUE && msg.payload_len >= sizeof(mst_t)) {
            mst_t* ms = (mst_t*)msg.payload;
            memcpy(s_frame.wheel_speeds, ms->speeds, sizeof(ms->speeds));
            s_frame.motion_state = ms->cmd;
        }

        // AI state
        typedef struct { uint8_t state; char reason[48]; } aim_t;
        if (xQueueReceive(s_ai_sub, &msg, 0) == pdTRUE) {
            aim_t* am = (aim_t*)msg.payload;
            s_frame.ai_state = am->state;
        }

        // Power status
        typedef struct { float v,pct,rt,mw; bool lb; } ps_t;
        if (xQueueReceive(s_power_sub, &msg, 0) == pdTRUE && msg.payload_len >= sizeof(ps_t)) {
            ps_t* ps = (ps_t*)msg.payload;
            s_frame.battery_v    = ps->v;
            s_frame.runtime_min  = ps->rt;
            s_frame.power_mw     = ps->mw;
        }

        s_frame.msgs_published = g_bus.total_published;
        s_frame.msgs_dropped   = g_bus.total_dropped;

        hk_update_cpu_usage();

        // Publish telemetry frame to bus (for WebSocket handler)
        int len = telem_to_json(json_buf, TELEM_JSON_SIZE);
        if (len > 0) {
            bus_publish(TOPIC_TELEMETRY, json_buf, (uint16_t)(len + 1));
        }
    }
}

hk_status_t telemetry_service_init(void) {
    bus_subscribe(TOPIC_SENSOR_DATA,  "telem_sensor", &s_sensor_sub);
    bus_subscribe(TOPIC_MOTOR_STATUS, "telem_motor",  &s_motor_sub);
    bus_subscribe(TOPIC_AI_DECISION,  "telem_ai",     &s_ai_sub);
    bus_subscribe(TOPIC_POWER_STATUS, "telem_power",  &s_power_sub);
    return HK_OK;
}

hk_status_t telemetry_service_start(void) {
    hk_task_descriptor_t* td;
    return hk_task_create("svc_telem", telemetry_task, STACK_TELEMETRY,
                           NULL, PRIORITY_TELEMETRY, CORE_SERVICES, &td);
}
