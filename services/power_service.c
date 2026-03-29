/**
 * Power Intelligence Service — VelocityOS
 * Dynamic power allocation, battery prediction, thermal monitoring
 */

#include "../kernel/hyperkernel.h"
#include "../hal/hal.h"
#include "../bus/message_bus.h"
#include "esp_log.h"

static const char* TAG = "PowerSvc";

#define BATT_FULL_V      8.4f   // 2S LiPo full
#define BATT_EMPTY_V     6.0f   // Cutoff
#define BATT_CAPACITY_MAH 3000.0f
#define THERMAL_WARN_C   65.0f
#define THERMAL_CRIT_C   80.0f

typedef struct {
    float voltage;
    float percentage;
    float estimated_runtime_min;
    float current_draw_ma;
    float temperature_c;
    bool  charging;
    bool  thermal_warning;
    bool  low_battery;
    float module_power[HK_MAX_MODULES];
    float total_power_mw;
} power_status_t;

static power_status_t s_power = {0};

static float estimate_runtime(float voltage, float current_ma) {
    if (current_ma < 1.0f) return 9999.0f;
    float pct = (voltage - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V);
    if (pct < 0) pct = 0;
    float remaining_mah = BATT_CAPACITY_MAH * pct;
    return (remaining_mah / current_ma) * 60.0f; // minutes
}

static float batt_pct(float v) {
    float p = (v - BATT_EMPTY_V) / (BATT_FULL_V - BATT_EMPTY_V) * 100.0f;
    if (p < 0) p = 0;
    if (p > 100) p = 100;
    return p;
}

static void power_task(void* arg) {
    TickType_t last_wake = xTaskGetTickCount();
    ESP_LOGI(TAG, "Power Intelligence Service online");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(1000)); // 1Hz

        s_power.voltage     = hal_battery_voltage();
        s_power.percentage  = batt_pct(s_power.voltage);
        s_power.temperature_c = 30.0f + (esp_timer_get_time() % 1000) / 100.0f; // Simulated

        // Calculate total module power draw
        s_power.total_power_mw = 0;
        for (int i = 0; i < g_kernel.module_count; i++) {
            if (g_modules[i].active) {
                s_power.total_power_mw += g_modules[i].power_mw;
            }
        }
        // Add base system power
        s_power.total_power_mw += 500.0f; // ESP32-S3 base ~500mW

        // Estimate current draw from power
        s_power.current_draw_ma = s_power.total_power_mw / s_power.voltage;
        s_power.estimated_runtime_min = estimate_runtime(s_power.voltage, s_power.current_draw_ma);

        // Warnings
        s_power.low_battery    = (s_power.voltage < 6.8f);
        s_power.thermal_warning = (s_power.temperature_c > THERMAL_WARN_C);

        if (s_power.thermal_warning) {
            char alert[64];
            snprintf(alert, sizeof(alert), "THERMAL: %.1f°C", s_power.temperature_c);
            bus_publish(TOPIC_SYSTEM_ALERT, alert, strlen(alert) + 1);
        }

        // Publish power status
        bus_publish(TOPIC_POWER_STATUS, &s_power, sizeof(s_power));

        // Update kernel state
        g_kernel.battery_voltage = s_power.voltage;
        g_kernel.temperature_c   = s_power.temperature_c;

        ESP_LOGD(TAG, "Batt: %.2fV (%.0f%%) Runtime: %.1fmin Temp: %.1f°C",
                 s_power.voltage, s_power.percentage,
                 s_power.estimated_runtime_min, s_power.temperature_c);
    }
}

hk_status_t power_service_init(void) { return HK_OK; }

hk_status_t power_service_start(void) {
    hk_task_descriptor_t* td;
    return hk_task_create("svc_power", power_task, STACK_POWER,
                           NULL, PRIORITY_POWER, CORE_SERVICES, &td);
}
