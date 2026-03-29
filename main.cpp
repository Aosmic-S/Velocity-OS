/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║         AOSMIC HYPERKERNEL OS — VelocityOS v1.0          ║
 * ║                  main.cpp — OS Entry Point               ║
 * ║     Powered by Aosmic Studio × Absolute Tech             ║
 * ╚══════════════════════════════════════════════════════════╝
 */

#include "kernel/hyperkernel.h"
#include "hal/hal.h"
#include "bus/message_bus.h"
#include "security/security.h"
#include "ota/ota_manager.h"
#include "modules/modules.h"
#include "services/motion_service.h"
#include "services/sensor_service.h"
#include "services/power_service.h"
#include "services/telemetry_service.h"
#include "webserver/comm_service.h"
#include "ai/ai_engine.h"
#include "esp_log.h"

static const char* TAG = "VelocityOS";

// ─── Service Descriptors ──────────────────────────────────
static hk_service_t svc_motion = {
    .name     = "MotionService",
    .id       = SVC_MOTION,
    .critical = true,
    .power_mw = 50.0f,
    .init     = motion_service_init,
    .start    = motion_service_start,
    .stop     = motion_service_stop,
};
static hk_service_t svc_sensor = {
    .name     = "SensorService",
    .id       = SVC_SENSOR,
    .critical = true,
    .power_mw = 20.0f,
    .init     = sensor_service_init,
    .start    = sensor_service_start,
};
static hk_service_t svc_power = {
    .name     = "PowerService",
    .id       = SVC_POWER,
    .critical = true,
    .power_mw = 10.0f,
    .init     = power_service_init,
    .start    = power_service_start,
};
static hk_service_t svc_telemetry = {
    .name     = "TelemetryService",
    .id       = SVC_TELEMETRY,
    .critical = false,
    .power_mw = 5.0f,
    .init     = telemetry_service_init,
    .start    = telemetry_service_start,
};
static hk_service_t svc_comm = {
    .name     = "CommService",
    .id       = SVC_COMMUNICATION,
    .critical = false,
    .power_mw = 200.0f,
    .init     = comm_service_init,
    .start    = comm_service_start,
};
static hk_service_t svc_ai = {
    .name     = "AIDecisionEngine",
    .id       = SVC_AI_DECISION,
    .critical = false,
    .power_mw = 30.0f,
    .init     = ai_service_init,
    .start    = ai_service_start,
    .stop     = ai_service_stop,
};

// ─── OS Boot Sequence ─────────────────────────────────────
extern "C" void app_main(void) {

    // ── Phase 1: HyperKernel Init ─────────────────────────
    ESP_LOGI(TAG, "Phase 1: HyperKernel initialization");
    hk_init();

    // ── Phase 2: Hardware Abstraction Layer ───────────────
    ESP_LOGI(TAG, "Phase 2: HAL initialization");
    if (hal_init() != HK_OK) {
        hk_panic("HAL initialization failed");
    }

    // ── Phase 3: Message Bus ──────────────────────────────
    ESP_LOGI(TAG, "Phase 3: Message Bus initialization");
    if (bus_init() != HK_OK) {
        hk_panic("Message Bus initialization failed");
    }

    // ── Phase 4: Security Layer ───────────────────────────
    ESP_LOGI(TAG, "Phase 4: Security Layer initialization");
    security_init();

    // ── Phase 5: OTA System ───────────────────────────────
    ESP_LOGI(TAG, "Phase 5: OTA System initialization");
    ota_init();

    // ── Phase 6: Module Registry ──────────────────────────
    ESP_LOGI(TAG, "Phase 6: Registering hardware modules");
    modules_register_all();

    // ── Phase 7: Service Registration ────────────────────
    ESP_LOGI(TAG, "Phase 7: Registering services");
    hk_service_register(&svc_motion);
    hk_service_register(&svc_sensor);
    hk_service_register(&svc_power);
    hk_service_register(&svc_telemetry);
    hk_service_register(&svc_comm);
    hk_service_register(&svc_ai);

    // ── Phase 8: Start Critical Services ─────────────────
    ESP_LOGI(TAG, "Phase 8: Starting critical services");
    hk_service_start(SVC_POWER);
    vTaskDelay(pdMS_TO_TICKS(100));
    hk_service_start(SVC_SENSOR);
    vTaskDelay(pdMS_TO_TICKS(100));
    hk_service_start(SVC_MOTION);
    vTaskDelay(pdMS_TO_TICKS(100));

    // ── Phase 9: Start AI + Telemetry ────────────────────
    ESP_LOGI(TAG, "Phase 9: Starting AI + Telemetry");
    hk_service_start(SVC_AI_DECISION);
    hk_service_start(SVC_TELEMETRY);
    vTaskDelay(pdMS_TO_TICKS(200));

    // ── Phase 10: Start Comm (WiFi + WebServer) ───────────
    ESP_LOGI(TAG, "Phase 10: Starting Communication Service");
    hk_service_start(SVC_COMMUNICATION);

    // ── Phase 11: Kernel Running ──────────────────────────
    hk_start();

    ESP_LOGI(TAG, "\033[1;32m");
    ESP_LOGI(TAG, "╔══════════════════════════════════════╗");
    ESP_LOGI(TAG, "║   VelocityOS BOOT COMPLETE ✓         ║");
    ESP_LOGI(TAG, "║   Services: %2u  Modules: %2u          ║",
             g_kernel.service_count, g_kernel.module_count);
    ESP_LOGI(TAG, "║   Free Heap: %u bytes               ║", g_kernel.free_heap);
    ESP_LOGI(TAG, "║   Dashboard: http://[device-ip]/    ║");
    ESP_LOGI(TAG, "╚══════════════════════════════════════╝");
    ESP_LOGI(TAG, "\033[0m");

    // ── Kernel Monitor Loop ───────────────────────────────
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(5000));
        hk_update_cpu_usage();

        // Health-check all critical services
        for (int i = 0; i < SVC_COUNT; i++) {
            hk_service_t* svc = &g_services[i];
            if (!svc->critical || svc->id == 0) continue;
            if (svc->state == SERVICE_ERROR) {
                ESP_LOGW(TAG, "Service %s in ERROR state — restarting", svc->name);
                svc->restart_count++;
                hk_service_start((service_id_t)i);
            }
        }

        ESP_LOGD(TAG, "Kernel alive | heap=%u | tasks=%u | msgs=%u",
                 g_kernel.free_heap,
                 g_kernel.task_count,
                 g_kernel.msg_total_sent);
    }
}
