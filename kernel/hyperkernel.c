/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║         AOSMIC HYPERKERNEL OS — VelocityOS v1.0          ║
 * ║              HyperKernel Core Implementation             ║
 * ╚══════════════════════════════════════════════════════════╝
 */

#include "hyperkernel.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "esp_chip_info.h"
#include "esp_idf_version.h"

static const char* TAG = "HyperKernel";

// ─── Global State ─────────────────────────────────────────
hk_kernel_state_t      g_kernel   = {0};
hk_task_descriptor_t   g_tasks[HK_MAX_TASKS];
hk_service_t           g_services[HK_MAX_SERVICES];
hk_module_t            g_modules[HK_MAX_MODULES];

static uint32_t s_next_task_id   = 1000;
static uint32_t s_next_module_id = 2000;
static uint32_t s_next_msg_id    = 1;
static SemaphoreHandle_t s_task_lock;
static SemaphoreHandle_t s_module_lock;

// CPU idle counter for usage tracking
static uint32_t s_cpu_idle0_ticks = 0;
static uint32_t s_cpu_idle1_ticks = 0;

// ─── Boot Banner ──────────────────────────────────────────
void hk_boot_banner(void) {
    printf("\n\n");
    printf("  \033[36m╔══════════════════════════════════════════════════════════╗\033[0m\n");
    printf("  \033[36m║\033[0m  \033[1;33m    ██╗   ██╗███████╗██╗      ██████╗  ██████╗██╗████████╗██╗   ██╗\033[0m  \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m  \033[1;33m    ██║   ██║██╔════╝██║     ██╔═══██╗██╔════╝██║╚══██╔══╝╚██╗ ██╔╝\033[0m  \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m  \033[1;33m    ██║   ██║█████╗  ██║     ██║   ██║██║     ██║   ██║    ╚████╔╝ \033[0m  \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m  \033[1;33m    ╚██╗ ██╔╝██╔══╝  ██║     ██║   ██║██║     ██║   ██║     ╚██╔╝  \033[0m  \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m  \033[1;33m     ╚████╔╝ ███████╗███████╗╚██████╔╝╚██████╗██║   ██║      ██║   \033[0m  \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m  \033[1;33m      ╚═══╝  ╚══════╝╚══════╝ ╚═════╝  ╚═════╝╚═╝   ╚═╝      ╚═╝   \033[0m  \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m                                                              \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m   \033[1;32m  ██████╗ ███████╗    ██╗   ██╗ ██╗      ██████╗    \033[0m        \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m   \033[1;32m ██╔═══██╗██╔════╝    ██║   ██║███║     ██╔═══██╗   \033[0m        \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m   \033[1;32m ██║   ██║███████╗    ██║   ██║╚██║     ██║   ██║   \033[0m        \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m   \033[1;32m ██║   ██║╚════██║    ╚██╗ ██╔╝ ██║     ██║   ██║   \033[0m        \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m   \033[1;32m ╚██████╔╝███████║     ╚████╔╝  ██║     ╚██████╔╝   \033[0m        \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m   \033[1;32m  ╚═════╝ ╚══════╝      ╚═══╝   ╚═╝      ╚═════╝    \033[0m        \033[36m║\033[0m\n");
    printf("  \033[36m║\033[0m                                                              \033[36m║\033[0m\n");
    printf("  \033[36m╠══════════════════════════════════════════════════════════╣\033[0m\n");
    printf("  \033[36m║\033[0m  \033[1;37m  Kernel:  \033[0m\033[1;32mAosmic HyperKernel v%s\033[0m                          \033[36m║\033[0m\n", HYPERKERNEL_VERSION_STR);
    printf("  \033[36m║\033[0m  \033[1;37m  Target:  \033[0m\033[1;32mESP32-S3 | FreeRTOS %s\033[0m                 \033[36m║\033[0m\n", tskKERNEL_VERSION_NUMBER);
    printf("  \033[36m║\033[0m  \033[1;37m  Arch:    \033[0m\033[1;32mMicrokernel + SOA + Event-Driven\033[0m               \033[36m║\033[0m\n");
    printf("  \033[36m╠══════════════════════════════════════════════════════════╣\033[0m\n");
    printf("  \033[36m║\033[0m  \033[1;35m  Powered by Aosmic Studio × Absolute Tech\033[0m              \033[36m║\033[0m\n");
    printf("  \033[36m╚══════════════════════════════════════════════════════════╝\033[0m\n\n");
    vTaskDelay(pdMS_TO_TICKS(500));
}

// ─── Kernel Initialization ────────────────────────────────
hk_status_t hk_init(void) {
    if (g_kernel.initialized) return HK_OK;

    hk_boot_banner();

    ESP_LOGI(TAG, "Initializing HyperKernel...");

    // Zero all descriptors
    memset(g_tasks,    0, sizeof(g_tasks));
    memset(g_services, 0, sizeof(g_services));
    memset(g_modules,  0, sizeof(g_modules));

    // Create synchronization primitives
    s_task_lock   = xSemaphoreCreateMutex();
    s_module_lock = xSemaphoreCreateMutex();
    g_kernel.registry_lock = xSemaphoreCreateMutex();

    if (!s_task_lock || !s_module_lock || !g_kernel.registry_lock) {
        ESP_LOGE(TAG, "FATAL: Failed to create kernel mutexes");
        return HK_ERR_NOMEM;
    }

    g_kernel.boot_time_us  = esp_timer_get_time();
    g_kernel.free_heap     = esp_get_free_heap_size();
    g_kernel.min_free_heap = g_kernel.free_heap;
    g_kernel.task_count    = 0;
    g_kernel.service_count = 0;
    g_kernel.module_count  = 0;
    g_kernel.initialized   = true;

    ESP_LOGI(TAG, "HyperKernel initialized. Free heap: %u bytes", g_kernel.free_heap);
    return HK_OK;
}

hk_status_t hk_start(void) {
    if (!g_kernel.initialized) {
        ESP_LOGE(TAG, "Kernel not initialized!");
        return HK_ERR_FAULT;
    }
    g_kernel.running = true;
    ESP_LOGI(TAG, "HyperKernel RUNNING");
    return HK_OK;
}

void hk_panic(const char* reason) {
    ESP_LOGE(TAG, "\033[1;31m╔═══════════════════════════════╗\033[0m");
    ESP_LOGE(TAG, "\033[1;31m║       KERNEL PANIC            ║\033[0m");
    ESP_LOGE(TAG, "\033[1;31m║  %s\033[0m", reason);
    ESP_LOGE(TAG, "\033[1;31m╚═══════════════════════════════╝\033[0m");
    esp_restart();
}

// ─── Task Management ──────────────────────────────────────
hk_status_t hk_task_create(const char* name, TaskFunction_t func,
                             uint32_t stack, void* arg, uint8_t priority,
                             uint32_t core, hk_task_descriptor_t** out) {
    xSemaphoreTake(s_task_lock, portMAX_DELAY);

    if (g_kernel.task_count >= HK_MAX_TASKS) {
        xSemaphoreGive(s_task_lock);
        return HK_ERR_LIMIT;
    }

    // Find free slot
    int slot = -1;
    for (int i = 0; i < HK_MAX_TASKS; i++) {
        if (g_tasks[i].state == TASK_STATE_DEAD ||
            g_tasks[i].task_id == 0) {
            slot = i; break;
        }
    }
    if (slot < 0) { xSemaphoreGive(s_task_lock); return HK_ERR_LIMIT; }

    hk_task_descriptor_t* td = &g_tasks[slot];
    strncpy(td->name, name, HK_TASK_NAME_LEN - 1);
    td->task_id    = s_next_task_id++;
    td->state      = TASK_STATE_INIT;
    td->priority   = priority;
    td->stack_size = stack;
    td->cpu_core   = core;
    td->created_at = esp_timer_get_time();

    // Create inbox queue
    td->inbox = xQueueCreate(HK_IPC_QUEUE_DEPTH, sizeof(hk_message_t));
    if (!td->inbox) {
        xSemaphoreGive(s_task_lock);
        return HK_ERR_NOMEM;
    }

    // Create FreeRTOS task
    BaseType_t result;
    if (core == tskNO_AFFINITY) {
        result = xTaskCreate(func, name, stack, arg, priority, &td->handle);
    } else {
        result = xTaskCreatePinnedToCore(func, name, stack, arg, priority,
                                          &td->handle, core);
    }

    if (result != pdPASS) {
        vQueueDelete(td->inbox);
        memset(td, 0, sizeof(*td));
        xSemaphoreGive(s_task_lock);
        return HK_ERR_NOMEM;
    }

    td->state = TASK_STATE_READY;
    g_kernel.task_count++;

    if (out) *out = td;
    xSemaphoreGive(s_task_lock);

    ESP_LOGD(TAG, "Task created: %s (id=%u, core=%u, prio=%u)",
             name, td->task_id, core, priority);
    return HK_OK;
}

hk_status_t hk_task_suspend(uint32_t task_id) {
    for (int i = 0; i < HK_MAX_TASKS; i++) {
        if (g_tasks[i].task_id == task_id) {
            vTaskSuspend(g_tasks[i].handle);
            g_tasks[i].state = TASK_STATE_SUSPENDED;
            return HK_OK;
        }
    }
    return HK_ERR_NOTFOUND;
}

hk_status_t hk_task_resume(uint32_t task_id) {
    for (int i = 0; i < HK_MAX_TASKS; i++) {
        if (g_tasks[i].task_id == task_id) {
            vTaskResume(g_tasks[i].handle);
            g_tasks[i].state = TASK_STATE_RUNNING;
            return HK_OK;
        }
    }
    return HK_ERR_NOTFOUND;
}

hk_task_descriptor_t* hk_task_find(const char* name) {
    for (int i = 0; i < HK_MAX_TASKS; i++) {
        if (strcmp(g_tasks[i].name, name) == 0) return &g_tasks[i];
    }
    return NULL;
}

// ─── Service Management ───────────────────────────────────
hk_status_t hk_service_register(hk_service_t* svc) {
    if (!svc || svc->id >= SVC_COUNT) return HK_ERR_INVALID;
    xSemaphoreTake(g_kernel.registry_lock, portMAX_DELAY);

    memcpy(&g_services[svc->id], svc, sizeof(hk_service_t));
    g_services[svc->id].state = SERVICE_STOPPED;
    g_kernel.service_count++;

    xSemaphoreGive(g_kernel.registry_lock);
    ESP_LOGI(TAG, "Service registered: %s", svc->name);
    return HK_OK;
}

hk_status_t hk_service_start(service_id_t id) {
    if (id >= SVC_COUNT) return HK_ERR_INVALID;
    hk_service_t* svc = &g_services[id];
    if (svc->state == SERVICE_RUNNING) return HK_OK;

    svc->state = SERVICE_STARTING;
    if (svc->init) svc->init();
    if (svc->start) {
        hk_status_t r = svc->start();
        if (r != HK_OK) { svc->state = SERVICE_ERROR; return r; }
    }
    svc->state = SERVICE_RUNNING;
    svc->uptime_start = esp_timer_get_time();
    ESP_LOGI(TAG, "Service started: %s", svc->name);
    return HK_OK;
}

hk_status_t hk_service_stop(service_id_t id) {
    if (id >= SVC_COUNT) return HK_ERR_INVALID;
    hk_service_t* svc = &g_services[id];
    svc->state = SERVICE_STOPPING;
    if (svc->stop) svc->stop();
    svc->state = SERVICE_STOPPED;
    ESP_LOGI(TAG, "Service stopped: %s", svc->name);
    return HK_OK;
}

hk_service_t* hk_service_get(service_id_t id) {
    if (id >= SVC_COUNT) return NULL;
    return &g_services[id];
}

// ─── Module Management ────────────────────────────────────
hk_status_t hk_module_register(hk_module_t* mod) {
    xSemaphoreTake(s_module_lock, portMAX_DELAY);
    if (g_kernel.module_count >= HK_MAX_MODULES) {
        xSemaphoreGive(s_module_lock);
        return HK_ERR_LIMIT;
    }
    int slot = g_kernel.module_count;
    memcpy(&g_modules[slot], mod, sizeof(hk_module_t));
    g_modules[slot].module_id     = s_next_module_id++;
    g_modules[slot].registered_at = esp_timer_get_time();
    g_modules[slot].active        = false;
    g_kernel.module_count++;
    xSemaphoreGive(s_module_lock);

    // Announce on message bus
    hk_message_t msg = {0};
    msg.topic    = TOPIC_MODULE_REGISTRY;
    msg.msg_id   = hk_new_msg_id();
    msg.timestamp_us = esp_timer_get_time();
    strncpy((char*)msg.payload, mod->name, HK_MSG_PAYLOAD_SIZE - 1);
    msg.payload_len = strlen(mod->name);
    hk_raise_event(TOPIC_MODULE_REGISTRY, msg.payload, msg.payload_len);

    ESP_LOGI(TAG, "Module registered: %s (id=%u, power=%.1fmW)",
             mod->name, g_modules[slot].module_id, mod->power_mw);
    return HK_OK;
}

hk_status_t hk_module_enable(uint32_t module_id) {
    for (int i = 0; i < HK_MAX_MODULES; i++) {
        if (g_modules[i].module_id == module_id) {
            if (g_modules[i].probe && g_modules[i].probe() != HK_OK)
                return HK_ERR_FAULT;
            if (g_modules[i].enable) g_modules[i].enable();
            g_modules[i].active = true;
            return HK_OK;
        }
    }
    return HK_ERR_NOTFOUND;
}

hk_module_t* hk_module_find(const char* name) {
    for (int i = 0; i < HK_MAX_MODULES; i++) {
        if (strcmp(g_modules[i].name, name) == 0) return &g_modules[i];
    }
    return NULL;
}

// ─── IPC ──────────────────────────────────────────────────
uint32_t hk_new_msg_id(void) {
    return s_next_msg_id++;
}

hk_status_t hk_send(uint32_t target_id, hk_message_t* msg, TickType_t timeout) {
    for (int i = 0; i < HK_MAX_TASKS; i++) {
        if (g_tasks[i].task_id == target_id && g_tasks[i].inbox) {
            if (xQueueSend(g_tasks[i].inbox, msg, timeout) == pdTRUE) {
                g_kernel.msg_total_sent++;
                return HK_OK;
            }
            g_kernel.msg_total_dropped++;
            return HK_ERR_BUSY;
        }
    }
    return HK_ERR_NOTFOUND;
}

hk_status_t hk_recv(QueueHandle_t inbox, hk_message_t* msg, TickType_t timeout) {
    if (!inbox || !msg) return HK_ERR_INVALID;
    if (xQueueReceive(inbox, msg, timeout) == pdTRUE) return HK_OK;
    return HK_ERR_TIMEOUT;
}

// ─── Emergency Stop ───────────────────────────────────────
hk_status_t hk_estop(const char* reason) {
    ESP_LOGW(TAG, "\033[1;31m[ESTOP] %s\033[0m", reason);
    hk_raise_event(TOPIC_ESTOP, (void*)reason, strlen(reason) + 1);
    return HK_OK;
}

// ─── CPU Usage ────────────────────────────────────────────
void hk_update_cpu_usage(void) {
    g_kernel.free_heap = esp_get_free_heap_size();
    if (g_kernel.free_heap < g_kernel.min_free_heap)
        g_kernel.min_free_heap = g_kernel.free_heap;
    int64_t uptime = (esp_timer_get_time() - g_kernel.boot_time_us) / 1000000;
    g_kernel.uptime_seconds = (uint32_t)uptime;
}

uint32_t hk_get_free_heap(void) {
    return esp_get_free_heap_size();
}
