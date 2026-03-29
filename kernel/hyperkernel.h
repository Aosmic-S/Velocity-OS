/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║         AOSMIC HYPERKERNEL OS — VelocityOS v1.0          ║
 * ║              HyperKernel Core Definitions                ║
 * ║     Powered by Aosmic Studio × Absolute Tech             ║
 * ╚══════════════════════════════════════════════════════════╝
 */

#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <stdint.h>
#include <stdbool.h>
#include <string.h>

// ─── Kernel Version ───────────────────────────────────────
#define HYPERKERNEL_VERSION_MAJOR  1
#define HYPERKERNEL_VERSION_MINOR  0
#define HYPERKERNEL_VERSION_PATCH  0
#define HYPERKERNEL_VERSION_STR    "1.0.0"
#define OS_NAME                    "VelocityOS"
#define OS_CODENAME                "Aosmic HyperKernel"

// ─── System Limits ────────────────────────────────────────
#define HK_MAX_TASKS               32
#define HK_MAX_SERVICES            16
#define HK_MAX_MODULES             24
#define HK_MAX_TOPICS              64
#define HK_MAX_SUBSCRIBERS         16
#define HK_TASK_NAME_LEN           32
#define HK_MSG_PAYLOAD_SIZE        256
#define HK_IPC_QUEUE_DEPTH         32
#define HK_SERVICE_QUEUE_DEPTH     16
#define HK_TELEMETRY_RATE_HZ       20

// ─── Task Priorities ──────────────────────────────────────
#define PRIORITY_KERNEL            (configMAX_PRIORITIES - 1)  // 24
#define PRIORITY_CRITICAL          (configMAX_PRIORITIES - 2)  // 23
#define PRIORITY_MOTION            (configMAX_PRIORITIES - 3)  // 22
#define PRIORITY_SENSOR            (configMAX_PRIORITIES - 4)  // 21
#define PRIORITY_AI                (configMAX_PRIORITIES - 5)  // 20
#define PRIORITY_COMM              (configMAX_PRIORITIES - 6)  // 19
#define PRIORITY_VISION            (configMAX_PRIORITIES - 7)  // 18
#define PRIORITY_POWER             (configMAX_PRIORITIES - 8)  // 17
#define PRIORITY_TELEMETRY         (configMAX_PRIORITIES - 9)  // 16
#define PRIORITY_WEBSERVER         (configMAX_PRIORITIES - 10) // 15
#define PRIORITY_IDLE              1

// ─── Core Assignments ─────────────────────────────────────
#define CORE_KERNEL                0
#define CORE_SERVICES              1
#define CORE_ANY                   tskNO_AFFINITY

// ─── Stack Sizes ──────────────────────────────────────────
#define STACK_KERNEL               8192
#define STACK_MOTION               6144
#define STACK_SENSOR               4096
#define STACK_AI                   8192
#define STACK_COMM                 8192
#define STACK_WEBSERVER            16384
#define STACK_TELEMETRY            4096
#define STACK_POWER                3072

// ─── Kernel Return Codes ──────────────────────────────────
typedef enum {
    HK_OK            = 0,
    HK_ERR_NOMEM     = -1,
    HK_ERR_NOTFOUND  = -2,
    HK_ERR_BUSY      = -3,
    HK_ERR_TIMEOUT   = -4,
    HK_ERR_INVALID   = -5,
    HK_ERR_LIMIT     = -6,
    HK_ERR_DENIED    = -7,
    HK_ERR_FAULT     = -8,
} hk_status_t;

// ─── Task States ──────────────────────────────────────────
typedef enum {
    TASK_STATE_INIT     = 0,
    TASK_STATE_READY    = 1,
    TASK_STATE_RUNNING  = 2,
    TASK_STATE_BLOCKED  = 3,
    TASK_STATE_SUSPENDED= 4,
    TASK_STATE_ZOMBIE   = 5,
    TASK_STATE_DEAD     = 6,
} hk_task_state_t;

// ─── Service States ───────────────────────────────────────
typedef enum {
    SERVICE_STOPPED   = 0,
    SERVICE_STARTING  = 1,
    SERVICE_RUNNING   = 2,
    SERVICE_PAUSED    = 3,
    SERVICE_STOPPING  = 4,
    SERVICE_ERROR     = 5,
} hk_service_state_t;

// ─── Service IDs ──────────────────────────────────────────
typedef enum {
    SVC_MOTION        = 0,
    SVC_SENSOR        = 1,
    SVC_COMMUNICATION = 2,
    SVC_VISION        = 3,
    SVC_POWER         = 4,
    SVC_AI_DECISION   = 5,
    SVC_TELEMETRY     = 6,
    SVC_SECURITY      = 7,
    SVC_COUNT
} service_id_t;

// ─── Module Types ─────────────────────────────────────────
typedef enum {
    MODULE_MOTOR       = 0,
    MODULE_SENSOR_US   = 1,
    MODULE_SENSOR_GAS  = 2,
    MODULE_ACTUATOR    = 3,
    MODULE_CAMERA      = 4,
    MODULE_IMU         = 5,
    MODULE_PUMP        = 6,
    MODULE_CUSTOM      = 7,
} module_type_t;

// ─── Message Bus Topic IDs ────────────────────────────────
typedef enum {
    TOPIC_MOTOR_CMD       = 0,
    TOPIC_MOTOR_STATUS    = 1,
    TOPIC_SENSOR_DATA     = 2,
    TOPIC_SENSOR_ALERT    = 3,
    TOPIC_SYSTEM_ALERT    = 4,
    TOPIC_AI_DECISION     = 5,
    TOPIC_POWER_STATUS    = 6,
    TOPIC_TELEMETRY       = 7,
    TOPIC_OBSTACLE        = 8,
    TOPIC_ESTOP           = 9,
    TOPIC_OTA_STATUS      = 10,
    TOPIC_MODULE_REGISTRY = 11,
    TOPIC_COUNT
} topic_id_t;

// ─── IPC Message Structure ────────────────────────────────
typedef struct {
    uint32_t    msg_id;
    topic_id_t  topic;
    uint32_t    sender_id;
    int64_t     timestamp_us;
    uint16_t    payload_len;
    uint8_t     payload[HK_MSG_PAYLOAD_SIZE];
    uint8_t     priority;
    bool        requires_ack;
} hk_message_t;

// ─── Task Descriptor ──────────────────────────────────────
typedef struct {
    char            name[HK_TASK_NAME_LEN];
    uint32_t        task_id;
    TaskHandle_t    handle;
    hk_task_state_t state;
    uint8_t         priority;
    uint32_t        stack_size;
    uint32_t        stack_hwm;     // High Water Mark
    uint32_t        cpu_core;
    uint64_t        runtime_ticks;
    uint32_t        wakeups;
    int64_t         created_at;
    QueueHandle_t   inbox;
} hk_task_descriptor_t;

// ─── Service Descriptor ───────────────────────────────────
typedef struct {
    char                name[HK_TASK_NAME_LEN];
    service_id_t        id;
    hk_service_state_t  state;
    hk_task_descriptor_t* task;
    uint32_t            restart_count;
    int64_t             uptime_start;
    float               power_mw;
    bool                critical;
    hk_status_t (*init)(void);
    hk_status_t (*start)(void);
    hk_status_t (*stop)(void);
    hk_status_t (*health_check)(void);
} hk_service_t;

// ─── Module Descriptor ────────────────────────────────────
typedef struct {
    char            name[HK_TASK_NAME_LEN];
    uint32_t        module_id;
    module_type_t   type;
    uint32_t        capabilities;   // Bitmask
    float           power_mw;
    bool            active;
    bool            hot_pluggable;
    int64_t         registered_at;
    void*           driver_handle;
    hk_status_t (*probe)(void);
    hk_status_t (*enable)(void);
    hk_status_t (*disable)(void);
} hk_module_t;

// ─── Kernel State ─────────────────────────────────────────
typedef struct {
    bool        initialized;
    bool        running;
    int64_t     boot_time_us;
    uint32_t    uptime_seconds;
    uint32_t    task_count;
    uint32_t    service_count;
    uint32_t    module_count;
    uint32_t    msg_total_sent;
    uint32_t    msg_total_dropped;
    float       cpu_usage_core0;
    float       cpu_usage_core1;
    uint32_t    free_heap;
    uint32_t    min_free_heap;
    float       battery_voltage;
    float       temperature_c;
    SemaphoreHandle_t registry_lock;
} hk_kernel_state_t;

// ─── Kernel API ───────────────────────────────────────────
extern hk_kernel_state_t g_kernel;
extern hk_task_descriptor_t g_tasks[HK_MAX_TASKS];
extern hk_service_t g_services[HK_MAX_SERVICES];
extern hk_module_t g_modules[HK_MAX_MODULES];

// Kernel lifecycle
hk_status_t hk_init(void);
hk_status_t hk_start(void);
void        hk_panic(const char* reason);
void        hk_boot_banner(void);

// Task management
hk_status_t hk_task_create(const char* name, TaskFunction_t func,
                             uint32_t stack, void* arg, uint8_t priority,
                             uint32_t core, hk_task_descriptor_t** out);
hk_status_t hk_task_suspend(uint32_t task_id);
hk_status_t hk_task_resume(uint32_t task_id);
hk_status_t hk_task_delete(uint32_t task_id);
hk_task_descriptor_t* hk_task_find(const char* name);

// Service management
hk_status_t hk_service_register(hk_service_t* svc);
hk_status_t hk_service_start(service_id_t id);
hk_status_t hk_service_stop(service_id_t id);
hk_service_t* hk_service_get(service_id_t id);

// Module management
hk_status_t hk_module_register(hk_module_t* mod);
hk_status_t hk_module_enable(uint32_t module_id);
hk_status_t hk_module_disable(uint32_t module_id);
hk_module_t* hk_module_find(const char* name);

// IPC
hk_status_t hk_send(uint32_t target_id, hk_message_t* msg, TickType_t timeout);
hk_status_t hk_recv(QueueHandle_t inbox, hk_message_t* msg, TickType_t timeout);
uint32_t    hk_new_msg_id(void);

// Software interrupt / event system
hk_status_t hk_raise_event(topic_id_t topic, void* data, uint16_t len);
hk_status_t hk_estop(const char* reason);

// Telemetry helpers
void hk_update_cpu_usage(void);
float hk_get_cpu_usage(uint8_t core);
uint32_t hk_get_free_heap(void);
