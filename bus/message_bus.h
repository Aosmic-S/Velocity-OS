/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║         AOSMIC HYPERKERNEL OS — VelocityOS v1.0          ║
 * ║         Central Message Bus — Pub/Sub Architecture       ║
 * ╚══════════════════════════════════════════════════════════╝
 */

#pragma once
#include "../kernel/hyperkernel.h"

// ─── Bus Config ───────────────────────────────────────────
#define BUS_MAX_SUBSCRIBERS     (HK_MAX_SUBSCRIBERS * TOPIC_COUNT)
#define BUS_QUEUE_DEPTH         64
#define BUS_TOPIC_NAME_LEN      48

// ─── Topic Strings ────────────────────────────────────────
static const char* TOPIC_STRINGS[] = {
    "/motor/cmd",
    "/motor/status",
    "/sensor/data",
    "/sensor/alert",
    "/system/alert",
    "/ai/decision",
    "/power/status",
    "/telemetry",
    "/obstacle",
    "/estop",
    "/ota/status",
    "/module/registry",
};

// ─── Subscriber Entry ─────────────────────────────────────
typedef struct {
    topic_id_t      topic;
    QueueHandle_t   queue;
    char            subscriber_name[HK_TASK_NAME_LEN];
    bool            active;
    uint32_t        received_count;
    uint32_t        dropped_count;
} bus_subscriber_t;

// ─── Topic Stats ──────────────────────────────────────────
typedef struct {
    topic_id_t  id;
    uint32_t    published;
    uint32_t    dropped;
    int64_t     last_published;
    float       rate_hz;
} bus_topic_stats_t;

// ─── Message Bus State ────────────────────────────────────
typedef struct {
    bool                initialized;
    bus_subscriber_t    subscribers[BUS_MAX_SUBSCRIBERS];
    bus_topic_stats_t   topic_stats[TOPIC_COUNT];
    uint32_t            sub_count;
    uint32_t            total_published;
    uint32_t            total_dropped;
    SemaphoreHandle_t   lock;
    QueueHandle_t       main_bus;    // Central dispatch queue
    TaskHandle_t        dispatcher;  // Background dispatch task
} message_bus_t;

extern message_bus_t g_bus;

// ─── Bus API ──────────────────────────────────────────────
hk_status_t bus_init(void);
hk_status_t bus_subscribe(topic_id_t topic, const char* name, QueueHandle_t* out_queue);
hk_status_t bus_unsubscribe(topic_id_t topic, QueueHandle_t queue);
hk_status_t bus_publish(topic_id_t topic, void* data, uint16_t len);
hk_status_t bus_publish_msg(hk_message_t* msg);
const char* bus_topic_name(topic_id_t topic);
void        bus_get_stats(bus_topic_stats_t* out, uint8_t count);
