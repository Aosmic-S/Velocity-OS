/**
 * Central Message Bus Implementation
 * High-performance non-blocking pub/sub for VelocityOS
 */

#include "message_bus.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "MessageBus";
message_bus_t g_bus = {0};

// ─── Dispatcher Task ──────────────────────────────────────
static void bus_dispatcher_task(void* arg) {
    hk_message_t msg;
    while (1) {
        if (xQueueReceive(g_bus.main_bus, &msg, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Fan out to all subscribers for this topic
            xSemaphoreTake(g_bus.lock, portMAX_DELAY);
            for (int i = 0; i < BUS_MAX_SUBSCRIBERS; i++) {
                bus_subscriber_t* sub = &g_bus.subscribers[i];
                if (!sub->active || sub->topic != msg.topic) continue;
                if (xQueueSend(sub->queue, &msg, 0) == pdTRUE) {
                    sub->received_count++;
                } else {
                    sub->dropped_count++;
                    g_bus.total_dropped++;
                }
            }
            // Update topic stats
            bus_topic_stats_t* stats = &g_bus.topic_stats[msg.topic];
            stats->published++;
            stats->last_published = esp_timer_get_time();
            g_bus.total_published++;
            xSemaphoreGive(g_bus.lock);
        }
    }
}

// ─── Init ─────────────────────────────────────────────────
hk_status_t bus_init(void) {
    if (g_bus.initialized) return HK_OK;

    memset(&g_bus, 0, sizeof(g_bus));
    g_bus.lock     = xSemaphoreCreateMutex();
    g_bus.main_bus = xQueueCreate(BUS_QUEUE_DEPTH, sizeof(hk_message_t));

    if (!g_bus.lock || !g_bus.main_bus) {
        ESP_LOGE(TAG, "Bus init failed: out of memory");
        return HK_ERR_NOMEM;
    }

    // Initialize topic stats
    for (int i = 0; i < TOPIC_COUNT; i++) {
        g_bus.topic_stats[i].id = (topic_id_t)i;
    }

    // Start dispatcher
    xTaskCreatePinnedToCore(bus_dispatcher_task, "bus_dispatch",
                             4096, NULL, PRIORITY_CRITICAL, &g_bus.dispatcher, CORE_KERNEL);

    g_bus.initialized = true;
    ESP_LOGI(TAG, "Message Bus online. Topics: %d, Queue depth: %d",
             TOPIC_COUNT, BUS_QUEUE_DEPTH);
    return HK_OK;
}

// ─── Subscribe ────────────────────────────────────────────
hk_status_t bus_subscribe(topic_id_t topic, const char* name, QueueHandle_t* out_queue) {
    if (topic >= TOPIC_COUNT || !name || !out_queue) return HK_ERR_INVALID;

    xSemaphoreTake(g_bus.lock, portMAX_DELAY);
    if (g_bus.sub_count >= BUS_MAX_SUBSCRIBERS) {
        xSemaphoreGive(g_bus.lock);
        return HK_ERR_LIMIT;
    }

    // Find free slot
    for (int i = 0; i < BUS_MAX_SUBSCRIBERS; i++) {
        if (!g_bus.subscribers[i].active) {
            g_bus.subscribers[i].topic  = topic;
            g_bus.subscribers[i].active = true;
            g_bus.subscribers[i].queue  = xQueueCreate(HK_SERVICE_QUEUE_DEPTH, sizeof(hk_message_t));
            strncpy(g_bus.subscribers[i].subscriber_name, name, HK_TASK_NAME_LEN - 1);
            g_bus.sub_count++;
            *out_queue = g_bus.subscribers[i].queue;
            xSemaphoreGive(g_bus.lock);
            ESP_LOGD(TAG, "Subscribed: %s → %s", name, bus_topic_name(topic));
            return HK_OK;
        }
    }
    xSemaphoreGive(g_bus.lock);
    return HK_ERR_LIMIT;
}

// ─── Publish ──────────────────────────────────────────────
hk_status_t bus_publish(topic_id_t topic, void* data, uint16_t len) {
    if (!g_bus.initialized) return HK_ERR_FAULT;
    if (topic >= TOPIC_COUNT) return HK_ERR_INVALID;
    if (len > HK_MSG_PAYLOAD_SIZE) len = HK_MSG_PAYLOAD_SIZE;

    hk_message_t msg = {0};
    msg.topic       = topic;
    msg.msg_id      = hk_new_msg_id();
    msg.timestamp_us = esp_timer_get_time();
    msg.payload_len = len;
    if (data && len > 0) memcpy(msg.payload, data, len);

    if (xQueueSend(g_bus.main_bus, &msg, 0) != pdTRUE) {
        g_bus.total_dropped++;
        return HK_ERR_BUSY;
    }
    return HK_OK;
}

hk_status_t bus_publish_msg(hk_message_t* msg) {
    if (!msg) return HK_ERR_INVALID;
    if (!g_bus.initialized) return HK_ERR_FAULT;
    msg->timestamp_us = esp_timer_get_time();
    if (xQueueSend(g_bus.main_bus, msg, 0) != pdTRUE) {
        g_bus.total_dropped++;
        return HK_ERR_BUSY;
    }
    return HK_OK;
}

hk_status_t hk_raise_event(topic_id_t topic, void* data, uint16_t len) {
    return bus_publish(topic, data, len);
}

const char* bus_topic_name(topic_id_t topic) {
    if (topic >= TOPIC_COUNT) return "unknown";
    return TOPIC_STRINGS[topic];
}

void bus_get_stats(bus_topic_stats_t* out, uint8_t count) {
    uint8_t n = count < TOPIC_COUNT ? count : TOPIC_COUNT;
    memcpy(out, g_bus.topic_stats, n * sizeof(bus_topic_stats_t));
}
