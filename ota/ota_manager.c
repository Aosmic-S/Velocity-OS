/**
 * OTA + Recovery System — VelocityOS
 * Dual partition firmware with safe rollback
 */

#include "../kernel/hyperkernel.h"
#include "../bus/message_bus.h"
#include "esp_ota_ops.h"
#include "esp_http_client.h"
#include "esp_log.h"

static const char* TAG = "OTA";

#define OTA_CHUNK_SIZE   4096

typedef enum {
    OTA_STATE_IDLE      = 0,
    OTA_STATE_CHECKING  = 1,
    OTA_STATE_RECEIVING = 2,
    OTA_STATE_VERIFYING = 3,
    OTA_STATE_APPLYING  = 4,
    OTA_STATE_REBOOTING = 5,
    OTA_STATE_ERROR     = 6,
    OTA_STATE_ROLLBACK  = 7,
} ota_state_t;

typedef struct {
    ota_state_t state;
    float       progress_pct;
    size_t      bytes_received;
    size_t      total_bytes;
    char        current_version[32];
    char        new_version[32];
    char        error_msg[64];
    bool        rollback_available;
} ota_status_t;

static ota_status_t s_ota = {0};

static void ota_publish_status(void) {
    typedef struct { uint8_t state; float pct; char ver[32]; } ota_msg_t;
    ota_msg_t msg = {
        .state = s_ota.state,
        .pct   = s_ota.progress_pct,
    };
    strncpy(msg.ver, s_ota.new_version, 31);
    bus_publish(TOPIC_OTA_STATUS, &msg, sizeof(msg));
}

hk_status_t ota_start_update(const char* url) {
    ESP_LOGI(TAG, "OTA starting from: %s", url);
    s_ota.state = OTA_STATE_CHECKING;
    ota_publish_status();

    const esp_partition_t* update_partition = esp_ota_get_next_update_partition(NULL);
    if (!update_partition) {
        snprintf(s_ota.error_msg, 64, "No OTA partition found");
        s_ota.state = OTA_STATE_ERROR;
        ota_publish_status();
        return HK_ERR_FAULT;
    }

    esp_http_client_config_t cfg = { .url = url, .timeout_ms = 30000 };
    esp_http_client_handle_t client = esp_http_client_init(&cfg);

    if (esp_http_client_open(client, 0) != ESP_OK) {
        s_ota.state = OTA_STATE_ERROR;
        esp_http_client_cleanup(client);
        return HK_ERR_FAULT;
    }

    s_ota.total_bytes = esp_http_client_fetch_headers(client);
    s_ota.state = OTA_STATE_RECEIVING;

    esp_ota_handle_t ota_handle;
    esp_ota_begin(update_partition, OTA_WITH_SEQUENTIAL_WRITES, &ota_handle);

    uint8_t* buf = malloc(OTA_CHUNK_SIZE);
    if (!buf) { s_ota.state = OTA_STATE_ERROR; return HK_ERR_NOMEM; }

    int read_len;
    while ((read_len = esp_http_client_read(client, (char*)buf, OTA_CHUNK_SIZE)) > 0) {
        if (esp_ota_write(ota_handle, buf, read_len) != ESP_OK) {
            s_ota.state = OTA_STATE_ERROR;
            break;
        }
        s_ota.bytes_received += read_len;
        if (s_ota.total_bytes > 0) {
            s_ota.progress_pct = (float)s_ota.bytes_received / s_ota.total_bytes * 100.0f;
        }
        ota_publish_status();
    }

    free(buf);
    esp_http_client_cleanup(client);

    if (s_ota.state == OTA_STATE_ERROR) return HK_ERR_FAULT;

    s_ota.state = OTA_STATE_VERIFYING;
    ota_publish_status();

    if (esp_ota_end(ota_handle) != ESP_OK) {
        s_ota.state = OTA_STATE_ERROR;
        snprintf(s_ota.error_msg, 64, "OTA verification failed");
        ota_publish_status();
        return HK_ERR_FAULT;
    }

    s_ota.state = OTA_STATE_APPLYING;
    if (esp_ota_set_boot_partition(update_partition) != ESP_OK) {
        s_ota.state = OTA_STATE_ERROR;
        return HK_ERR_FAULT;
    }

    s_ota.state = OTA_STATE_REBOOTING;
    ota_publish_status();
    ESP_LOGI(TAG, "OTA complete. Rebooting...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    esp_restart();
    return HK_OK;
}

hk_status_t ota_rollback(void) {
    ESP_LOGW(TAG, "OTA ROLLBACK initiated");
    s_ota.state = OTA_STATE_ROLLBACK;
    ota_publish_status();

    if (esp_ota_mark_app_invalid_rollback_and_reboot() == ESP_OK) {
        ESP_LOGI(TAG, "Rollback successful — rebooting");
        return HK_OK;
    }
    s_ota.state = OTA_STATE_ERROR;
    snprintf(s_ota.error_msg, 64, "Rollback failed");
    return HK_ERR_FAULT;
}

hk_status_t ota_mark_valid(void) {
    return esp_ota_mark_app_valid_cancel_rollback() == ESP_OK ? HK_OK : HK_ERR_FAULT;
}

ota_status_t* ota_get_status(void) { return &s_ota; }

hk_status_t ota_init(void) {
    s_ota.state = OTA_STATE_IDLE;
    const esp_app_desc_t* desc = esp_app_get_description();
    strncpy(s_ota.current_version, desc->version, 31);
    s_ota.rollback_available = (esp_ota_get_last_invalid_partition() != NULL);
    ESP_LOGI(TAG, "OTA ready. Current version: %s, Rollback: %s",
             s_ota.current_version, s_ota.rollback_available ? "YES" : "NO");
    ota_mark_valid(); // Mark current firmware as valid
    return HK_OK;
}
