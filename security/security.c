/**
 * Security Layer Implementation
 */
#include "security.h"
#include "esp_log.h"
#include "esp_random.h"
#include <stdio.h>

static const char* TAG = "Security";
security_state_t g_security = {0};

static void generate_token(char* buf) {
    const char charset[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789";
    for (int i = 0; i < SEC_TOKEN_LEN; i++) {
        buf[i] = charset[esp_random() % (sizeof(charset) - 1)];
    }
    buf[SEC_TOKEN_LEN] = '\0';
}

hk_status_t security_init(void) {
    memset(&g_security, 0, sizeof(g_security));
    g_security.lock = xSemaphoreCreateMutex();
    // Default admin token — should be changed via config
    strncpy(g_security.admin_token, "VELOCITY_ADMIN_SECRET_2024", SEC_TOKEN_LEN);
    ESP_LOGI(TAG, "Security layer initialized");
    return HK_OK;
}

hk_status_t security_create_session(const char* password, char* out_token, const char* client_ip) {
    if (security_is_locked()) {
        ESP_LOGW(TAG, "Auth rejected: system locked");
        return HK_ERR_DENIED;
    }

    bool valid = (strcmp(password, g_security.admin_token) == 0);
    if (!valid) {
        g_security.failed_auths++;
        ESP_LOGW(TAG, "Auth failed from %s (%u total)", client_ip, g_security.failed_auths);
        if (g_security.failed_auths >= SEC_MAX_FAILED_AUTH) {
            security_lockout();
        }
        return HK_ERR_DENIED;
    }

    g_security.failed_auths = 0;
    xSemaphoreTake(g_security.lock, portMAX_DELAY);

    // Find free session slot
    for (int i = 0; i < SEC_MAX_SESSIONS; i++) {
        if (!g_security.sessions[i].valid) {
            generate_token(g_security.sessions[i].token);
            g_security.sessions[i].permissions = SEC_PERM_ADMIN;
            g_security.sessions[i].created_at  = esp_timer_get_time();
            g_security.sessions[i].last_used   = esp_timer_get_time();
            g_security.sessions[i].valid       = true;
            strncpy(g_security.sessions[i].client_ip, client_ip, 19);
            strncpy(out_token, g_security.sessions[i].token, SEC_TOKEN_LEN + 1);
            g_security.session_count++;
            xSemaphoreGive(g_security.lock);
            ESP_LOGI(TAG, "Session created for %s", client_ip);
            return HK_OK;
        }
    }
    xSemaphoreGive(g_security.lock);
    return HK_ERR_LIMIT;
}

bool security_authenticate(const char* token, uint8_t required_perm) {
    if (!token) return false;
    if (security_is_locked()) return false;

    int64_t now = esp_timer_get_time();
    for (int i = 0; i < SEC_MAX_SESSIONS; i++) {
        sec_session_t* s = &g_security.sessions[i];
        if (!s->valid) continue;
        if (strncmp(s->token, token, SEC_TOKEN_LEN) != 0) continue;

        // Check expiry
        if ((now - s->created_at) / 1000 > SEC_TOKEN_EXPIRE_MS) {
            s->valid = false;
            ESP_LOGI(TAG, "Session expired");
            return false;
        }

        s->last_used = now;
        return (s->permissions & required_perm) != 0;
    }
    return false;
}

void security_lockout(void) {
    g_security.locked       = true;
    g_security.lockout_until = esp_timer_get_time() + (int64_t)SEC_LOCKOUT_MS * 1000;
    ESP_LOGE(TAG, "SECURITY LOCKOUT: too many failed auths");
    bus_publish(TOPIC_SYSTEM_ALERT, "SECURITY: LOCKOUT ACTIVE", 25);
}

bool security_is_locked(void) {
    if (!g_security.locked) return false;
    if (esp_timer_get_time() >= g_security.lockout_until) {
        g_security.locked       = false;
        g_security.failed_auths = 0;
        return false;
    }
    return true;
}

bool security_validate_cmd(const char* token, const char* cmd) {
    if (!security_authenticate(token, SEC_PERM_MOTION)) return false;
    // Block dangerous raw commands
    if (strstr(cmd, "OVERRIDE_ESTOP") && !security_authenticate(token, SEC_PERM_ADMIN))
        return false;
    return true;
}
