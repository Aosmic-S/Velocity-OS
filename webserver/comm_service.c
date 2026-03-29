/**
 * Communication Service — VelocityOS
 * Async WebServer + WebSocket + REST API + ESP-NOW
 */

#include "../kernel/hyperkernel.h"
#include "../bus/message_bus.h"
#include "../security/security.h"
#include "../services/motion_service.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_http_server.h"

static const char* TAG = "CommSvc";

// WiFi config — override via menuconfig or NVS
#define WIFI_SSID       CONFIG_WIFI_SSID
#define WIFI_PASS       CONFIG_WIFI_PASSWORD
#define WIFI_AP_SSID    "Velo V1"
#define WIFI_AP_PASS    "VelocityBot"
#define WIFI_MAX_RETRY  5

static httpd_handle_t s_server = NULL;
static int            s_ws_fds[8] = {-1,-1,-1,-1,-1,-1,-1,-1};
static int            s_ws_count  = 0;

// ─── REST API Handlers ────────────────────────────────────

static esp_err_t handle_status(httpd_req_t* req) {
    char resp[512];
    snprintf(resp, sizeof(resp),
        "{\"os\":\"%s\",\"version\":\"%s\","
        "\"uptime\":%u,\"heap\":%u,"
        "\"services\":%u,\"modules\":%u,"
        "\"battery\":%.2f,\"ai_state\":\"%s\"}",
        OS_NAME, HYPERKERNEL_VERSION_STR,
        g_kernel.uptime_seconds, g_kernel.free_heap,
        g_kernel.service_count, g_kernel.module_count,
        g_kernel.battery_voltage, "IDLE"
    );
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t handle_move(httpd_req_t* req) {
    char buf[256] = {0};
    int  received = httpd_req_recv(req, buf, sizeof(buf) - 1);
    if (received <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No body");

    // Parse: {"cmd":"forward","speed":75,"token":"..."}
    motion_msg_t mcmd = { .profile = ACCEL_SMOOTH };

    if (strstr(buf, "\"forward\""))       mcmd.cmd = MOTION_CMD_FORWARD;
    else if (strstr(buf, "\"backward\"")) mcmd.cmd = MOTION_CMD_BACKWARD;
    else if (strstr(buf, "\"left\""))     mcmd.cmd = MOTION_CMD_PIVOT_LEFT;
    else if (strstr(buf, "\"right\""))    mcmd.cmd = MOTION_CMD_PIVOT_RIGHT;
    else if (strstr(buf, "\"stop\""))     mcmd.cmd = MOTION_CMD_STOP;
    else { return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Unknown cmd"); }

    // Parse speed
    char* sp = strstr(buf, "\"speed\":");
    if (sp) mcmd.speed = atof(sp + 8);
    else    mcmd.speed = 60.0f;

    // Publish to motion service via bus
    bus_publish(TOPIC_MOTOR_CMD, &mcmd, sizeof(mcmd));
    motion_send_cmd(&mcmd);

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, "{\"ok\":true}");
}

static esp_err_t handle_auth(httpd_req_t* req) {
    char buf[128] = {0};
    httpd_req_recv(req, buf, sizeof(buf) - 1);

    char* pass = strstr(buf, "\"password\":\"");
    if (!pass) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing password");
    pass += 12;
    char* end = strchr(pass, '"');
    if (end) *end = '\0';

    char token[SEC_TOKEN_LEN + 1] = {0};
    hk_status_t r = security_create_session(pass, token, "remote");

    char resp[128];
    if (r == HK_OK) snprintf(resp, sizeof(resp), "{\"token\":\"%s\"}", token);
    else            snprintf(resp, sizeof(resp), "{\"error\":\"auth failed\"}");

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, resp);
}

static esp_err_t handle_modules(httpd_req_t* req) {
    char buf[1024] = {0};
    int  n = 0;
    n += snprintf(buf + n, sizeof(buf) - n, "{\"modules\":[");
    for (int i = 0; i < g_kernel.module_count; i++) {
        hk_module_t* m = &g_modules[i];
        n += snprintf(buf + n, sizeof(buf) - n,
            "%s{\"id\":%u,\"name\":\"%s\",\"type\":%d,"
            "\"power\":%.1f,\"active\":%s}",
            i > 0 ? "," : "",
            m->module_id, m->name, m->type,
            m->power_mw, m->active ? "true" : "false");
    }
    n += snprintf(buf + n, sizeof(buf) - n, "]}");
    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, buf);
}

static esp_err_t handle_estop(httpd_req_t* req) {
    hk_estop("REST API ESTOP");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, "{\"estop\":true}");
}

static esp_err_t handle_ota_trigger(httpd_req_t* req) {
    char buf[256] = {0};
    httpd_req_recv(req, buf, sizeof(buf) - 1);
    // Trigger OTA in background task
    bus_publish(TOPIC_OTA_STATUS, buf, strlen(buf) + 1);
    return httpd_resp_sendstr(req, "{\"ota\":\"started\"}");
}

// ─── WebSocket Handler ────────────────────────────────────
static esp_err_t handle_ws(httpd_req_t* req) {
    if (req->method == HTTP_GET) {
        // New connection
        int fd = httpd_req_to_sockfd(req);
        for (int i = 0; i < 8; i++) {
            if (s_ws_fds[i] == -1) {
                s_ws_fds[i] = fd;
                s_ws_count++;
                ESP_LOGI(TAG, "WS client connected (fd=%d, total=%d)", fd, s_ws_count);
                break;
            }
        }
        return ESP_OK;
    }

    httpd_ws_frame_t frame = { .type = HTTPD_WS_TYPE_TEXT };
    uint8_t buf[HK_MSG_PAYLOAD_SIZE] = {0};
    frame.payload = buf;
    frame.len = 0;

    esp_err_t r = httpd_ws_recv_frame(req, &frame, sizeof(buf) - 1);
    if (r != ESP_OK) return r;

    buf[frame.len] = '\0';
    ESP_LOGD(TAG, "WS recv: %s", buf);

    // Parse WS command {"type":"move","cmd":"forward","speed":60}
    if (strstr((char*)buf, "\"move\"")) {
        motion_msg_t mcmd = { .profile = ACCEL_SMOOTH, .speed = 60.0f };
        if (strstr((char*)buf, "\"forward\""))       mcmd.cmd = MOTION_CMD_FORWARD;
        else if (strstr((char*)buf, "\"backward\"")) mcmd.cmd = MOTION_CMD_BACKWARD;
        else if (strstr((char*)buf, "\"left\""))     mcmd.cmd = MOTION_CMD_PIVOT_LEFT;
        else if (strstr((char*)buf, "\"right\""))    mcmd.cmd = MOTION_CMD_PIVOT_RIGHT;
        else                                          mcmd.cmd = MOTION_CMD_STOP;

        char* sp = strstr((char*)buf, "\"speed\":");
        if (sp) mcmd.speed = atof(sp + 8);
        motion_send_cmd(&mcmd);
        bus_publish(TOPIC_MOTOR_CMD, &mcmd, sizeof(mcmd));

        // Enable AI manual mode
        uint8_t manual_on = 1;
        bus_publish(TOPIC_MOTOR_CMD, &manual_on, 1);
    } else if (strstr((char*)buf, "\"estop\"")) {
        hk_estop("WebSocket ESTOP");
    } else if (strstr((char*)buf, "\"ping\"")) {
        httpd_ws_frame_t pong = {.type=HTTPD_WS_TYPE_TEXT, .payload=(uint8_t*)"{\"pong\":1}", .len=10};
        httpd_ws_send_frame(req, &pong);
    }

    return ESP_OK;
}

// ─── WS Broadcast Task ────────────────────────────────────
static QueueHandle_t s_telem_sub;

static void ws_broadcast_task(void* arg) {
    hk_message_t msg;
    while (1) {
        if (xQueueReceive(s_telem_sub, &msg, pdMS_TO_TICKS(100)) == pdTRUE) {
            if (s_ws_count == 0 || !s_server) continue;

            httpd_ws_frame_t frame = {
                .type    = HTTPD_WS_TYPE_TEXT,
                .payload = msg.payload,
                .len     = msg.payload_len,
            };

            for (int i = 0; i < 8; i++) {
                if (s_ws_fds[i] == -1) continue;
                esp_err_t r = httpd_ws_send_frame_async(s_server, s_ws_fds[i], &frame);
                if (r != ESP_OK) {
                    // Client disconnected
                    s_ws_fds[i] = -1;
                    s_ws_count = (s_ws_count > 0) ? s_ws_count - 1 : 0;
                }
            }
        }
    }
}

// ─── WiFi Setup ───────────────────────────────────────────
static void wifi_event_handler(void* arg, esp_event_base_t base,
                                int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        esp_wifi_connect();
        ESP_LOGW(TAG, "WiFi disconnected, reconnecting...");
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* ev = (ip_event_got_ip_t*)data;
        ESP_LOGI(TAG, "WiFi connected! IP: " IPSTR, IP2STR(&ev->ip_info.ip));
        char ip_str[20];
        snprintf(ip_str, sizeof(ip_str), IPSTR, IP2STR(&ev->ip_info.ip));
        bus_publish(TOPIC_SYSTEM_ALERT, ip_str, strlen(ip_str) + 1);
    }
}

static hk_status_t wifi_init_sta(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, wifi_event_handler, NULL);
    esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, wifi_event_handler, NULL);

    wifi_config_t wcfg = {
        .sta = {
            .ssid     = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
        }
    };
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wcfg);
    esp_wifi_start();
    esp_wifi_connect();
    return HK_OK;
}

// ─── HTTP Server Start ────────────────────────────────────
static httpd_handle_t start_webserver(void) {
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.max_open_sockets = 12;
    cfg.task_priority    = PRIORITY_WEBSERVER;
    cfg.stack_size       = STACK_WEBSERVER;
    cfg.server_port      = 80;

    httpd_handle_t server;
    if (httpd_start(&server, &cfg) != ESP_OK) return NULL;

    // REST routes
    httpd_uri_t routes[] = {
        { .uri="/api/status",  .method=HTTP_GET,  .handler=handle_status },
        { .uri="/api/move",    .method=HTTP_POST, .handler=handle_move   },
        { .uri="/api/auth",    .method=HTTP_POST, .handler=handle_auth   },
        { .uri="/api/modules", .method=HTTP_GET,  .handler=handle_modules},
        { .uri="/api/estop",   .method=HTTP_POST, .handler=handle_estop  },
        { .uri="/api/ota",     .method=HTTP_POST, .handler=handle_ota_trigger },
        { .uri="/ws",          .method=HTTP_GET,  .handler=handle_ws, .is_websocket=true },
    };

    for (int i = 0; i < sizeof(routes)/sizeof(routes[0]); i++) {
        httpd_register_uri_handler(server, &routes[i]);
    }

    ESP_LOGI(TAG, "Web server started on port 80");
    return server;
}

// ─── Service Lifecycle ────────────────────────────────────
static void comm_task(void* arg) {
    nvs_flash_init();
    wifi_init_sta();

    vTaskDelay(pdMS_TO_TICKS(3000)); // Wait for IP

    s_server = start_webserver();

    // WS broadcast task
    bus_subscribe(TOPIC_TELEMETRY, "ws_telem", &s_telem_sub);
    xTaskCreatePinnedToCore(ws_broadcast_task, "ws_broadcast",
                             4096, NULL, PRIORITY_COMM, CORE_SERVICES, NULL);

    // Keep task alive
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10000));
        ESP_LOGD(TAG, "Comm alive. WS clients: %d", s_ws_count);
    }
}

hk_status_t comm_service_init(void) {
    memset(s_ws_fds, -1, sizeof(s_ws_fds));
    return HK_OK;
}

hk_status_t comm_service_start(void) {
    hk_task_descriptor_t* td;
    return hk_task_create("svc_comm", comm_task, STACK_COMM,
                           NULL, PRIORITY_COMM, CORE_SERVICES, &td);
}
