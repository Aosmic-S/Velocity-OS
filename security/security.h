/**
 * Security Layer — VelocityOS
 * Token-based authentication, command validation, fail-safe lock
 */

#pragma once
#include "../kernel/hyperkernel.h"
#include <string.h>
#include <stdbool.h>

#define SEC_TOKEN_LEN        32
#define SEC_MAX_SESSIONS     8
#define SEC_TOKEN_EXPIRE_MS  3600000   // 1 hour
#define SEC_MAX_FAILED_AUTH  5
#define SEC_LOCKOUT_MS       60000     // 1 min lockout

// ─── Permission Bits ──────────────────────────────────────
#define SEC_PERM_READ        (1 << 0)
#define SEC_PERM_MOTION      (1 << 1)
#define SEC_PERM_CONFIG      (1 << 2)
#define SEC_PERM_OTA         (1 << 3)
#define SEC_PERM_ADMIN       (0xFF)

typedef struct {
    char    token[SEC_TOKEN_LEN + 1];
    uint8_t permissions;
    int64_t created_at;
    int64_t last_used;
    bool    valid;
    char    client_ip[20];
} sec_session_t;

typedef struct {
    sec_session_t sessions[SEC_MAX_SESSIONS];
    uint8_t       session_count;
    uint32_t      failed_auths;
    int64_t       lockout_until;
    bool          locked;
    SemaphoreHandle_t lock;
    // Default admin token (change in production!)
    char          admin_token[SEC_TOKEN_LEN + 1];
} security_state_t;

extern security_state_t g_security;

hk_status_t security_init(void);
bool        security_authenticate(const char* token, uint8_t required_perm);
hk_status_t security_create_session(const char* password, char* out_token, const char* client_ip);
hk_status_t security_revoke_session(const char* token);
bool        security_validate_cmd(const char* token, const char* cmd);
void        security_lockout(void);
bool        security_is_locked(void);
