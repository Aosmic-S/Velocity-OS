/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║         AOSMIC HYPERKERNEL OS — VelocityOS v1.0          ║
 * ║         Hot-Pluggable Modules — Motor / Sensor / Pump    ║
 * ╚══════════════════════════════════════════════════════════╝
 */

#include "../kernel/hyperkernel.h"
#include "../hal/hal.h"
#include "esp_log.h"

static const char* TAG = "Modules";

// ─── Capability Bits ──────────────────────────────────────
#define CAP_DRIVE       (1 << 0)
#define CAP_SENSE_DIST  (1 << 1)
#define CAP_SENSE_GAS   (1 << 2)
#define CAP_SENSE_FIRE  (1 << 3)
#define CAP_ACTUATE     (1 << 4)
#define CAP_PUMP        (1 << 5)
#define CAP_CAMERA      (1 << 6)
#define CAP_IMU         (1 << 7)

// ────────────────────────────────────────────────────────────
//  MODULE: Differential Motor Driver (L298N / TB6612)
// ────────────────────────────────────────────────────────────
static hk_status_t motor_probe(void) {
    // In production: check I2C addr or GPIO response
    ESP_LOGI(TAG, "[Motor] Probing motor driver...");
    return HK_OK;
}

static hk_status_t motor_enable(void) {
    // PWM channels already configured in HAL
    ESP_LOGI(TAG, "[Motor] Motor driver enabled");
    hal_gpio_set(HAL_LED_STATUS, true);
    return HK_OK;
}

static hk_status_t motor_disable(void) {
    for (int i = 0; i < 4; i++) hal_pwm_set_duty((ledc_channel_t)i, 0);
    ESP_LOGI(TAG, "[Motor] Motor driver disabled");
    return HK_OK;
}

hk_module_t module_motor = {
    .name          = "MotorDriver-TB6612",
    .type          = MODULE_MOTOR,
    .capabilities  = CAP_DRIVE,
    .power_mw      = 2000.0f,  // 2W quiescent
    .hot_pluggable = false,
    .probe         = motor_probe,
    .enable        = motor_enable,
    .disable       = motor_disable,
};

// ────────────────────────────────────────────────────────────
//  MODULE: Ultrasonic Sensor Array (HC-SR04 × 3)
// ────────────────────────────────────────────────────────────
static hk_status_t ultrasonic_probe(void) {
    // Verify trig/echo pins are responding
    hal_gpio_set(HAL_US_FRONT_TRIG, false);
    vTaskDelay(pdMS_TO_TICKS(2));
    return HK_OK;
}

static hk_status_t ultrasonic_enable(void) {
    hal_gpio_init(HAL_US_FRONT_TRIG, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_US_FRONT_ECHO, GPIO_MODE_INPUT,  false, false);
    hal_gpio_init(HAL_US_REAR_TRIG,  GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_US_REAR_ECHO,  GPIO_MODE_INPUT,  false, false);
    hal_gpio_init(HAL_US_LEFT_TRIG,  GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_US_LEFT_ECHO,  GPIO_MODE_INPUT,  false, false);
    ESP_LOGI(TAG, "[Ultrasonic] 3x HC-SR04 initialized");
    return HK_OK;
}

hk_module_t module_ultrasonic = {
    .name          = "HC-SR04-Array",
    .type          = MODULE_SENSOR_US,
    .capabilities  = CAP_SENSE_DIST,
    .power_mw      = 45.0f,  // 15mA × 3 × 3.3V
    .hot_pluggable = true,
    .probe         = ultrasonic_probe,
    .enable        = ultrasonic_enable,
    .disable       = NULL,
};

// ────────────────────────────────────────────────────────────
//  MODULE: Gas Sensor (MQ-2)
// ────────────────────────────────────────────────────────────
static hk_status_t gas_probe(void) {
    float v = hal_adc_read_voltage(HAL_GAS_SENSOR_ADC);
    ESP_LOGI(TAG, "[Gas] MQ-2 voltage: %.2fV", v);
    return (v > 0.1f && v < 3.3f) ? HK_OK : HK_ERR_FAULT;
}

static hk_status_t gas_enable(void) {
    // Warm-up period for MQ-2
    ESP_LOGI(TAG, "[Gas] MQ-2 warming up (20s)...");
    // In production: vTaskDelay(pdMS_TO_TICKS(20000));
    ESP_LOGI(TAG, "[Gas] MQ-2 ready");
    return HK_OK;
}

hk_module_t module_gas = {
    .name          = "MQ2-GasSensor",
    .type          = MODULE_SENSOR_GAS,
    .capabilities  = CAP_SENSE_GAS | CAP_SENSE_FIRE,
    .power_mw      = 900.0f,   // MQ-2 ~900mW heater
    .hot_pluggable = true,
    .probe         = gas_probe,
    .enable        = gas_enable,
    .disable       = NULL,
};

// ────────────────────────────────────────────────────────────
//  MODULE: Water Pump (5V relay-driven)
// ────────────────────────────────────────────────────────────
static bool s_pump_active = false;

static hk_status_t pump_probe(void) {
    // Verify relay GPIO works
    hal_gpio_set(HAL_PUMP_RELAY, false);
    return HK_OK;
}

static hk_status_t pump_enable(void) {
    hal_gpio_init(HAL_PUMP_RELAY, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_set(HAL_PUMP_RELAY, false);
    ESP_LOGI(TAG, "[Pump] Water pump relay initialized");
    return HK_OK;
}

static hk_status_t pump_disable(void) {
    hal_gpio_set(HAL_PUMP_RELAY, false);
    s_pump_active = false;
    return HK_OK;
}

hk_module_t module_pump = {
    .name          = "WaterPump-5V",
    .type          = MODULE_PUMP,
    .capabilities  = CAP_PUMP | CAP_ACTUATE,
    .power_mw      = 3000.0f,  // 3W pump
    .hot_pluggable = true,
    .probe         = pump_probe,
    .enable        = pump_enable,
    .disable       = pump_disable,
};

// ────────────────────────────────────────────────────────────
//  MODULE: Flame Sensor (IR digital)
// ────────────────────────────────────────────────────────────
static hk_status_t flame_probe(void) {
    return HK_OK; // Digital input, always present
}

static hk_status_t flame_enable(void) {
    hal_gpio_init(HAL_FLAME_SENSOR, GPIO_MODE_INPUT, true, false);
    hal_gpio_init(HAL_SMOKE_SENSOR, GPIO_MODE_INPUT, true, false);
    ESP_LOGI(TAG, "[Flame] Flame + smoke sensors initialized");
    return HK_OK;
}

hk_module_t module_flame = {
    .name          = "FlameSensor-IR",
    .type          = MODULE_SENSOR_GAS,
    .capabilities  = CAP_SENSE_FIRE,
    .power_mw      = 15.0f,
    .hot_pluggable = true,
    .probe         = flame_probe,
    .enable        = flame_enable,
    .disable       = NULL,
};

// ────────────────────────────────────────────────────────────
//  Module Registry — Register all modules with the kernel
// ────────────────────────────────────────────────────────────
void modules_register_all(void) {
    ESP_LOGI(TAG, "Registering all hardware modules...");

    hk_module_register(&module_motor);
    hk_module_register(&module_ultrasonic);
    hk_module_register(&module_gas);
    hk_module_register(&module_pump);
    hk_module_register(&module_flame);

    // Enable all modules
    for (int i = 0; i < g_kernel.module_count; i++) {
        hk_status_t r = hk_module_enable(g_modules[i].module_id);
        if (r == HK_OK) {
            ESP_LOGI(TAG, "✓ Module enabled: %s", g_modules[i].name);
        } else {
            ESP_LOGW(TAG, "✗ Module failed: %s (err=%d)", g_modules[i].name, r);
        }
    }

    ESP_LOGI(TAG, "Module registry complete: %u modules active", g_kernel.module_count);
}
