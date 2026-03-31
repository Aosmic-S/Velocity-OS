/**
 * Hardware Abstraction Layer Implementation
 * VelocityOS / Aosmic HyperKernel OS
 */

#include "hal.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"

static const char* TAG = "HAL";
static adc_oneshot_unit_handle_t s_adc_handle = NULL;
static bool s_pwm_timer_init = false;

// ─── GPIO ─────────────────────────────────────────────────
hk_status_t hal_gpio_init(gpio_num_t pin, gpio_mode_t mode, bool pull_up, bool pull_down) {
    gpio_config_t cfg = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = mode,
        .pull_up_en   = pull_up   ? GPIO_PULLUP_ENABLE   : GPIO_PULLUP_DISABLE,
        .pull_down_en = pull_down ? GPIO_PULLDOWN_ENABLE : GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    return gpio_config(&cfg) == ESP_OK ? HK_OK : HK_ERR_FAULT;
}

hk_status_t hal_gpio_set(gpio_num_t pin, bool level) {
    return gpio_set_level(pin, level ? 1 : 0) == ESP_OK ? HK_OK : HK_ERR_FAULT;
}

bool hal_gpio_get(gpio_num_t pin) {
    return gpio_get_level(pin) != 0;
}

hk_status_t hal_gpio_set_interrupt(gpio_num_t pin, gpio_int_type_t type,
                                    gpio_isr_t handler, void* arg) {
    gpio_set_intr_type(pin, type);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(pin, handler, arg);
    gpio_intr_enable(pin);
    return HK_OK;
}

// ─── PWM ──────────────────────────────────────────────────
hk_status_t hal_pwm_init(ledc_channel_t ch, gpio_num_t pin, uint32_t freq_hz) {
    if (!s_pwm_timer_init) {
        ledc_timer_config_t timer = {
            .speed_mode      = LEDC_LOW_SPEED_MODE,
            .timer_num       = LEDC_TIMER_0,
            .duty_resolution = HAL_PWM_RESOLUTION,
            .freq_hz         = freq_hz,
            .clk_cfg         = LEDC_AUTO_CLK,
        };
        if (ledc_timer_config(&timer) != ESP_OK) return HK_ERR_FAULT;
        s_pwm_timer_init = true;
    }
    ledc_channel_config_t channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = ch,
        .timer_sel  = LEDC_TIMER_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .gpio_num   = pin,
        .duty       = 0,
        .hpoint     = 0,
    };
    return ledc_channel_config(&channel) == ESP_OK ? HK_OK : HK_ERR_FAULT;
}

hk_status_t hal_pwm_set_duty(ledc_channel_t ch, uint32_t duty) {
    if (duty > HAL_PWM_MAX_DUTY) duty = HAL_PWM_MAX_DUTY;
    ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
    return HK_OK;
}

hk_status_t hal_pwm_set_duty_pct(ledc_channel_t ch, float pct) {
    if (pct < 0.0f) pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    uint32_t duty = (uint32_t)((pct / 100.0f) * HAL_PWM_MAX_DUTY);
    return hal_pwm_set_duty(ch, duty);
}

hk_status_t hal_pwm_stop(ledc_channel_t ch) {
    ledc_stop(LEDC_LOW_SPEED_MODE, ch, 0);
    return HK_OK;
}

// ─── I2C ──────────────────────────────────────────────────
hk_status_t hal_i2c_init(void) {
    i2c_config_t cfg = {
        .mode             = I2C_MODE_MASTER,
        .sda_io_num       = HAL_I2C_SDA,
        .scl_io_num       = HAL_I2C_SCL,
        .sda_pullup_en    = GPIO_PULLUP_ENABLE,
        .scl_pullup_en    = GPIO_PULLUP_ENABLE,
        .master.clk_speed = HAL_I2C_FREQ_HZ,
    };
    esp_err_t r = i2c_param_config(HAL_I2C_PORT, &cfg);
    if (r != ESP_OK) return HK_ERR_FAULT;
    r = i2c_driver_install(HAL_I2C_PORT, I2C_MODE_MASTER, 0, 0, 0);
    return r == ESP_OK ? HK_OK : HK_ERR_FAULT;
}

hk_status_t hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    if (data && len > 0) i2c_master_write(cmd, data, len, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(HAL_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r == ESP_OK ? HK_OK : HK_ERR_FAULT;
}

hk_status_t hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_READ, true);
    if (len > 1) i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(HAL_I2C_PORT, cmd, pdMS_TO_TICKS(100));
    i2c_cmd_link_delete(cmd);
    return r == ESP_OK ? HK_OK : HK_ERR_FAULT;
}

bool hal_i2c_probe(uint8_t addr) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_stop(cmd);
    esp_err_t r = i2c_master_cmd_begin(HAL_I2C_PORT, cmd, pdMS_TO_TICKS(50));
    i2c_cmd_link_delete(cmd);
    return r == ESP_OK;
}

// ─── ADC ──────────────────────────────────────────────────
hk_status_t hal_adc_init(void) {
    adc_oneshot_unit_init_cfg_t cfg = { .unit_id = ADC_UNIT_1 };
    esp_err_t r = adc_oneshot_new_unit(&cfg, &s_adc_handle);
    if (r != ESP_OK) return HK_ERR_FAULT;

    adc_oneshot_chan_cfg_t ch_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    adc_oneshot_config_channel(s_adc_handle, HAL_GAS_SENSOR_ADC, &ch_cfg);
    adc_oneshot_config_channel(s_adc_handle, HAL_BATT_ADC, &ch_cfg);
    return HK_OK;
}

int hal_adc_read_raw(adc_channel_t ch) {
    int val = 0;
    if (s_adc_handle) adc_oneshot_read(s_adc_handle, ch, &val);
    return val;
}

float hal_adc_read_voltage(adc_channel_t ch) {
    int raw = hal_adc_read_raw(ch);
    return (raw / 4095.0f) * 3.3f;
}

float hal_battery_voltage(void) {
    float v = hal_adc_read_voltage(HAL_BATT_ADC);
    return v * HAL_BATT_DIVIDER;
}

// ─── Ultrasonic ───────────────────────────────────────────
hk_status_t hal_ultrasonic_init(hal_ultrasonic_t* us) {
    hal_gpio_init(us->trig, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(us->echo, GPIO_MODE_INPUT, false, false);
    hal_gpio_set(us->trig, false);
    return HK_OK;
}

float hal_ultrasonic_read_cm(hal_ultrasonic_t* us) {
    // Trigger 10us pulse
    hal_gpio_set(us->trig, false);
    ets_delay_us(2);
    hal_gpio_set(us->trig, true);
    ets_delay_us(10);
    hal_gpio_set(us->trig, false);

    // Measure echo pulse width
    int64_t start = esp_timer_get_time();
    int64_t timeout = start + 30000; // 30ms max

    while (!hal_gpio_get(us->echo)) {
        if (esp_timer_get_time() > timeout) return -1.0f;
    }
    int64_t rise = esp_timer_get_time();

    while (hal_gpio_get(us->echo)) {
        if (esp_timer_get_time() > timeout) return -1.0f;
    }
    int64_t fall = esp_timer_get_time();

    float duration_us = (float)(fall - rise);
    float distance_cm = (duration_us * 0.0343f) / 2.0f;
    us->last_cm = distance_cm;
    return distance_cm;
}

// ─── HAL Init ─────────────────────────────────────────────
hk_status_t hal_init(void) {
    ESP_LOGI(TAG, "HAL initializing...");

    // Status LEDs
    hal_gpio_init(HAL_LED_STATUS, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_LED_WARN,   GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_BUZZER,     GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_PUMP_RELAY, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_FLAME_SENSOR, GPIO_MODE_INPUT, true, false);
    hal_gpio_init(HAL_SMOKE_SENSOR, GPIO_MODE_INPUT, true, false);

    // ADC
    hal_adc_init();

    // I2C
    hal_i2c_init();

    // PWM channels for motors
    hal_pwm_init(LEDC_CHANNEL_0, HAL_MOTOR_FL_PWM, HAL_PWM_FREQ_HZ);
    hal_pwm_init(LEDC_CHANNEL_1, HAL_MOTOR_FR_PWM, HAL_PWM_FREQ_HZ);
    hal_pwm_init(LEDC_CHANNEL_2, HAL_MOTOR_RL_PWM, HAL_PWM_FREQ_HZ);
    hal_pwm_init(LEDC_CHANNEL_3, HAL_MOTOR_RR_PWM, HAL_PWM_FREQ_HZ);

    // Motor direction GPIOs
    hal_gpio_init(HAL_MOTOR_FL_DIR_A, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_MOTOR_FL_DIR_B, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_MOTOR_FR_DIR_A, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_MOTOR_FR_DIR_B, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_MOTOR_RL_DIR_A, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_MOTOR_RL_DIR_B, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_MOTOR_RR_DIR_A, GPIO_MODE_OUTPUT, false, false);
    hal_gpio_init(HAL_MOTOR_RR_DIR_B, GPIO_MODE_OUTPUT, false, false);

    ESP_LOGI(TAG, "HAL initialized successfully");
    hal_gpio_set(HAL_LED_STATUS, true); // Signal ready
    return HK_OK;
}
