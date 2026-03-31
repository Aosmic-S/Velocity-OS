/**
 * ╔══════════════════════════════════════════════════════════╗
 * ║         AOSMIC HYPERKERNEL OS — VelocityOS v1.0          ║
 * ║         Hardware Abstraction Layer (HAL)                 ║
 * ╚══════════════════════════════════════════════════════════╝
 * 
 * ALL hardware access goes through this layer.
 * Drivers must NEVER call ESP-IDF hardware APIs directly.
 */

#pragma once
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "driver/i2c.h"
#include "driver/spi_master.h"
#include "driver/adc.h"
#include "driver/uart.h"
#include "esp_adc/adc_oneshot.h"
#include "../kernel/hyperkernel.h"

// ─── Pin Definitions — ESP32-S3 4WD Robot ────────────────
// Motor Driver (L298N / TB6612)
#define HAL_MOTOR_FL_PWM    GPIO_NUM_1
#define HAL_MOTOR_FL_DIR_A  GPIO_NUM_2
#define HAL_MOTOR_FL_DIR_B  GPIO_NUM_3
#define HAL_MOTOR_FR_PWM    GPIO_NUM_4
#define HAL_MOTOR_FR_DIR_A  GPIO_NUM_5
#define HAL_MOTOR_FR_DIR_B  GPIO_NUM_6
#define HAL_MOTOR_RL_PWM    GPIO_NUM_7
#define HAL_MOTOR_RL_DIR_A  GPIO_NUM_8
#define HAL_MOTOR_RL_DIR_B  GPIO_NUM_9
#define HAL_MOTOR_RR_PWM    GPIO_NUM_10
#define HAL_MOTOR_RR_DIR_A  GPIO_NUM_11
#define HAL_MOTOR_RR_DIR_B  GPIO_NUM_12

// Ultrasonic Sensors
#define HAL_US_FRONT_TRIG   GPIO_NUM_13
#define HAL_US_FRONT_ECHO   GPIO_NUM_14
#define HAL_US_REAR_TRIG    GPIO_NUM_15
#define HAL_US_REAR_ECHO    GPIO_NUM_16
#define HAL_US_LEFT_TRIG    GPIO_NUM_17
#define HAL_US_LEFT_ECHO    GPIO_NUM_18

// Gas / Fire Sensors
#define HAL_GAS_SENSOR_ADC  ADC_CHANNEL_0   // GPIO1
#define HAL_FLAME_SENSOR    GPIO_NUM_19
#define HAL_SMOKE_SENSOR    GPIO_NUM_20

// Pump / Actuators
#define HAL_PUMP_RELAY      GPIO_NUM_21
#define HAL_BUZZER          GPIO_NUM_22
#define HAL_LED_STATUS      GPIO_NUM_23
#define HAL_LED_WARN        GPIO_NUM_24

// I2C (IMU, OLED)
#define HAL_I2C_SDA         GPIO_NUM_38
#define HAL_I2C_SCL         GPIO_NUM_39
#define HAL_I2C_PORT        I2C_NUM_0
#define HAL_I2C_FREQ_HZ     400000

// SPI (Camera, SD)
#define HAL_SPI_MOSI        GPIO_NUM_35
#define HAL_SPI_MISO        GPIO_NUM_36
#define HAL_SPI_CLK         GPIO_NUM_37
#define HAL_SPI_CS_CAM      GPIO_NUM_33
#define HAL_SPI_CS_SD       GPIO_NUM_34

// Battery Monitoring
#define HAL_BATT_ADC        ADC_CHANNEL_4
#define HAL_BATT_DIVIDER    2.0f   // Voltage divider ratio

// ─── PWM Config ───────────────────────────────────────────
#define HAL_PWM_FREQ_HZ     20000
#define HAL_PWM_RESOLUTION  LEDC_TIMER_10_BIT  // 0-1023
#define HAL_PWM_MAX_DUTY    1023
#define HAL_PWM_CHANNELS    8

// ─── HAL GPIO ─────────────────────────────────────────────
typedef struct {
    gpio_num_t  pin;
    gpio_mode_t mode;
    bool        pull_up;
    bool        pull_down;
    bool        initialized;
} hal_gpio_t;

hk_status_t hal_gpio_init(gpio_num_t pin, gpio_mode_t mode, bool pull_up, bool pull_down);
hk_status_t hal_gpio_set(gpio_num_t pin, bool level);
bool        hal_gpio_get(gpio_num_t pin);
hk_status_t hal_gpio_set_interrupt(gpio_num_t pin, gpio_int_type_t type, gpio_isr_t handler, void* arg);

// ─── HAL PWM ──────────────────────────────────────────────
typedef struct {
    ledc_channel_t  channel;
    gpio_num_t      pin;
    uint32_t        freq_hz;
    uint32_t        duty;       // 0-1023
    bool            initialized;
} hal_pwm_t;

hk_status_t hal_pwm_init(ledc_channel_t ch, gpio_num_t pin, uint32_t freq_hz);
hk_status_t hal_pwm_set_duty(ledc_channel_t ch, uint32_t duty);
hk_status_t hal_pwm_set_duty_pct(ledc_channel_t ch, float pct);
hk_status_t hal_pwm_stop(ledc_channel_t ch);

// ─── HAL I2C ──────────────────────────────────────────────
hk_status_t hal_i2c_init(void);
hk_status_t hal_i2c_write(uint8_t addr, uint8_t reg, const uint8_t* data, size_t len);
hk_status_t hal_i2c_read(uint8_t addr, uint8_t reg, uint8_t* data, size_t len);
bool        hal_i2c_probe(uint8_t addr);

// ─── HAL ADC ──────────────────────────────────────────────
hk_status_t hal_adc_init(void);
int         hal_adc_read_raw(adc_channel_t ch);
float       hal_adc_read_voltage(adc_channel_t ch);
float       hal_battery_voltage(void);

// ─── HAL Ultrasonic ───────────────────────────────────────
typedef struct {
    gpio_num_t trig;
    gpio_num_t echo;
    float      last_cm;
} hal_ultrasonic_t;

hk_status_t hal_ultrasonic_init(hal_ultrasonic_t* us);
float       hal_ultrasonic_read_cm(hal_ultrasonic_t* us);

// ─── HAL System Init ──────────────────────────────────────
hk_status_t hal_init(void);
