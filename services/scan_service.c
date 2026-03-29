/**
 * Sensor Service — VelocityOS
 * Aggregates all sensors and publishes to message bus
 */

#include "../kernel/hyperkernel.h"
#include "../hal/hal.h"
#include "../bus/message_bus.h"
#include "esp_log.h"

static const char* TAG = "SensorSvc";

typedef struct {
    float front_cm, rear_cm, left_cm;
    float gas, battery, temp;
    bool  flame, smoke;
} sensor_payload_t;

static hal_ultrasonic_t us_front = { .trig = HAL_US_FRONT_TRIG, .echo = HAL_US_FRONT_ECHO };
static hal_ultrasonic_t us_rear  = { .trig = HAL_US_REAR_TRIG,  .echo = HAL_US_REAR_ECHO  };
static hal_ultrasonic_t us_left  = { .trig = HAL_US_LEFT_TRIG,  .echo = HAL_US_LEFT_ECHO  };

static void sensor_task(void* arg) {
    TickType_t last_wake = xTaskGetTickCount();
    ESP_LOGI(TAG, "Sensor service running at 10Hz");

    while (1) {
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(100)); // 10Hz

        sensor_payload_t p = {0};

        // Ultrasonic readings (each takes ~10ms, stagger)
        p.front_cm = hal_ultrasonic_read_cm(&us_front);
        vTaskDelay(pdMS_TO_TICKS(5));
        p.rear_cm  = hal_ultrasonic_read_cm(&us_rear);
        vTaskDelay(pdMS_TO_TICKS(5));
        p.left_cm  = hal_ultrasonic_read_cm(&us_left);

        // ADC readings
        float gas_v  = hal_adc_read_voltage(HAL_GAS_SENSOR_ADC);
        p.gas        = gas_v / 3.3f;  // Normalize 0-1
        p.battery    = hal_battery_voltage();
        p.temp       = 25.0f + (hal_adc_read_raw(ADC_CHANNEL_2) / 4095.0f) * 50.0f;

        // Digital sensors
        p.flame = !hal_gpio_get(HAL_FLAME_SENSOR); // Active low
        p.smoke = !hal_gpio_get(HAL_SMOKE_SENSOR);

        // Publish sensor data
        bus_publish(TOPIC_SENSOR_DATA, &p, sizeof(p));

        // Obstacle alert
        if (p.front_cm > 0 && p.front_cm < 40.0f) {
            bus_publish(TOPIC_OBSTACLE, &p.front_cm, sizeof(float));
        }

        // High-level alerts
        if (p.gas > 0.7f || p.flame || p.smoke) {
            char alert[64];
            snprintf(alert, sizeof(alert),
                     "ALERT: gas=%.0f%% flame=%d smoke=%d",
                     p.gas * 100, p.flame, p.smoke);
            bus_publish(TOPIC_SENSOR_ALERT, alert, strlen(alert) + 1);
        }
    }
}

hk_status_t sensor_service_init(void) {
    hal_ultrasonic_init(&us_front);
    hal_ultrasonic_init(&us_rear);
    hal_ultrasonic_init(&us_left);
    return HK_OK;
}

hk_status_t sensor_service_start(void) {
    hk_task_descriptor_t* td;
    return hk_task_create("svc_sensor", sensor_task,
                           STACK_SENSOR, NULL, PRIORITY_SENSOR,
                           CORE_SERVICES, &td);
}
