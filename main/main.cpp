
#define ESP_PLATFORM 1

#include <Arduino.h>
#include <SPI.h>

#include <memory>


extern "C"
{
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_task_wdt.h"
#include "nvs_flash.h"
#include "esp_bme680.h"
#include "meter.h"
}

#define MAINLOOPCORE 1
#define MEASURING_INTERVAL 2000
#define DISPLAY_REFRESH_INTERVAL 35

static const char *TAG = "main";

TaskHandle_t runMeasuringLoopHandle = NULL;
TaskHandle_t runDisplayLoopHandle = NULL;

bool loopTaskWDTEnabled = false; // Enable if watchdog running
struct bme680_dev gas_sensor;

extern "C"
{
void setupTasks();

void runMeasuringLoop(void *pvParameters);
void runDisplayLoop(void *pvParameters);

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initArduino();
    meter_setup();
    esp_bme680_init(BME680_I2C_ADDR_SECONDARY, &gas_sensor);

    setupTasks();
}

void setupTasks() {
    ESP_LOGD(TAG, "Setup measuring task");
    xTaskCreateUniversal(runMeasuringLoop, "measuringTask", 8192, NULL, 2, &runMeasuringLoopHandle, MAINLOOPCORE);
    ESP_LOGD(TAG, "Setup display task");
    xTaskCreateUniversal(runDisplayLoop, "displayTask", 8192, NULL, 1, &runDisplayLoopHandle, MAINLOOPCORE);
}

void runMeasuringLoop(void *pvParameters) {
    /* Get the total measurement duration so as to sleep or wait till the
 * measurement is complete */
    uint16_t meas_period;
    bme680_get_profile_dur(&meas_period, &gas_sensor);
    ESP_LOGD(TAG, "Measure period is %d ms", meas_period);

    // *** Read
    struct bme680_field_data data;

    uint8_t rslt;
    for (;;) {
        if (loopTaskWDTEnabled) {
            esp_task_wdt_reset();
        }
        ESP_LOGD(TAG, "Executing main loop");


        rslt = bme680_get_sensor_data(&data, &gas_sensor);
        if (rslt != BME680_OK) {
            ESP_LOGI(TAG, "Invalid measurement");
            continue;
        }

        /* Avoid using measurements from an unstable heating setup */
        if (data.status & BME680_GASM_VALID_MSK) {
            printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH, G: %d ohms\n",
                   data.temperature / 100.0f,
                   data.pressure / 100.0f, data.humidity / 1000.0f, data.gas_resistance);
        } else {
            printf("T: %.2f degC, P: %.2f hPa, H %.2f %%rH\n", data.temperature / 100.0f,
                   data.pressure / 100.0f, data.humidity / 1000.0f);
        }

        /* Trigger the next measurement if you would like to read data out continuously */
        if (gas_sensor.power_mode == BME680_FORCED_MODE) {
            rslt = bme680_set_sensor_mode(&gas_sensor);
            if (rslt != BME680_OK) {
                ESP_LOGE(TAG, "Unable to set BME680 power mode");
            }
        }
        delay(MEASURING_INTERVAL);
    }
}

void runDisplayLoop(void *pvParameters) {
    for (;;) {
        if (loopTaskWDTEnabled) {
            esp_task_wdt_reset();
        }

        meter_loop();
        delay(DISPLAY_REFRESH_INTERVAL);
    }
}
}
