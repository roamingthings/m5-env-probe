
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
}

#define MAINLOOPCORE 1
TaskHandle_t runLoopHandle = NULL;
bool loopTaskWDTEnabled = false; // Enable if watchdog running

extern "C"
{
void runLoop(void *pvParameters);
void setupApp();

void app_main() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    initArduino();
    setupApp();
}

void setupApp() {
    printf("Initializing App\n");
    xTaskCreateUniversal(runLoop, "loopTask", 8192, NULL, 1, &runLoopHandle, MAINLOOPCORE);
}

void runLoop(void *pvParameters) {
    for (;;) {
        if (loopTaskWDTEnabled) {
            esp_task_wdt_reset();
        }
        printf("Executing main loop\n");
        delay(3000);
    }
}
}
