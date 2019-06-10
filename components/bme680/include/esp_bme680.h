#ifndef _ESP_BME680_H
#define _ESP_BME680_H

#include "bme680.h"

#include "sdkconfig.h"
#include "esp_err.h"


#ifdef __cplusplus
extern "C" {
#endif

// http://p-nand-q.com/programming/cplusplus/using_member_functions_with_c_function_pointers.html
esp_err_t esp_bme680_init(int bme680DeviceAddress, struct bme680_dev *gas_sensor);

#ifdef __cplusplus
}
#endif

#endif
