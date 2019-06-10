#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_system.h"

#include "esp_bme680.h"
#include "errno.h"

static const char *TAG = "esp_bme680";

#define _I2C_PORT_NUMBER(num) I2C_NUM_##num
#define I2C_PORT_NUMBER(num) _I2C_PORT_NUMBER(num)

#define BME680_I2C_SCL_IO CONFIG_BME680_I2C_SCL_IO                              /*!< gpio number for I2C master clock */
#define BME680_I2C_SDA_IO CONFIG_BME680_I2C_SDA_IO                              /*!< gpio number for I2C master data  */
#define BME680_I2C_PORT_NUM I2C_PORT_NUMBER(CONFIG_BME680_I2C_PORT_NUM)     /*!< I2C port number for master dev */


#define I2C_MASTER_FREQ_HZ 100000                                           /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                                         /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                                         /*!< I2C master doesn't need buffer */

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
#define ACK_VAL 0x0                             /*!< I2C ack value */
#define NACK_VAL 0x1                            /*!< I2C nack value */

#define WRITE_BIT I2C_MASTER_WRITE
#define READ_BIT I2C_MASTER_READ

void esp_bme680_delay_ms(uint32_t delay);

esp_err_t esp_bme680_i2c_init(i2c_port_t i2cPort, gpio_num_t i2cSclIo, gpio_num_t i2cSdaIo);

int8_t esp_bme680_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);

int8_t esp_bme680_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len);


/**************************************************************************/
/*!
 *  @brief  Delay user function for the Bosch BME680 library.
 *  @param  delay in ms
 */
/**************************************************************************/
void esp_bme680_delay_ms(uint32_t delay) {
    vTaskDelay(delay / portTICK_PERIOD_MS);
}

/**************************************************************************/
/*!
 * @brief  Initialize the I2C master interface.
 */
/**************************************************************************/
esp_err_t esp_bme680_i2c_init(i2c_port_t i2cPort, gpio_num_t i2cSclIo, gpio_num_t i2cSdaIo) {
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.scl_io_num = i2cSclIo;
    conf.sda_io_num = i2cSdaIo;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
    i2c_param_config(i2cPort, &conf);
    return i2c_driver_install(i2cPort,
                              conf.mode,
                              I2C_MASTER_RX_BUF_DISABLE,
                              I2C_MASTER_TX_BUF_DISABLE,
                              0);
}


esp_err_t esp_bme680_init(int bme680DeviceAddress, struct bme680_dev *gas_sensor) {
    ESP_LOGI(TAG, "Port[%d], SCL: %d, SDA: %d",
             CONFIG_BME680_I2C_PORT_NUM,
             BME680_I2C_SCL_IO,
             BME680_I2C_SDA_IO);

    // *** Initialize
    esp_err_t res = esp_bme680_i2c_init(BME680_I2C_PORT_NUM, BME680_I2C_SCL_IO, BME680_I2C_SDA_IO);
    if (res != ESP_OK) {
        ESP_LOGE(TAG, "I2C initialization failed");
        return ESP_FAIL;
    }

    gas_sensor->dev_id = bme680DeviceAddress;
    gas_sensor->intf = BME680_I2C_INTF;
    gas_sensor->read = esp_bme680_i2c_read;
    gas_sensor->write = esp_bme680_i2c_write;
    gas_sensor->delay_ms = esp_bme680_delay_ms;

    /* amb_temp can be set to 25 prior to configuring the gas sensor
     * or by performing a few temperature readings without operating the gas sensor.
     */
    gas_sensor->amb_temp = 25;

    int8_t rslt;
    rslt = bme680_init(gas_sensor);
    if (rslt != BME680_OK) {
        ESP_LOGE(TAG, "BME680 initialization failed");
        return ESP_FAIL;
    }

    // *** Configure
    uint8_t set_required_settings;

    /* Set the temperature, pressure and humidity settings */
    gas_sensor->tph_sett.os_hum = BME680_OS_2X;
    gas_sensor->tph_sett.os_pres = BME680_OS_4X;
    gas_sensor->tph_sett.os_temp = BME680_OS_8X;
    gas_sensor->tph_sett.filter = BME680_FILTER_SIZE_3;

    /* Set the remaining gas sensor settings and link the heating profile */
    gas_sensor->gas_sett.run_gas = BME680_ENABLE_GAS_MEAS;
    /* Create a ramp heat waveform in 3 steps */
    gas_sensor->gas_sett.heatr_temp = 320; /* degree Celsius */
    gas_sensor->gas_sett.heatr_dur = 150; /* milliseconds */

    /* Select the power mode */
    /* Must be set before writing the sensor configuration */
    gas_sensor->power_mode = BME680_FORCED_MODE;

    /* Set the required sensor settings needed */
    set_required_settings = BME680_OST_SEL | BME680_OSP_SEL | BME680_OSH_SEL | BME680_FILTER_SEL
                            | BME680_GAS_SENSOR_SEL;

    /* Set the desired sensor configuration */
    rslt = bme680_set_sensor_settings(set_required_settings, gas_sensor);
    if (rslt != BME680_OK) {
        ESP_LOGE(TAG, "BME680 configuration failed");
        return ESP_FAIL;
    }

    /* Set the power mode */
    rslt = bme680_set_sensor_mode(gas_sensor);
    if (rslt != BME680_OK) {
        ESP_LOGE(TAG, "BME680 sensor mode failed");
        return ESP_FAIL;
    }

    return ESP_OK;
}

int8_t esp_bme680_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = BME680_OK; /* Return 0 for Success, non-zero for failure */

    if (len > 0) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev_id << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev_id << 1) | READ_BIT, ACK_CHECK_EN);
        i2c_master_read(cmd, reg_data, len, I2C_MASTER_LAST_NACK);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(BME680_I2C_PORT_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            rslt = BME680_OK;
        } else {
            rslt = BME680_E_COM_FAIL;
        }
    }
    return rslt;
}

int8_t esp_bme680_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *reg_data, uint16_t len) {
    int8_t rslt = BME680_OK; /* Return 0 for Success, non-zero for failure */

    if (len > 0) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();

        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (dev_id << 1) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
        i2c_master_write(cmd, reg_data, len, ACK_CHECK_EN);
        i2c_master_stop(cmd);

        esp_err_t ret = i2c_master_cmd_begin(BME680_I2C_PORT_NUM, cmd, 1000 / portTICK_RATE_MS);
        i2c_cmd_link_delete(cmd);

        if (ret == ESP_OK) {
            rslt = BME680_OK;
        } else {
            rslt = BME680_E_COM_FAIL;
        }
    }
    return rslt;
}
