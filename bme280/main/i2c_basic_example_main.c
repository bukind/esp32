/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// BME280 Reader via I2C.

#include "wificonn.h"
#include "bme280.h"
#include "driver/i2c_master.h"
#include "nvs_flash.h"

static const char *TAG = "weather";

#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */

// Most of the BME280 code is reverse-engineered from the https://github.com/periph/devices/tree/v3.7.4/bmxx80.

#define BME280_SENSOR_ADDR         0x76        /*!< Address of the BME280 sensor */

// @brief i2c master initialization
static void i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_MASTER_NUM,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = BME280_SENSOR_ADDR,
        .scl_speed_hz = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
}

static void i2c_master_deinit(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t dev_handle) {
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

void app_main(void)
{
    // Initialize NVS.
    // It seems to be a requirement for WiFi, though they don't tell it anywhere.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    wificonn_init_sta();

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    bme280_sensor_t sensor;
    esp_err_t err = bme280_init(dev_handle, &sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bme280: %d", err);
        i2c_master_deinit(bus_handle, dev_handle);
        return;
    }

    for (int i = 0; i < 10; i++) {
        ESP_ERROR_CHECK(bme280_measure_once(sensor));
        usleep(1000000);
    }

    /* Demonstrate writing by halting the BME280 */
    ESP_ERROR_CHECK(bme280_halt(sensor));
    i2c_master_deinit(bus_handle, dev_handle);

    wificonn_deinit();
}
