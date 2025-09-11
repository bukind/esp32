/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// BME280 Reader via I2C.

#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/i2c_master.h"

static const char *TAG = "bme280";

#define I2C_MASTER_SCL_IO           9 // was CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           8 // was CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS       1000

// Most of the BME280 code is reverse-engineered from the https://github.com/periph/devices/tree/v3.7.4/bmxx80.

#define BME280_SENSOR_ADDR         0x76        /*!< Address of the BME280 sensor */
#define BME280_REG_ADDR_IDENT      0xD0 // Identity register, should return 0x60 for BME280.
#define BME280_REG_ADDR_CONFIG     0xF5
#define BME280_REG_ADDR_CTRL_MEAS  0xF4

#define BME280_SAMPLING_04X        3
#define BME280_STANDBY_1S          5
#define BME280_FILTER_NONE         0
#define BME280_MODE_SLEEP          0

// @brief Read a sequence of bytes from a BME280 sensor registers.
static esp_err_t bme280_register_read(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(dev_handle, &reg_addr, 1, data, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

/*
  Write a byte to a BME280 sensor register
static esp_err_t bme280_register_write_byte(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t data)
{
    uint8_t write_buf[2] = {reg_addr, data};
    return i2c_master_transmit(dev_handle, write_buf, sizeof(write_buf), I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}
 */

// @brief Write bytes to a BME280.
static esp_err_t bme280_register_write_bytes(i2c_master_dev_handle_t dev_handle, uint8_t *write_buf, size_t len)
{
    return i2c_master_transmit(dev_handle, write_buf, len, I2C_MASTER_TIMEOUT_MS / portTICK_PERIOD_MS);
}

typedef struct {
    i2c_master_dev_handle_t dev;
} bme280_sensor_t;

// @brief Halt the BME280.
static esp_err_t bme280_halt(bme280_sensor_t sensor) {
    uint8_t temp_sampling = BME280_SAMPLING_04X;
    uint8_t pres_sampling = BME280_SAMPLING_04X;
    uint8_t halt_buf[] = {
        BME280_REG_ADDR_CONFIG, (BME280_STANDBY_1S << 5) | (BME280_FILTER_NONE << 2),
        BME280_REG_ADDR_CTRL_MEAS, (temp_sampling << 5) | (pres_sampling << 2) | BME280_MODE_SLEEP,
    };
    ESP_LOGI(TAG, "halting the sensor");
    return bme280_register_write_bytes(sensor.dev, halt_buf, sizeof(halt_buf));
}

// @brief Initialize the BME280.
static esp_err_t bme280_init(i2c_master_dev_handle_t dev_handle, bme280_sensor_t *sensor) {
    uint8_t data[2];
    esp_err_t err;
    // Read the BME280 WHO_AM_I register.
    err = bme280_register_read(dev_handle, BME280_REG_ADDR_IDENT, data, 1);
    if (err != ESP_OK) {
        return err;
    }
    ESP_LOGI(TAG, "WHO_AM_I = %X", data[0]);
    if (data[0] != 0x60) {
        ESP_LOGE(TAG, "The identity does not match BME280, must be 0x60.  Exiting");
        return ESP_ERR_INVALID_STATE;
    }
    sensor->dev = dev_handle;

    // TODO: read calibration.
    return ESP_OK;
}

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

    /* Demonstrate writing by halting the BME280 */
    ESP_ERROR_CHECK(bme280_halt(sensor));
    i2c_master_deinit(bus_handle, dev_handle);
}
