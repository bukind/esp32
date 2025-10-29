// BME280 Reader via I2C.

#include <unistd.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_check.h"
#include "driver/i2c_master.h"
#include "bme280.h"

static const char *TAG = "bme280";

#define I2C_MASTER_TIMEOUT_MS       1000
#define I2C_MASTER_SCL_IO           CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock */
#define I2C_MASTER_SDA_IO           CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  */
#define I2C_MASTER_NUM              I2C_NUM_0                   /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ          CONFIG_I2C_MASTER_FREQUENCY /*!< I2C master clock frequency */
// #define I2C_MASTER_TX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */
// #define I2C_MASTER_RX_BUF_DISABLE   0                           /*!< I2C master doesn't need buffer */

// Most of the BME280 code is reverse-engineered from the https://github.com/periph/devices/tree/v3.7.4/bmxx80.

#define BME280_SENSOR_ADDR         0x76        /*!< Address of the BME280 sensor */

// @brief i2c master initialization
void bme280_i2c_master_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
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

void bme280_i2c_master_deinit(i2c_master_bus_handle_t bus_handle, i2c_master_dev_handle_t dev_handle) {
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(dev_handle));
    ESP_ERROR_CHECK(i2c_del_master_bus(bus_handle));
    ESP_LOGI(TAG, "I2C de-initialized successfully");
}

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

// @brief Halt the BME280.
esp_err_t bme280_halt(bme280_sensor_t sensor) {
    uint8_t temp_sampling = BME280_SAMPLING_04X;
    uint8_t pres_sampling = BME280_SAMPLING_04X;
    uint8_t halt_buf[] = {
        BME280_REG_ADDR_CONFIG, (BME280_STANDBY_1S << 5) | (BME280_FILTER_NONE << 2),
        BME280_REG_ADDR_CTRL_MEAS, (temp_sampling << 5) | (pres_sampling << 2) | BME280_MODE_SLEEP,
    };
    ESP_LOGI(TAG, "halting the sensor");
    return bme280_register_write_bytes(sensor.dev, halt_buf, sizeof(halt_buf));
}

static useconds_t bme280_calculate_delay_us(bme280_sensor_t sensor) {
    const uint8_t sv[] = {0, 1, 2, 4, 8, 16};
    useconds_t delay_us = 1000;
    if (sensor.tsamp != BME280_SAMPLING_OFF) {
        delay_us += 2000 * sv[sensor.tsamp];
    }
    if (sensor.psamp != BME280_SAMPLING_OFF) {
        delay_us += 2000 * sv[sensor.psamp] + 500;
    }
    if (sensor.hsamp != BME280_SAMPLING_OFF) {
        delay_us += 2000 * sv[sensor.hsamp] + 500;
    }
    return delay_us;
}

// @brief Initialize the BME280.
esp_err_t bme280_init(i2c_master_dev_handle_t dev_handle, bme280_sensor_t *sensor) {
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

    // TODO: make the values configurable.
    sensor->mode = BME280_MODE_SLEEP;
    sensor->tsamp = BME280_SAMPLING_04X;
    sensor->psamp = BME280_SAMPLING_04X;
    sensor->hsamp = BME280_SAMPLING_04X;
    sensor->standby = BME280_STANDBY_1S;
    sensor->filter = BME280_FILTER_NONE;
    sensor->delay_us = bme280_calculate_delay_us(*sensor);

    // Read calibration.
    uint8_t tph[BME280_CAL1_SIZE];
    err = bme280_register_read(sensor->dev, BME280_REG_ADDR_CAL1, tph, BME280_CAL1_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read cal1: %d", err);
        return err;
    }

    uint8_t h[BME280_CAL2_SIZE];
    err = bme280_register_read(sensor->dev, BME280_REG_ADDR_CAL2, h, BME280_CAL2_SIZE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Cannot read cal2: %d", err);
        return err;
    }

    ESP_LOGI(TAG, "Calibration is read");
    sensor->cal.t1 = ((uint16_t)tph[0]) | ((uint16_t)tph[1])<<8;
    sensor->cal.t2 = ((int16_t)tph[2]) | ((int16_t)tph[3])<<8;
    sensor->cal.t3 = ((int16_t)tph[4]) | ((int16_t)tph[5])<<8;
    sensor->cal.p1 = ((uint16_t)tph[6]) | ((uint16_t)tph[7])<<8;
    sensor->cal.p2 = ((int16_t)tph[8]) | ((int16_t)tph[9])<<8;
    sensor->cal.p3 = ((int16_t)tph[10]) | ((int16_t)tph[11])<<8;
    sensor->cal.p4 = ((int16_t)tph[12]) | ((int16_t)tph[13])<<8;
    sensor->cal.p5 = ((int16_t)tph[14]) | ((int16_t)tph[15])<<8;
    sensor->cal.p6 = ((int16_t)tph[16]) | ((int16_t)tph[17])<<8;
    sensor->cal.p7 = ((int16_t)tph[18]) | ((int16_t)tph[19])<<8;
    sensor->cal.p8 = ((int16_t)tph[20]) | ((int16_t)tph[21])<<8;
    sensor->cal.p9 = ((int16_t)tph[22]) | ((int16_t)tph[23])<<8;
    sensor->cal.h1 = tph[25];
    sensor->cal.h2 = ((int16_t)h[0]) | ((int16_t)h[1])<<8;
    sensor->cal.h3 = h[2];
    sensor->cal.h4 = ((int16_t)h[3])<<4 | (((int16_t)h[4])&0xF);
    sensor->cal.h5 = ((int16_t)h[4])>>4 | ((int16_t)h[5])<<4;
    sensor->cal.h6 = ((int8_t)h[6]);

    uint8_t cmd[] = {
        // ctrl_meas; put it to sleep otherwise the config update may be
        // ignored. This is really just in case the device was somehow put
        // into normal but was not Halt'ed.
        BME280_REG_ADDR_CTRL_MEAS, sensor->tsamp<<5 | sensor->psamp<<2 | BME280_MODE_SLEEP,
        // ctrl_hum
        BME280_REG_ADDR_CTRL_HUM, sensor->hsamp,
        // config
        BME280_REG_ADDR_CONFIG, sensor->standby<<5 | sensor->filter<<2,
        // As per page 25, ctrl_meas must be re-written last.
        BME280_REG_ADDR_CTRL_MEAS, sensor->tsamp<<5 | sensor->psamp<<2 | sensor->mode,
    };
    return bme280_register_write_bytes(sensor->dev, cmd, sizeof(cmd));
}

// Wait until bme280 is in the idle state.
static esp_err_t bme280_wait_idle(bme280_sensor_t sensor) {
    while (true) {
        uint8_t status;
        esp_err_t err = bme280_register_read(sensor.dev, BME280_REG_ADDR_STATUS, &status, 1);
        ESP_RETURN_ON_ERROR(err, TAG, "failed to read status: %d", err);
        if ((status & 8) == 0) {
            // Bit3 is cleared -- the sensor is idle.
            return ESP_OK;
        }
    }
}

// return tfine.  Use (tfine*5+128)>>8 to get centicelsius.
static int32_t bme280_compensate_temp_fine(bme280_calibration_t c, int32_t raw) {
    int32_t x = (((raw>>3) - (((int32_t)c.t1)<<1)) * ((int32_t)c.t2)) >> 11;
    int32_t y = (((((raw>>4) - ((int32_t)c.t1)) * ((raw>>4) - ((int32_t)c.t1))) >> 12) * ((int32_t)c.t3)) >> 14;
    return x + y;
}

// return pressure in 24.8 (24 integer bits and 8 fractional).  Divide by 256 to get Pascal.
static uint32_t bme280_compensate_pressure(bme280_calibration_t c, int32_t raw, int32_t tfine) {
    int64_t x = ((int64_t)tfine) - 128000;
    int64_t y = x * x * ((int64_t)c.p6);
    y += (x * ((int64_t)c.p5)) << 17;
    y += ((int64_t)c.p4) << 35;
    x = ((x*x*((int64_t)c.p3))>>8) + ((x * ((int64_t)c.p2)) << 12);
    x = (((((int64_t)1)<<47) + x) * ((int64_t)c.p1)) >> 33;
    if (x == 0) {
        return 0;
    }
    int64_t p = ((((1048576 - ((int64_t)raw)) << 31) - y) * 3125) / x;
    x = (((int64_t)c.p9) * (p >> 13) * (p >> 13)) >> 25;
    y = (((int64_t)c.p8) * p) >> 19;
    return ((uint32_t)((p + x + y) >> 8) + (((int64_t)c.p7) << 4));
}

// Return relative humidity in %RH in 22.10 format (22 integer bits and 10 fractional).
// Divide by 1024 to get %.
static uint32_t bme280_compensate_humidity(bme280_calibration_t c, int32_t raw, int32_t tfine) {
    int32_t x = tfine - 76800;
    int32_t x1 = (raw<<14) - (((int32_t)c.h4)<<20) - ((int32_t)c.h5)*x;
    int32_t x2 = (x1 + 16384) >> 15;
    int32_t x3 = (x * ((int32_t)c.h6)) >> 10;
    int32_t x4 = (x * ((int32_t)c.h3)) >> 11;
    int32_t x5 = (x3 * (x4 + 32768)) >> 10;
    int32_t x6 = ((x5+2097152)*((int32_t)c.h2) + 8192) >> 14;
    x = x2 * x6;
    x = x - (((((x>>15)*(x>>15))>>7)*((int32_t)c.h1))>>4);
    if (x < 0) {
        return 0;
    }
    if (x > 419430400) {
        return 419430400 >> 12;
    }
    return ((uint32_t)x >> 12);
}

// TOOD: pass struct to hold results.
esp_err_t bme280_measure_once(bme280_sensor_t sensor, bme280_readout_t *measurement) {
    uint8_t cmd[] = {
        BME280_REG_ADDR_CTRL_MEAS, sensor.tsamp<<5 | sensor.psamp<<2 | BME280_MODE_FORCED,
    };
    esp_err_t err = bme280_register_write_bytes(sensor.dev, cmd, sizeof(cmd));
    ESP_RETURN_ON_ERROR(err, TAG, "failed to start measurements: %d", err);
    usleep(sensor.delay_us);
    err = bme280_wait_idle(sensor);
    if (err != ESP_OK) {
        return err;
    }
    uint8_t data[BME280_DATA_SIZE]; // press + temp + hum
    err = bme280_register_read(sensor.dev, BME280_REG_ADDR_DATA, data, sizeof(data));
    ESP_RETURN_ON_ERROR(err, TAG, "cannot read data: %d", err);

    // 20 bits per measurement.
    int32_t traw = ((int32_t)data[3])<<12 | ((int32_t)data[4])<<4 | ((int32_t)data[5])>>4;
    int32_t tfine = bme280_compensate_temp_fine(sensor.cal, traw);
    int32_t tcenticelsius = (tfine*5 + 128)>>8;

    uint32_t ppasc256 = 0;
    if (sensor.psamp != BME280_SAMPLING_OFF) {
        int32_t praw = ((int32_t)data[0])<<12 | ((int32_t)data[1])<<4 | ((int32_t)data[2])>>4;
        ppasc256 = bme280_compensate_pressure(sensor.cal, praw, tfine);
    }
    uint32_t hum = 0;
    if (sensor.hsamp != BME280_SAMPLING_OFF) {
        int32_t hraw = ((int32_t)data[6])<<8 | ((int32_t)data[7]);
        hum = bme280_compensate_humidity(sensor.cal, hraw, tfine);
    }
    ESP_LOGI(TAG, "measurement complete: T=%d P=%d H=%d", tcenticelsius/100, ppasc256/256, hum/1024);
    if (measurement != NULL) {
        measurement->celsius100 = tcenticelsius;
        measurement->pascal256 = ppasc256;
        measurement->relhum1024 = hum;
    }
    return ESP_OK;
}
