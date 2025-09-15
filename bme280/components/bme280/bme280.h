#pragma once

#include <unistd.h>
#include "esp_log.h"
#include "driver/i2c_master.h"

#ifdef __cplusplus
extern "C" {
#endif

// Most of the BME280 code is reverse-engineered from the https://github.com/periph/devices/tree/v3.7.4/bmxx80.

#define BME280_REG_ADDR_IDENT      0xD0 // Identity register, should return 0x60 for BME280.
#define BME280_REG_ADDR_CAL1       0x88
#define BME280_REG_ADDR_CAL2       0xE1
#define BME280_REG_ADDR_CTRL_HUM   0xF2
#define BME280_REG_ADDR_STATUS     0xF3
#define BME280_REG_ADDR_CTRL_MEAS  0xF4
#define BME280_REG_ADDR_CONFIG     0xF5
#define BME280_REG_ADDR_DATA       0xF7

#define BME280_SAMPLING_OFF        0
#define BME280_SAMPLING_04X        3
#define BME280_STANDBY_1S          5
#define BME280_FILTER_NONE         0
#define BME280_MODE_SLEEP          0
#define BME280_MODE_FORCED         1 // perform one measurement, return to sleep mode.
#define BME280_CAL1_SIZE           0xA2 - 0x88
#define BME280_CAL2_SIZE           0xE8 - 0xE1
#define BME280_DATA_SIZE           8

typedef struct {
    uint16_t t1;
    int16_t  t2, t3;
    uint16_t p1;
    int16_t  p2, p3, p4, p5, p6, p7, p8, p9;
    int16_t  h2; // Reordered for packing
    uint8_t  h1, h3;
    int16_t  h4, h5;
    int8_t   h6;
} bme280_calibration_t;

typedef struct {
    i2c_master_dev_handle_t dev;
    bme280_calibration_t    cal;
    useconds_t              delay_us;
    uint8_t                 mode;
    uint8_t                 tsamp;
    uint8_t                 psamp;
    uint8_t                 hsamp;
    uint8_t                 standby;
    uint8_t                 filter;
} bme280_sensor_t;

esp_err_t bme280_halt(bme280_sensor_t sensor);

esp_err_t bme280_init(i2c_master_dev_handle_t dev_handle, bme280_sensor_t *sensor);

esp_err_t bme280_measure_once(bme280_sensor_t sensor);

#ifdef __cplusplus
}
#endif
