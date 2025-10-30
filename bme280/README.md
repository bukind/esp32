| Supported Targets | ESP32-C3 |
| ----------------- | -------- |

# Weather station

## Overview

The bme280reader is a weather station unit to read temperature, pressure and relative humidity via BME280 sensor.

BME280 is connected to the ESP32-C3 via I2C bus.

The data is sent to the HTTP server.

## How to use example

### Hardware Required

To run this example, you should have an ESP32-C3 Supermini development board as well as BME280 sensor board.

### Configuration

**Note:** The following pin assignments are used by default, you can change these in the `menuconfig` .

|                  | SDA             | SCL           |
| ---------------- | -------------- | -------------- |
| ESP I2C Master   | I2C_MASTER_SDA | I2C_MASTER_SCL |
| BME280  Sensor   | SDA            | SCL            |

For the actual default value of `I2C_MASTER_SDA` and `I2C_MASTER_SCL` see `I2C Configuration` in `menuconfig`.

**Note:** There's no need to add an external pull-up resistors for SDA/SCL pin, because the driver will enable the internal pull-up resistors.

It is also required to setup the WiFi parameters, as well as HTTP server URL.

TODO: Add the description for that.

### Build and Flash

Enter `idf.py -p PORT flash monitor` to build, flash and monitor the
project.  On my Rocky8 Linux machine the `PORT` value is `/dev/ttyACM0`.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html) for full steps to configure and use ESP-IDF to build projects.

## Example Output

```bash
I (256) phy_init: Saving new calibration data due to checksum failure or
outdated calibration data, mode(2)
I (256) wifi:mode : sta (XX:XX:XX:XX:XX:XX)
...
I (726) wifi:state: init -> auth (0xb0)
I (766) wifi:state: auth -> assoc (0x0)
I (776) wifi:state: assoc -> run (0x10)
I (806) wifi:connected with XXXXXXX, aid = 2, channel 4, BW20, bssid = XX:XX:XX:XX:XX:XX
I (806) wifi:security: WPA2-PSK, phy: bgn, rssi: -71
...
I (2486) wifi station: got ip:XXX.XXX.XXX.XXX
I (2486) wifi station: connected to ap SSID:XXXXXXX password:*****
I (5006) weather: system time is set to 2025-10-30 06:14:48
I (5006) weather: I2C initialized successfully
I (5006) bme280: WHO_AM_I = 60
I (5016) bme280: Calibration is read
I (5016) weather: producer task is created
I (5016) weather: producer is unlocked @ 2025-10-30 06:14:48
I (5016) weather: user_data has: buf=3fc9dedc buflen=513 gotlen=0
I (5016) weather: Initialized HTTP client with user data @ 3fc9ded0
I (5016) weather: set connection header: (0) ESP_OK
I (5016) weather: set keep-alive header: (0) ESP_OK
...
I (5046) bme280: measurement complete: T=21 P=100361 H=64
I (5046) weather: store item#0, @2025-10-30 06:14:48 avail=19
...
```

## Troubleshooting

At your own risk.
