/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// BME280 Reader via I2C.

#include "wificonn.h"
#include "bme280.h"
#if WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif
#include "httpcli.h"

static const char *TAG = "weather";

static char errbuf[64];
#define DBGERR(err) err, esp_err_to_name_r(err, errbuf, sizeof(errbuf))

#define STACK_SIZE 0x800

static void producerTask(void *sensorParam) {
    bme280_sensor_t *sensor = (bme280_sensor_t*)(sensorParam);
    const TickType_t readPeriod = 1000 / portTICK_PERIOD_MS;
    for (int i = 0; i < 100; i++) {
        ESP_ERROR_CHECK(bme280_measure_once(*sensor));
        vTaskDelay(readPeriod);
    }
    vTaskDelete(NULL); // delete itself.
}

void app_main(void)
{
#if WIFI_NVS_ENABLED
    // Initialize NVS.
    // It seems to be a requirement for WiFi, though they don't tell it anywhere.
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
#else
    ESP_LOGI(TAG, "skipping NVS initialization");
#endif

    wificonn_init_sta();

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    bme280_i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    bme280_sensor_t sensor;
    esp_err_t err = bme280_init(dev_handle, &sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bme280: %d", err);
        bme280_i2c_master_deinit(bus_handle, dev_handle);
        return;
    }

    TaskHandle_t producerHandle = NULL;
    xTaskCreate(producerTask, "producer", STACK_SIZE, &sensor, tskIDLE_PRIORITY, &producerHandle);
    if (producerHandle != NULL) {
        ESP_LOGI(TAG, "producer task is created");
    }

    // HTTP Client.
    char response_buf[512+1];
    httpcli_user_data_t user_data = HTTPCLI_USER_DATA_INIT(response_buf, sizeof(response_buf));
    ESP_LOGI(TAG, "user_data has: buf=%x buflen=%d gotlen=%d", user_data.buffer, user_data.buflen, user_data.gotlen);
    const char* url = "http://192.168.5.2:8080/tmp.txt";
    esp_http_client_method_t method = HTTP_METHOD_GET;
    esp_http_client_config_t http_config = {
        .url = url,
        .method = method,
        .event_handler = httpcli_event_handler,
        .user_data = &user_data,
        .keep_alive_enable = true,
        .keep_alive_idle = 5,
        .keep_alive_interval = 5,
        .keep_alive_count = 3,
    };
    esp_http_client_handle_t client = esp_http_client_init(&http_config);
    if (client == NULL) {
        ESP_LOGE(TAG, "cannot initialize HTTP client");
        return;
    }
    ESP_LOGI(TAG, "Initialized HTTP client with user data @ %x", &user_data);
    err = esp_http_client_set_header(client, "Connection", "keep-alive");
    ESP_LOGI(TAG, "set connection header: (%d) %s", DBGERR(err));
    esp_http_client_set_header(client, "Keep-Alive", "timeout=60, max=10");
    ESP_LOGI(TAG, "set keep-alive header: (%d) %s", DBGERR(err));

    err = esp_http_client_perform(client);
    ESP_LOGI(TAG, "HTTP perform: (%d) %s", DBGERR(err));
    ESP_LOGI(TAG, "user_data has: buf=%x buflen=%d gotlen=%d", user_data.buffer, user_data.buflen, user_data.gotlen);
    ESP_LOG_BUFFER_HEX(TAG, user_data.buffer, user_data.gotlen);

    sleep(100);

    // Reset the response.
    user_data.gotlen = 0;
    // The second call does not work yet!
    const char* url2 = "http://192.168.5.2:8080/tmp2.txt";
    err = esp_http_client_set_url(client, url2);
    ESP_LOGI(TAG, "Set URL: (%d) %s", DBGERR(err));
    err = esp_http_client_set_method(client, method);
    ESP_LOGI(TAG, "Set method: (%d) %s", DBGERR(err));
    err = esp_http_client_perform(client);
    ESP_LOGI(TAG, "HTTP perform: (%d) %s", DBGERR(err));
    ESP_LOGI(TAG, "user_data has: buf=%x buflen=%d gotlen=%d", user_data.buffer, user_data.buflen, user_data.gotlen);
    ESP_LOG_BUFFER_HEX(TAG, user_data.buffer, user_data.gotlen);
    esp_http_client_cleanup(client);

    // Halting the sensor.
    ESP_ERROR_CHECK(bme280_halt(sensor));
    bme280_i2c_master_deinit(bus_handle, dev_handle);

    wificonn_deinit();
}
