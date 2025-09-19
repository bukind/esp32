/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// BME280 Reader via I2C.

#include "wificonn.h"
#if WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif
#include "httpcli.h"
#include "esp_log.h"

static const char *TAG = "weather";

static char errbuf[64];
#define DBGERR(err) err, esp_err_to_name_r(err, errbuf, sizeof(errbuf))

void app_main(void)
{
    esp_err_t err;
#if WIFI_NVS_ENABLED
    // Initialize NVS.
    // It seems to be a requirement for WiFi, though they don't tell it anywhere.
    err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
#else
    ESP_LOGI(TAG, "skipping NVS initialization");
#endif

    wificonn_init_sta();

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

    sleep(10);

    // Reset the response.
    user_data.gotlen = 0;
    // The second call does not work!!!
    err = esp_http_client_set_url(client, url);
    ESP_LOGI(TAG, "Set URL: (%d) %s", DBGERR(err));
    err = esp_http_client_set_method(client, method);
    ESP_LOGI(TAG, "Set method: (%d) %s", DBGERR(err));
    err = esp_http_client_perform(client);
    ESP_LOGI(TAG, "HTTP perform: (%d) %s", DBGERR(err));
    ESP_LOGI(TAG, "user_data has: buf=%x buflen=%d gotlen=%d", user_data.buffer, user_data.buflen, user_data.gotlen);
    ESP_LOG_BUFFER_HEX(TAG, user_data.buffer, user_data.gotlen);
    esp_http_client_cleanup(client);

    wificonn_deinit();
}
