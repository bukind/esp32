/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

// BME280 Reader via I2C.

#include <time.h>
#include <sys/time.h>
#include <inttypes.h>
#include "wificonn.h"
#include "bme280.h"
#if WIFI_NVS_ENABLED
#include "nvs_flash.h"
#endif
#include "httpcli.h"
#include "esp_netif_sntp.h"
#include "esp_sntp.h"

static const char *TAG = "weather";

static char errbuf[64];
#define DBGERR(err) err, esp_err_to_name_r(err, errbuf, sizeof(errbuf))

#define PRODUCER_STACK_SIZE 0x800
#define PRODUCER_PRIORITY 2
#define CONSUMER_STACK_SIZE 0x800
#define CONSUMER_PRIORITY 2
#define DATABUFFER_SIZE 20

// The storage for one weather measurement: time + BME280 readout info.
typedef struct {
    time_t           timestamp;
    bme280_readout_t data;
} readout_data_t;

// The ring buffer for the weather measurements.
// Used to pass the measuments between the producer and the consumer tasks.
typedef struct {
    readout_data_t *data;     // pointer to a data array
    uint32_t       buf_size;  // total size
    uint32_t       index;     // current index
} databuffer_t;

#define DATABUFFER_INC(x) if (++((x).index) >= (x).buf_size) { (x).index = 0; }

// The data passed into the consumer task.
typedef struct {
    TaskHandle_t producer;
    databuffer_t buffer;
} consumer_data_t;

// The data passed into the producer task.
typedef struct {
    bme280_sensor_t *sensor;
    TaskHandle_t    consumer;
    databuffer_t buffer;
    uint32_t     avail;
} producer_data_t;

// Format the time t as a YYYY-MM-DD HH:MM:SS string in the provided buffer.
static const char* timestr(time_t t, char *buf, size_t bufsize) {
    struct tm timeinfo = {0};
    localtime_r(&t, &timeinfo);
    strftime(buf, bufsize, "%Y-%m-%d %H:%M:%S", &timeinfo);
    return buf;
}

// The producer task.
// *  Stays locked initially waiting for a direct to task message to proceed.
//    This is needed to synchronize with the consumer task creation.
// *  In a loop read measurements from BME280 sensor and pass them
//    to the consumer task via the ring buffer.  The consumer is
//    notified about the outstanding data via the direct to task notification.
// *  TODO: work on the flag to exit the task gracefully.
static void producerTask(void *producerData) {
    producer_data_t *data = (producer_data_t*)(producerData);
    const TickType_t readPeriod = 1000 / portTICK_PERIOD_MS;

    // Initial lock for the producer to sync the setup.
    for (;;) {
        uint32_t got = ulTaskNotifyTake(pdTRUE, readPeriod);
        if (got != 0) {
            break;
        }
    }
    struct timeval tv = {0};
    gettimeofday(&tv, NULL);
    char timebuf[40];
    ESP_LOGI(TAG, "producer is unlocked @ %s", timestr(tv.tv_sec, timebuf, sizeof(timebuf)));

    uint32_t min_avail = data->buffer.buf_size / 2;
    for (int i = 0; i < 100; i++) {
        if (data->avail > 0 ) {
            char timebuf[40];
            readout_data_t *item = data->buffer.data + data->buffer.index;
            ESP_ERROR_CHECK(bme280_measure_once(*data->sensor, &(item->data)));
            item->timestamp = time(NULL);
            --data->avail;
            ESP_LOGI(TAG, "store item#%d, @%s avail=%d", data->buffer.index, timestr(item->timestamp, timebuf, sizeof(timebuf)), data->avail);
            DATABUFFER_INC(data->buffer);
            xTaskNotifyGive(data->consumer);
        }
        if (data->avail <= min_avail) {
            // ESP_LOGI(TAG, "producer -> consumer: 1");
            // Quickly check if we have any return.
            uint32_t gotBack = ulTaskNotifyTake(pdTRUE, 1);
            if (gotBack != 0) {
                ESP_LOGI(TAG, "producer <- consumer: %d", gotBack);
                data->avail += gotBack;
            }
        }
        vTaskDelay(readPeriod);
    }
    vTaskDelete(NULL); // delete itself.
}

// The consumer task.
// *  In a loop, waits for the direct to task notification
//    from the producer.
// *  When a notification is received, reads the measurement
//    data from the ring buffer.
// *  TODO: send the data to the HTTP server.
// *  TODO: work on the flag to exit the task gracefully.
static void consumerTask(void *consumerData) {
    const TickType_t waitPeriod = 1000 / portTICK_PERIOD_MS;
    consumer_data_t *data = (consumer_data_t*)(consumerData);
    if (data == NULL) {
        vTaskDelete(NULL);
    }
    for (;;) {
        uint32_t gotItems = ulTaskNotifyTake(pdTRUE, waitPeriod);
        if (gotItems == 0) {
            // Didn't get a notification.
            continue;
        }
        ESP_LOGI(TAG, "consumer got %d items", gotItems);
        // TODO: process them here.
        for (uint32_t i = 0; i < gotItems; i++) {
            readout_data_t *item = data->buffer.data + data->buffer.index;
            char timebuf[40];
            ESP_LOGI(TAG, "processing item#%" PRIu32 ": %s T=%" PRId32 " P=%" PRIu32 " H=%" PRIu32,
                     data->buffer.index,
                     timestr(item->timestamp, timebuf, sizeof(timebuf)),
                     item->data.celsius100/100,
                     item->data.pascal256/256,
                     item->data.relhum1024/1024);
            DATABUFFER_INC(data->buffer);
            // Return it back.
            xTaskNotifyGive(data->producer);
        }
        // Extra wait to emulate the delay in the task.
        vTaskDelay(waitPeriod * 2);
    }
}

// Proceed processing after WiFi is initialized.
void proceed_after_wifi(void) {
    esp_err_t err;

    i2c_master_bus_handle_t bus_handle;
    i2c_master_dev_handle_t dev_handle;
    bme280_i2c_master_init(&bus_handle, &dev_handle);
    ESP_LOGI(TAG, "I2C initialized successfully");

    bme280_sensor_t sensor;
    err = bme280_init(dev_handle, &sensor);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize bme280: %d", err);
        bme280_i2c_master_deinit(bus_handle, dev_handle);
        return;
    }

    readout_data_t buffer[DATABUFFER_SIZE];

    TaskHandle_t consumerHandle = NULL;
    consumer_data_t consumerData = {
        .producer = NULL,
        .buffer.data = buffer,
        .buffer.buf_size = sizeof(buffer)/sizeof(buffer[0]),
        .buffer.index = 0,
    };
    xTaskCreate(consumerTask, "consumer", CONSUMER_STACK_SIZE, &consumerData, CONSUMER_PRIORITY, &consumerHandle);

    TaskHandle_t producerHandle = NULL;
    producer_data_t producerData = {
        .sensor = &sensor,
        .consumer = consumerHandle,
        .buffer.data = buffer,
        .buffer.buf_size = sizeof(buffer)/sizeof(buffer[0]),
        .buffer.index = 0,
        .avail = sizeof(buffer)/sizeof(buffer[0]),
    };
    xTaskCreate(producerTask, "producer", PRODUCER_STACK_SIZE, &producerData, PRODUCER_PRIORITY, &producerHandle);
    if (producerHandle != NULL) {
        ESP_LOGI(TAG, "producer task is created");
    }
    consumerData.producer = producerHandle;
    // We can unblock the producer now.
    xTaskNotifyGive(producerHandle);

    // HTTP Client.
    // TODO: move it to the consumer task.
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
}

// Main entry point.
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

    // Set the system time.
    esp_sntp_config_t config = ESP_NETIF_SNTP_DEFAULT_CONFIG("pool.ntp.org");
    esp_netif_sntp_init(&config);
    err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(10000));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "cannot synchronize the system time: (%d) %s", DBGERR(err));
        wificonn_deinit();
        return;
    }
    char timebuf[40];
    time_t curtime = time(NULL);
    ESP_LOGI(TAG, "system time is set to %s", timestr(curtime, timebuf, sizeof(timebuf)));

    proceed_after_wifi();
    wificonn_deinit();
}
