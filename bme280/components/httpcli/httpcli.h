#pragma once

#include "esp_http_client.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    char   *buffer;
    size_t buflen;
    size_t gotlen;  // how many bytes got into the buffer.
} httpcli_user_data_t;

#define HTTPCLI_USER_DATA_INIT(buf, bufsize) { \
    .buffer = (buf), \
    .buflen = (bufsize), \
    .gotlen = 0 \
}

// Standard event handler when the client has attached the httpcli_user_data_t data.
esp_err_t httpcli_event_handler(esp_http_client_event_t *evt);

#ifdef __cplusplus
}
#endif
