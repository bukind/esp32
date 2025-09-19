idf_component_register(SRCS "bme280reader.c"
                       INCLUDE_DIRS "."
                       REQUIRES wificonn httpcli bme280 nvs_flash)
