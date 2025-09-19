idf_component_register(SRCS "bme280reader.c"
                       INCLUDE_DIRS "."
                       REQUIRES wificonn httpcli nvs_flash)
