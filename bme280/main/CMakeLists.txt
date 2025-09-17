idf_component_register(SRCS "i2c_basic_example_main.c"
                       INCLUDE_DIRS "."
                       REQUIRES wificonn httpcli bme280 nvs_flash)
