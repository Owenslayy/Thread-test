idf_component_register(SRCS "esp_ot_cli.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_event
                                esp_netif
                                openthread
                                nvs_flash
                                vfs
                                driver
                                freertos
                                lwip
                                esp_driver_gpio
                                esp_driver_uart
                                esp_driver_ledc
                                led_strip)

# Uncomment the line below to configure as End Device
#target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_DEVICE_TYPE_END_DEVICE)
