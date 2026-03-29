idf_component_register(SRCS "esp_ot_cli.c"
                       INCLUDE_DIRS ".")

# Uncomment the line below to configure as End Device
#target_compile_definitions(${COMPONENT_LIB} PRIVATE CONFIG_DEVICE_TYPE_END_DEVICE)
