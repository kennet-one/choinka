idf_component_register(SRCS "mesh_main.c"
                        "legacy_proto.c"
                        "pump_node.c"
                        "stack_monitor.c"
                    PRIV_REQUIRES esp_wifi esp_driver_gpio esp_adc driver nvs_flash
                    INCLUDE_DIRS "." "include")
