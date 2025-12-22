idf_component_register(SRCS "mesh_log_stream.c" "mesh_time_sync.c" "log_time_vprintf.c" "mesh_main.c"
                        "legacy_proto.c"
                        "pump_node.c"
                        "stack_monitor.c"
                        "mesh_time_sync.c"
                        "log_time_vprintf.c"
                    PRIV_REQUIRES esp_wifi esp_driver_gpio esp_adc driver nvs_flash
                    INCLUDE_DIRS "." "include")
