idf_component_register(SRCS "mesh_main.c"
                        "legacy_proto.c"
                        "pump_node.c"
			"pump_controller.c"
			"pump_controller_selftest.c"
			"pump_driver.c"
			"water_level_sensor.c"
                        "stack_monitor.c"
                        "mesh_v2_link.c"
                    PRIV_REQUIRES keemash_mesh_core esp_wifi esp_driver_gpio esp_adc driver nvs_flash heap esp_timer app_update spi_flash bootloader_support mbedtls
                    INCLUDE_DIRS "." "include")
