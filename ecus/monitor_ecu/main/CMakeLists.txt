idf_component_register(
    SRCS "src/monitor_ecu.c"
    INCLUDE_DIRS "include"
    REQUIRES wifi_connection_module
             mqtt_connection_module
             esp_mesh_connection_module
             routing_module
             ota_module
             sd_storage_module
             logger_module
             telemetry_module
             obdii_module
             diagnosis_module
             alert_module
)
