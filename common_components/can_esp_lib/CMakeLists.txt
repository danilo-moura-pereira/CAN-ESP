idf_component_register(
    SRC_DIRS "src"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS 
    PRIV_REQUIRES esp_timer
    REQUIRES driver
)
