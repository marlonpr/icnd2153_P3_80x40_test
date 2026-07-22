idf_component_register(
    SRCS "main.cpp"
    
    INCLUDE_DIRS "."
    
    PRIV_REQUIRES esp_lcd 
    esp_timer
    esp_mm
    esp_hw_support
    esp_driver_gpio
    esp_driver_dma
)
