idf_component_register(
    SRCS 
        "bme280.c"
        "bme280_api.c" 
    INCLUDE_DIRS 
        "include" 

    REQUIRES 
        driver 
        i2c_driver
        
)   