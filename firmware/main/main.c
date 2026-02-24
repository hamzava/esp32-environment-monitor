#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "bme280_defs.h"
#include "bme280_wrapper.h"
#include "ssd1306.h"
#include "ssd1306_wrapper.h"
#include "i2c.h"
#include "ltr390_wrapper.h"
#include "ltr390uv.h"
#include "driver/gpio.h"


#define I2C_SDA         21
#define I2C_SCL         22

static const char *TAG_APP = "APP";

void app_main(void)
{    

    i2c_init(); 

    i2c_scan();

    ESP_LOGI(TAG_APP, "Starting Application");

    bme280_i2c_init();

    bme280_i2c_config_and_init();

    bme280_settings(BME280_POWERMODE_NORMAL);
    
    bme280_values_t bme_values;

    ltr390_cfg_and_init();

    ltr390_values_t ltr390_values;
  
    ssd1306_cfg_and_init();

    test_display();

    while (1)
    {      
        
        bme280_read_values(&bme_values);

        ESP_LOGI(TAG_APP, "Temp: %.1fC, Hum: %.1f%%, Pres: %.1fhPa",
        bme_values.temperature_c,
        bme_values.humidity_pct,
        bme_values.pressure_hpa);

        display_weather_data(bme_values.temperature_c, bme_values.humidity_pct, bme_values.pressure_hpa);
       
        ltr390_data(&ltr390_values);

        ESP_LOGI(TAG_APP, "Ambient Light:%.2f, UV Index:%.2f",
        ltr390_values.ambient_light,
        ltr390_values.ultraviolet_index);
        
        display_ltr390_data(ltr390_values.ambient_light, ltr390_values.ultraviolet_index);
        

        
    }

}
    


          
    
 


