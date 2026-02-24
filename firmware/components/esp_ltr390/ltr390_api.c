#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "i2c.h"
#include "include/ltr390uv.h"
#include "include/ltr390_wrapper.h"


#define I2C_SDA         21
#define I2C_SCL         22
#define I2C_FREQ_HZ     100000
#define LTR390_ADDR     0x53

static const char *TAG_LTR390 = "LTR390";

ltr390uv_config_t ltr390_dev_cfg;
static ltr390uv_handle_t ltr390_dev_handle;

void ltr390_cfg_and_init(void){

    ltr390_dev_cfg = (ltr390uv_config_t){                      
    .i2c_address               = LTR390_ADDR,
    .i2c_clock_speed           = I2C_FREQ_HZ,  
    .window_factor             = 1, 
    .operation_mode             =0,               
    .als_sensor_resolution     = LTR390UV_SR_18BIT,         
    .als_measurement_rate      = LTR390UV_MR_100MS,         
    .als_measurement_gain      = LTR390UV_MG_X3,            
    .uvs_sensor_resolution     = LTR390UV_SR_18BIT,         
    .uvs_measurement_rate      = LTR390UV_MR_100MS,         
    .uvs_measurement_gain      = LTR390UV_MG_X3,
    };

    ESP_LOGI(TAG_LTR390, "LTR390 Configuration Successful!");
    
    i2c_master_bus_handle_t bus = i2c_manager_get_bus();

    for(uint8_t i=0; i<10; i++){

        esp_err_t ret = i2c_master_probe(bus, LTR390_ADDR, 100);
        ESP_LOGI(TAG_LTR390, "Connecting to LTR390... Try #:%d\n", i );
        
        if (ret == ESP_OK)
        {   
            ltr390uv_init(bus, &ltr390_dev_cfg, &ltr390_dev_handle);
            ESP_LOGI(TAG_LTR390, "LTR390 Initialization Successful!");
            vTaskDelay(pdMS_TO_TICKS(1000));
            break;
        }
        else{
            ESP_LOGI(TAG_LTR390, "LTR390 Initialization Failed!");
        }

    }

    esp_err_t ret = ltr390uv_enable(ltr390_dev_handle);
    if(ret != ESP_OK){
        ESP_LOGI(TAG_LTR390, "Unable to measure data from LTR390!");
    }
    else{
     
        ltr390uv_enable(ltr390_dev_handle);

        ESP_LOGI(TAG_LTR390, "LTR390 Enabled Successfully!");

    }


}

void ltr390_data(ltr390_values_t *out){

    float lux = 0.0f;
    
    float uvi_temp = 0.0f;
 
    ltr390uv_set_mode(ltr390_dev_handle, 0);
    
    vTaskDelay(pdMS_TO_TICKS(150));

    ltr390uv_get_ambient_light(ltr390_dev_handle, &lux);
    
    vTaskDelay(pdMS_TO_TICKS(150));
    
    ltr390uv_set_mode(ltr390_dev_handle, 1);
    
    vTaskDelay(pdMS_TO_TICKS(150));

    ltr390uv_get_ultraviolet_index(ltr390_dev_handle, &uvi_temp);
    
    vTaskDelay(pdMS_TO_TICKS(150));
    
    out->ambient_light = lux;
    
    out->ultraviolet_index = uvi_temp;
    
}