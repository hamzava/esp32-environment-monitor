#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <ssd1306.h>
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "i2c.h"
#include "ssd1306_wrapper.h"

#define I2C_SDA         21
#define I2C_SCL         22
#define I2C_FREQ_HZ     100000
#define OLED_ADDR       0x3C

static const char *TAG_SSD1306 = "SSD1306";

i2c_master_dev_handle_t ssd1306_i2c_handle;

ssd1306_config_t ssd1306_dev_cfg;

static ssd1306_handle_t ssd1306_dev_handle = NULL;

void ssd1306_cfg_and_init(void){

     ssd1306_dev_cfg = (ssd1306_config_t)
    {
        .i2c_address = OLED_ADDR,
        .i2c_clock_speed = I2C_FREQ_HZ,
        .panel_size = 1,
        .offset_x = 0,
        .flip_enabled = false,
        .display_enabled = true,
    };

    ESP_LOGI(TAG_SSD1306, "OLED Configuration Successful!");

                    // Add SSD1306 to I2C Bus //

    i2c_master_bus_handle_t bus = i2c_manager_get_bus();

    for(uint8_t i=0; i<10; i++){

    esp_err_t ret = i2c_master_probe(bus, OLED_ADDR, 100);
    ESP_LOGI(TAG_SSD1306, "Connecting to SSD1306... Try #:%d\n", i );
    
    if (ret == ESP_OK)
    {   
        ssd1306_init(bus, &ssd1306_dev_cfg, &ssd1306_dev_handle);
        ESP_LOGI(TAG_SSD1306, "OLED Initialization Successful!");
        vTaskDelay(pdMS_TO_TICKS(1000));
        break;
    }
    else{
        ESP_LOGI(TAG_SSD1306, "OLED Initialization Failed!");
    }

    }

    //                 // Enable Display //

    // esp_err_t ret = ssd1306_enable_display(ssd1306_dev_handle);

    // if (ret == ESP_OK)
    // {   
    //     ESP_LOGI(TAG_SSD1306, "OLED Display Enabled Successfully!");
    //     vTaskDelay(pdMS_TO_TICKS(1000));
    //     break;
    // }
    // else{
    //     ESP_LOGI(TAG_SSD1306, "OLED Display Initialization Failed!");
    // }

};

void display_x3_text(void){

                    // Display x3 text
        ESP_LOGI(TAG_SSD1306, "Display x3 Text");
        ssd1306_clear_display(ssd1306_dev_handle, false);
        ssd1306_set_contrast(ssd1306_dev_handle, 0xff);
        ssd1306_display_text_x3(ssd1306_dev_handle, 0, "Hello", false);
        vTaskDelay(3000 / portTICK_PERIOD_MS);

}

void clear_display(void){
    
    ssd1306_clear_display(ssd1306_dev_handle, false);

}

void test_display(void) {

    ESP_LOGI(TAG_SSD1306, "Starting Welcome Text...");
    
    ssd1306_clear_display(ssd1306_dev_handle, false);
    
    ssd1306_display_text(ssd1306_dev_handle, 2, "     Welcome", false);

    vTaskDelay(pdMS_TO_TICKS(5));

    ssd1306_display_text(ssd1306_dev_handle, 3, "     to the", false);

    vTaskDelay(pdMS_TO_TICKS(5));

    ssd1306_display_text(ssd1306_dev_handle, 4, "   Environment", false);

    vTaskDelay(pdMS_TO_TICKS(5));

    ssd1306_display_text(ssd1306_dev_handle, 5, "     Monitor", false);

    vTaskDelay(pdMS_TO_TICKS(5));
    

    vTaskDelay(pdMS_TO_TICKS(10000));

    ssd1306_clear_display(ssd1306_dev_handle, false);
}

void display_weather_data(float temp, float hum, float pres) {
 
    char line2[32];

    char line4[32];
    
    char line6[32];
       
    // Format and display
    snprintf(line2, sizeof(line2), "  Temp:%.1f C", temp);

    snprintf(line4, sizeof(line4), "  Hum :%.1f %%", hum);
    
    snprintf(line6, sizeof(line6), "  Pres:%.1fhPa", pres);
    
    ssd1306_display_text(ssd1306_dev_handle, 1, line2, false);
    
    ssd1306_display_text(ssd1306_dev_handle, 3, line4, false);
    
    ssd1306_display_text(ssd1306_dev_handle, 5, line6, false);

    draw_rectangle();
    
    vTaskDelay(pdMS_TO_TICKS(10000));

    ssd1306_clear_display(ssd1306_dev_handle, false);
    
}

void display_ltr390_data(float ambient_light, float ultraviolet_index){

    char line3[32];

    char line6[32];
   
    snprintf(line3, sizeof(line3), "  UV Index:%.2f", ultraviolet_index);

    snprintf(line6, sizeof(line6), "  Lux:%.1f", ambient_light);

    ssd1306_display_text(ssd1306_dev_handle, 2, line3, false);

    vTaskDelay(pdMS_TO_TICKS(10));

    ssd1306_display_text(ssd1306_dev_handle, 5, line6, false);

    draw_rectangle();

    vTaskDelay(pdMS_TO_TICKS(10000)); 

    ssd1306_clear_display(ssd1306_dev_handle, false);
}

void draw_rectangle(void){

    ssd1306_display_rectangle(ssd1306_dev_handle, 0, 0, 128, 64, false);

}