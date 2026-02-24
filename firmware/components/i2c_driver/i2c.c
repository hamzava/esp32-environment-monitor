#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // needed for ets_delay_us()

#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_FREQ_HZ 100000

static const char *TAG_I2C ="I2C";

i2c_master_bus_handle_t bus;

void i2c_scan(void){
    
    uint8_t found = 0;
    for(uint8_t addr=1; addr<127; addr++){

        esp_err_t ret = i2c_master_probe(bus, addr, 10);
        if (ret == ESP_OK)
        {
            ESP_LOGW(TAG_I2C, "Device found at 0x%02X\n", addr);
            found++;
        }   

    }

    printf("Total Number of Devices Found: %d\n", found);
    ESP_LOGI(TAG_I2C, "I2C Scan Complete");

}

void i2c_init(void)
{
    ESP_LOGI(TAG_I2C, "Initialising I2C Bus");

    const i2c_master_bus_config_t i2c_config ={
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
        .scl_io_num = I2C_SCL,
        .sda_io_num = I2C_SDA,
        //.i2c_port = I2C_PORT
    };
    
    esp_err_t ret = i2c_new_master_bus(&i2c_config, &bus);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG_I2C, "Bus Initialised Successfully");
    }
    else{
        ESP_LOGI(TAG_I2C, "Bus Initialization Unsuccessful");    
    }

}

i2c_master_bus_handle_t i2c_manager_get_bus(void)
{
    return bus;
}