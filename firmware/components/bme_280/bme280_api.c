#include <stdio.h>
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_rom_sys.h"   // needed for ets_delay_us()
#include "i2c.h"

#include "bme280.h"
#include "bme280_defs.h"
#include "bme280_wrapper.h"

#define I2C_SDA     21
#define I2C_SCL     22
#define I2C_FREQ_HZ 100000
#define BME280_ADDR 0x76

static const char *TAG_I2C = "i2c_bus";

static i2c_master_dev_handle_t dev;

struct bme280_dev bme;
struct bme280_data data;
struct bme280_uncomp_data uncompdata;
struct bme280_calib_data calibdata;

const char *TAG_BME280 = "bme280";

static void delay_us(uint32_t period, void *intf_ptr)
{
    esp_rom_delay_us(period);
}

void bme280_i2c_init(void){

i2c_master_bus_handle_t bus = i2c_manager_get_bus();

 // Configure I2C Device
    i2c_device_config_t bme280_cfg ={
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address =BME280_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
        .scl_wait_us = 100,
    };
    
    // Check if BME280 exists
    for(uint8_t i=0; i<10; i++){

        esp_err_t ret = i2c_master_probe(bus, BME280_ADDR, 100);
        ESP_LOGI(TAG_I2C, "Connecting to BME280... Try #:%d\n", i );
        
        if (ret == ESP_OK)
        {   
            i2c_master_bus_add_device(bus, &bme280_cfg, &dev);
            ESP_LOGI(TAG_I2C, "BME280 added successfully.");
            break;
        }
        else{
             ESP_LOGI(TAG_I2C, "BME280 I2C Init Unsuccessful");
        }
        
    }
    
}

static int8_t bme280_i2c_read(uint8_t reg, uint8_t *buf, uint32_t len, void *intf_ptr)
{
    i2c_master_dev_handle_t dev = (i2c_master_dev_handle_t )intf_ptr;

    esp_err_t ret = i2c_master_transmit_receive(dev, &reg, 1,buf, len,1000);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG_I2C, "I2C read failed: %s", esp_err_to_name(ret));
        return BME280_E_COMM_FAIL;
    }
    else{
        ESP_LOGE(TAG_I2C, "I2C read complete");
        return BME280_OK;
    }

    
}

static int8_t bme280_i2c_write(uint8_t reg, const uint8_t *buf, uint32_t len, void *intf_ptr)
{

    i2c_master_dev_handle_t dev =(i2c_master_dev_handle_t)intf_ptr;

    uint8_t tmp[len + 1];
    tmp[0] = reg;
    memcpy(&tmp[1], buf, len);

    esp_err_t ret = i2c_master_transmit(dev, tmp, len + 1, 1000);

    if (ret != ESP_OK) {
        ESP_LOGE(TAG_I2C, "I2C write failed: %s", esp_err_to_name(ret));
        return BME280_E_COMM_FAIL;
    }
    else{
        ESP_LOGE(TAG_I2C, "I2C write complete");
        return BME280_OK;
    }
}

void bme280_i2c_config_and_init(void){

    //Initialize Bosch driver struct
    bme.read = bme280_i2c_read;
    bme.write = bme280_i2c_write;
    bme.intf_ptr = dev;
    bme.intf = BME280_I2C_INTF;
    bme.delay_us = delay_us;
    
    ESP_LOGI(TAG_I2C, "Bosch Driver Struct Initialized");

    if (bme280_init(&bme) != BME280_OK)
    {
        ESP_LOGE(TAG_BME280, "BME280 init failed");
    }
    else{
        ESP_LOGE(TAG_BME280, "BME280 Sensor Initialized");
    }

}

void bme280_settings(uint8_t mode){

    struct bme280_settings settings = {
    .osr_h = BME280_OVERSAMPLING_1X,
    .osr_p = BME280_OVERSAMPLING_16X,
    .osr_t = BME280_OVERSAMPLING_2X,
    .filter = BME280_FILTER_COEFF_OFF,
    .standby_time = BME280_STANDBY_TIME_0_5_MS
    };

    uint8_t sel = BME280_SEL_ALL_SETTINGS;


    bme280_set_sensor_settings(sel, &settings, &bme);
    
    bme280_set_sensor_mode(mode, &bme);
    
    if( bme280_set_sensor_settings(sel, &settings, &bme)<0 || bme280_set_sensor_mode(mode, &bme)<0 ){
        ESP_LOGI(TAG_I2C, "Sensor Setting Failed");
    }
    else{
        ESP_LOGI(TAG_I2C, "Sensor Settings Set");
    }

    ESP_LOGI(TAG_BME280, "BME280 configured and running.");

}

void read_bme(void)
{
    if (bme280_get_sensor_data(BME280_ALL, &data, &bme) != BME280_OK)
    {
        ESP_LOGE(TAG_BME280, "Sensor read failed!");
        return;
    }

    ESP_LOGI(TAG_BME280, "Temperature: %.2f C", data.temperature);
    ESP_LOGI(TAG_BME280, "Humidity:    %.2f %%", data.humidity);
    ESP_LOGI(TAG_BME280, "Pressure:    %.2f hPa", data.pressure / 100.0);
}

void bme280_read_values(bme280_values_t *out){

    bme280_get_sensor_data(BME280_ALL, &data, &bme);
    
    vTaskDelay(pdMS_TO_TICKS(10));

    out->temperature_c = data.temperature;
    out->humidity_pct  = data.humidity;
    out->pressure_hpa  = data.pressure / 100.0f;

}