#ifndef _BME280_WRAPPER_H
#define _BME280_WRAPPER_H

#include "bme280_defs.h"
#include "bme280.h"



/* I2C setup */
void i2c_init(void);
void i2c_scan(void);

/* BME280 lifecycle */
void bme280_i2c_init(void);
void bme280_i2c_config_and_init(void);
void bme280_settings(uint8_t mode);


typedef struct
{
    float temperature_c;
    float humidity_pct;
    float pressure_hpa;
}bme280_values_t;

void bme280_read_values(bme280_values_t *out);

/* Sensor read */
void read_bme(void);

#endif 

