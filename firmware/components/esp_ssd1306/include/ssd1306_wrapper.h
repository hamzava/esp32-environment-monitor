#ifndef _ssd1306_wrapper_h
#define _ssd1306_wrapper_h

#include "ssd1306.h"
#include "driver/i2c_master.h"

void ssd1306_cfg_and_init(void);

void display_x3_text(void);

void clear_display(void);

void test_display(void);

void display_weather_data(float temp, float hum, float pres);

void display_ltr390_data(float ambient_light, float als);

void draw_rectangle(void);


#endif