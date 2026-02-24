#ifndef _i2c_h
#define _i2c_h

#include "driver/i2c_master.h"

void i2c_scan(void);
void i2c_init(void);

i2c_master_bus_handle_t i2c_manager_get_bus(void);


#endif 