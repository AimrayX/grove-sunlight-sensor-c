#ifndef _SI1145_H_
#define _SI1145_H_

#include "si1145_defs.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {

    uint8_t part_ID;
    uint8_t i2c_addr;
    int i2c_fd;
    bool initialized;
    uint8_t last_error;
    
} si1145_t;

bool si1145_begin(si1145_t *dev);
void si1145_reset(si1145_t *dev);
void si1145_default_init(si1145_t *dev);
uint8_t si1145_read_from_register(si1145_t *dev, uint8_t reg);
int si1145_write_to_register(si1145_t *dev, uint8_t reg, uint8_t value);
int si1145_exec_command(si1145_t *dev, uint8_t value);
int si1145_ps_als_force(si1145_t *dev);
uint8_t si1145_read_param_data(si1145_t *dev, uint8_t param);
uint8_t si1145_write_param_data(si1145_t *dev, uint8_t param, uint8_t value);
uint16_t si1145_read_visible(si1145_t *dev);
uint16_t si1145_read_IR(si1145_t *dev);
uint16_t si1145_read_proximity(si1145_t *dev, int PSn);
uint16_t si1145_read_UV(si1145_t *dev);

#endif