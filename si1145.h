#ifndef _SI1145_H_
#define _SI1145_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct si1145_t
{
    uint8_t i2c_addr;
};

bool si1145_begin(si1145_t *dev);
void si1145_reset(si1145_t *dev);
void si1145_de_init(si1145_t *dev);
uint8_t si1145_read_from_register(si1145_t *dev, uint8_t reg);
int si1145_write_to_register(si1145_t *dev, uint8_t reg, uint8_t value);
int si1145_param_set(si1145_t *dev, int address, uint8_t value);
int si1145_ps_als_force(si1145_t *dev);
uint8_t si1145_read_param_data(si1145_t *dev, uint8_t reg);
uint8_t si1145_write_param_data(si1145_t *dev, uint8_t reg, uint8_t value);
uint16_t si1145_read_visible(si1145_t *dev);
uint16_t si1145_read_IR(si1145_t *dev);
uint16_t si1145_read_proximity(si1145_t *dev, int PSn);
uint16_t si1145_read_UV(si1145_t *dev);
int si1145_write_byte(si1145_t *dev, uint8_t reg, uint8_t value);
int si1145_read_byte(si1145_t *dev, uint8_t reg);
int si1145_read_half_word(si1145_t *dev, uint8_t reg);

#endif