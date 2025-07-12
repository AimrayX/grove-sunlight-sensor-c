#ifndef _SI1151_H_
#define _SI1151_H_

#include "si1151_defs.h"
#include <stdint.h>
#include <stdbool.h>

typedef struct {

    uint8_t part_ID;
    uint8_t i2c_addr;
    int i2c_fd;
    uint8_t cmd_ctr;
    bool initialized;
    uint8_t last_error;
    uint16_t visible_light;
    uint16_t IR;
    uint16_t proximity;
    uint16_t UV;
    
} si1151_t;

/**
 * @brief Default inits the sensor
 *
 * 
 * @param dev Device pointer to si1151 struct
 * @return Returns 0 if normal -1 for error
 */
int si1151_begin(si1151_t *dev);

/**
 * @brief Resets sensor
 *
 * 
 * @param dev Device pointer to si1151 struct
 * @return void
 */
void si1151_reset(si1151_t *dev);

/**
 * @brief Default init for sensor
 *
 * 
 * @param dev Device pointer to si1151 struct
 * @return void
 */
void si1151_default_init(si1151_t *dev);

/**
 * @brief Read from a register
 * 
 * Reads from a register via the i2c protocol
 *
 * 
 * @param dev Device pointer to si1151 struct
 * @param reg Register to read from
 * @return Returns read value
 */
uint8_t si1151_read_from_register(si1151_t *dev, uint8_t reg);
int si1151_write_to_register(si1151_t *dev, uint8_t reg, uint8_t value);

/**
 * @brief Executes a command
 *
 * This function takes in a command and executes it via the command protocol.
 * The function retries a couple of times if it notices that the command 
 * hasn't been written to the command register or the response register is still empty 
 * and thus the command didn't execute
 * 
 * @param dev Device pointer to si1151 struct
 * @return Returns 0 if normal -1 for error
 */
int si1151_exec_command(si1151_t *dev, uint8_t value);

/**
 * @brief Forces a single PS and ALS measurement
 *
 * 
 * @param dev Device pointer to si1151 struct
 * @return Returns 0 if normal -1 for error
 */
int si1151_force_measurement(si1151_t *dev);

/**
 * @brief Read from a internal param register
 * 
 * Reads from a register via the i2c protocol 
 * by first executing the query command with the given param
 * and then reading the PARAM_RD register 
 * 
 * @param dev Device pointer to si1151 struct
 * @param param Param to read
 * @return Returns read value
 */
uint8_t si1151_read_param_data(si1151_t *dev, uint8_t param);


uint8_t si1151_write_param_data(si1151_t *dev, uint8_t param, uint8_t value);


uint16_t si1151_read_visible(si1151_t *dev);


uint16_t si1151_read_IR(si1151_t *dev);


uint16_t si1151_read_proximity(si1151_t *dev, int PSn);


uint16_t si1151_read_UV(si1151_t *dev);

#endif