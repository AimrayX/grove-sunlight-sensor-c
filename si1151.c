#include "si1151.h"
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

int si1151_begin(si1151_t *dev, bool mode) {
    dev->is_autonomous = mode;
    dev->i2c_fd = wiringPiI2CSetup(SI1151_ADDR);
    dev->part_ID = si1151_read_from_register(dev, SI1151_REG_PART_ID);
    si1151_reset(dev);
    si1151_default_init(dev);
    fprintf(stderr, "[INFO] %s:%d: Sensor has been set to default settings\n", __FILE__, __LINE__);
    si1151_exec_command(dev, SI1151_CMD_START);
    fprintf(stderr, "[INFO] %s:%d: Starting measurements\n", __FILE__, __LINE__);
    return 0;
}

void si1151_reset(si1151_t *dev) {
    si1151_exec_command(dev, SI1151_CMD_RESET_SW);
}

void si1151_default_init(si1151_t *dev) {
    si1151_write_param_data(dev, SI1151_PARAM_CHAN_LIST, SI1151_SET_CHAN_LIST_CHAN0_EN 
                                                       | SI1151_SET_CHAN_LIST_CHAN1_EN);
    
    si1151_write_to_register(dev, SI1151_REG_IRQ_ENABLE, SI1151_SET_IRQ_ENABLE_IE0
                                                       | SI1151_SET_IRQ_ENABLE_IE1);

    //set LED1 current
    //not sure if really needed
    si1151_write_param_data(dev, SI1151_PARAM_MEAS_RATE_H,  0);
    si1151_write_param_data(dev, SI1151_PARAM_MEAS_RATE_L,  1);
    si1151_write_param_data(dev, SI1151_PARAM_MEAS_COUNT0,  1);
    si1151_write_param_data(dev, SI1151_PARAM_MEAS_COUNT1,  1);
    si1151_write_param_data(dev, SI1151_PARAM_THRESHOLD0_L, 0);
    si1151_write_param_data(dev, SI1151_PARAM_THRESHOLD0_H, 0);

    si1151_write_param_data(dev, SI1151_PARAM_ADC_CONFIG0, SI1151_SET_ADC_CONF_MUX_SMALL_IR 
                                                         | SI1151_SET_ADC_CONF_DECIM_RATE_512);

    si1151_write_param_data(dev, SI1151_PARAM_ADC_CONFIG1, SI1151_SET_ADC_CONF_MUX_VISIBLE 
                                                         | SI1151_SET_ADC_CONF_DECIM_RATE_512);

    si1151_write_param_data(dev, SI1151_PARAM_ADC_SENS0, SI1151_SET_ADC_SENS_HW_GAIN1);
    si1151_write_param_data(dev, SI1151_PARAM_ADC_SENS1, SI1151_SET_ADC_SENS_HW_GAIN1);

    si1151_write_param_data(dev, SI1151_PARAM_ADC_POST0, SI1151_SET_ADC_POST_THRESH_EN1);
    si1151_write_param_data(dev, SI1151_PARAM_ADC_POST1, SI1151_SET_ADC_POST_THRESH_EN1);

    si1151_write_param_data(dev, SI1151_PARAM_MEAS_CONFIG0, SI1151_SET_MEAS_CONF_LED1_EN 
                                                          | SI1151_SET_MEAS_CONF_BANK_SEL_LEDX_A 
                                                          | SI1151_SET_MEAS_CONF_COUNTER_INDEX0);

    si1151_write_param_data(dev, SI1151_PARAM_MEAS_CONFIG1, SI1151_SET_MEAS_CONF_LED2_EN 
                                                          | SI1151_SET_MEAS_CONF_BANK_SEL_LEDX_B
                                                          | SI1151_SET_MEAS_CONF_COUNTER_INDEX1);
    
}

uint8_t si1151_read_from_register(si1151_t *dev, uint8_t reg) {
    return wiringPiI2CReadReg8(dev->i2c_fd, reg);
}

int si1151_write_to_register(si1151_t *dev, uint8_t reg, uint8_t value) {
    
    //This is necessary because this register behaves like a param setting
    if (reg == SI1151_REG_IRQ_ENABLE)
    {
        value = value ^ si1151_read_from_register(dev, reg);
    }

    if(wiringPiI2CWriteReg8(dev->i2c_fd, reg, value) != 0) {
        fprintf(stderr, "[ERROR] %s:%d: Failed to write to register\n", __FILE__, __LINE__);
        return -1;
    }
    sleep(1);
    uint8_t rs = si1151_read_from_register(dev, reg);
    if (reg != SI1151_REG_COMMAND && si1151_read_from_register(dev, reg) != value)
    {
        fprintf(stderr, "[ERROR] %s:%d: Wrote to register, but check returned wrong value\n", __FILE__, __LINE__);
        return -1;
    }
    return 0;
}

int si1151_exec_command(si1151_t *dev, uint8_t value) {
    int i = 0;
    while (i < 2)
    {
        if(si1151_write_to_register(dev, SI1151_REG_COMMAND, SI1151_CMD_RESET_CMD_CTR)) {
            i++;
        } else {
            break;
        }
    }

    if (i > 1)
    {
        fprintf(stderr, "[ERROR] %s:%d: Failed to reset cmd counter\n", __FILE__, __LINE__);
        return -1;
    }
    
    dev->cmd_ctr = si1151_read_from_register(dev, SI1151_REG_RESPONSE0);

    i = 0;
    while (i < 2)
    {
        if(si1151_write_to_register(dev, SI1151_REG_COMMAND, value)) {
            i++;
        } else {
            break;
        }
    }

    if (i > 1)
    {
        fprintf(stderr, "[ERROR] %s:%d: Failed to write command to cmd register\n", __FILE__, __LINE__);
        return -1;
    }

    //wait until reset command has finished executing
    if (value == SI1151_CMD_RESET_SW)
    {
        while (((si1151_read_from_register(dev, SI1151_REG_RESPONSE0) | 0b11110000) ^ 0b11110000) != 0x0F)
        {
            fprintf(stderr, "[WARNING] %s:%d: Reset hasn't finished, waiting...\n", __FILE__, __LINE__);
            sleep(5);
        }
        fprintf(stderr, "[INFO] %s:%d: Sensor has been reset\n", __FILE__, __LINE__);
        return 0;
    }

    if(si1151_read_from_register(dev, SI1151_REG_RESPONSE0) == (dev->cmd_ctr)) {
        fprintf(stderr, "[ERROR] %s:%d: Wrote to register without the cmd counter changing\n", __FILE__, __LINE__);
        return -1;
    }

    return 0;
}

int si1151_force_measurement(si1151_t *dev) {
    return si1151_exec_command(dev, SI1151_CMD_FORCE);
}

uint8_t si1151_read_param_data(si1151_t *dev, uint8_t param) {
    si1151_exec_command(dev, SI1151_CMD_PARAM_QUERY | param);
    return si1151_read_from_register(dev, SI1151_REG_RESPONSE1);
}

uint8_t si1151_write_param_data(si1151_t *dev, uint8_t param, uint8_t value) {
    si1151_write_to_register(dev, SI1151_REG_HOST_IN0, value);
    si1151_exec_command(dev, SI1151_CMD_PARAM_SET | param);
    
    return si1151_read_from_register(dev, SI1151_REG_RESPONSE1);
}

uint16_t si1151_read_visible(si1151_t *dev) {
    if(!dev->is_autonomous) si1151_exec_command(dev, si1151_force_measurement);

    uint8_t low = si1151_read_from_register(dev, SI1151_REG_HOST_OUT0);
    uint8_t high = si1151_read_from_register(dev, SI1151_REG_HOST_OUT1);
    uint16_t rslt = ((uint16_t)high << 8) | low;
    dev->IR = rslt;
    
    return rslt;
}

uint16_t si1151_read_IR(si1151_t *dev) {
    uint8_t low = si1151_read_from_register(dev, SI1151_REG_HOST_OUT2);
    uint8_t high = si1151_read_from_register(dev, SI1151_REG_HOST_OUT3);
    uint16_t rslt = ((uint16_t)high << 8) | low;
    dev->IR = rslt;
    
    return rslt;
}