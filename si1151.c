#include "si1151.h"
#include <stdio.h>
#include <unistd.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

int si1151_begin(si1151_t *dev) {
    dev->i2c_fd = wiringPiI2CSetup(SI1151_ADDR);
    dev->part_ID = si1151_read_from_register(dev, SI1151_REG_PART_ID);
    si1151_reset(dev);
    si1151_default_init(dev);
}

void si1151_reset(si1151_t *dev) {
    si1151_write_to_register(dev, SI1151_REG_MEAS_RATE0, 0);
    si1151_write_to_register(dev, SI1151_REG_MEAS_RATE1, 0);
    si1151_write_to_register(dev, SI1151_REG_IRQ_ENABLE, 0);
    si1151_write_to_register(dev, SI1151_REG_INT_CFG, 0);
    si1151_write_to_register(dev, SI1151_REG_IRQ_STATUS, 0);
    si1151_exec_command(dev, SI1151_CMD_RESET);
    sleep(10);
    si1151_write_to_register(dev, SI1151_REG_HW_KEY, 0x17);
    delay(10);
}

void si1151_default_init(si1151_t *dev) {
    si1151_write_to_register(dev, SI1151_REG_UCOEF0, 0x29);
    si1151_write_to_register(dev, SI1151_REG_UCOEF1, 0x89);
    si1151_write_to_register(dev, SI1151_REG_UCOEF2, 0x02);
    si1151_write_to_register(dev, SI1151_REG_UCOEF3, 0x00);
    si1151_write_param_data(dev, SI1151_PARAM_CHLIST, 
                                 SI1151_PARAM_CHLIST_EN_UV 
                               | SI1151_PARAM_CHLIST_EN_ALS_IR 
                               | SI1151_PARAM_CHLIST_EN_ALS_VIS
                               | SI1151_PARAM_CHLIST_EN_PS1);
    
    //set LED1 current
    //not sure if really needed
    si1151_write_param_data(dev, SI1151_PARAM_PS1_ADCMUX, SI1151_ADCMUX_LARGE_IR);
    si1151_write_to_register(dev, SI1151_REG_PS_LED21, SI1151_SET_LED_CURRENT_22MA);
    si1151_write_param_data(dev, SI1151_PARAM_PSLED12_SELECT, SI1151_PSLED12_SELECT_PS1_LED1);
    
    //adc settings
    si1151_write_param_data(dev, SI1151_PARAM_PS_ADC_GAIN, SI1151_SET_ADC_GAIN_DIV1);
    si1151_write_param_data(dev, SI1151_PARAM_PS_ADC_COUNTER, SI1151_SET_ADC_COUNTER_511ADCCLK);
    si1151_write_param_data(dev, SI1151_PARAM_PS_ADC_MISC, SI1151_SET_ADC_MISC_HIGHRANGE);

    //vis settings
    si1151_write_param_data(dev, SI1151_PARAM_ALS_VIS_ADC_GAIN, SI1151_SET_ADC_GAIN_DIV1);
    si1151_write_param_data(dev, SI1151_PARAM_ALS_VIS_ADC_COUNTER, SI1151_SET_ADC_COUNTER_511ADCCLK);
    si1151_write_param_data(dev, SI1151_PARAM_ALS_VIS_ADC_MISC, SI1151_SET_ADC_MISC_HIGHRANGE);

    //ir settings
    si1151_write_param_data(dev, SI1151_PARAM_ALS_IR_ADC_GAIN, SI1151_SET_ADC_GAIN_DIV1);
    si1151_write_param_data(dev, SI1151_PARAM_ALS_IR_ADC_COUNTER, SI1151_SET_ADC_COUNTER_511ADCCLK);
    si1151_write_param_data(dev, SI1151_PARAM_ALS_IR_ADC_MISC, SI1151_SET_ADC_MISC_HIGHRANGE);

    //enable interrupt
    si1151_write_to_register(dev, SI1151_REG_INT_CFG, 0x01);
    si1151_write_to_register(dev, SI1151_REG_IRQ_ENABLE, SI1151_SET_IRQ_ENABLE_ALS_IE);

    //auto run
    si1151_write_to_register(dev, SI1151_REG_MEAS_RATE0, 0xFF);
    si1151_exec_command(dev, SI1151_CMD_PSALS_AUTO);
}

uint8_t si1151_read_from_register(si1151_t *dev, uint8_t reg) {
    return wiringPiI2CReadReg8(dev->i2c_fd, reg);
}

int si1151_write_to_register(si1151_t *dev, uint8_t reg, uint8_t value) {
    
    if(wiringPiI2CWriteReg8(dev->i2c_fd, reg, value) != 0) {
        fprintf(stderr, "[ERROR] %s:%d: Failed to write to register\n", __FILE__, __LINE__);
        return -1;
    }
    sleep(1);
    if (si1151_read_from_register(dev, reg) != value)
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
    uint8_t low = si1151_read_from_register(dev, SI1151_REG_ALS_VIS_DATA0);
    uint8_t high = si1151_read_from_register(dev, SI1151_REG_ALS_VIS_DATA1);
    uint16_t rslt = ((uint16_t)high << 8) | low;
    dev->IR = rslt;
    
    return rslt;
}



uint16_t si1151_read_IR(si1151_t *dev) {
    uint8_t low = si1151_read_from_register(dev, SI1151_REG_ALS_IR_DATA0);
    uint8_t high = si1151_read_from_register(dev, SI1151_REG_ALS_IR_DATA1);
    uint16_t rslt = ((uint16_t)high << 8) | low;
    dev->IR = rslt;
    
    return rslt;
}



uint16_t si1151_read_proximity(si1151_t *dev, int PSn) {
    uint8_t low = si1151_read_from_register(dev, SI1151_REG_PS1_DATA0);
    uint8_t high = si1151_read_from_register(dev, SI1151_REG_PS1_DATA1);
    uint16_t rslt = ((uint16_t)high << 8) | low;
    dev->IR = rslt;
    
    return rslt;
}



uint16_t si1151_read_UV(si1151_t *dev) {
    uint8_t low = si1151_read_from_register(dev, SI1151_REG_UVINDEX0);
    uint8_t high = si1151_read_from_register(dev, SI1151_REG_UVINDEX1);
    uint16_t rslt = ((uint16_t)high << 8) | low;
    dev->IR = rslt;
    
    return rslt;
}
