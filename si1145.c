#include "si1145.h"
#include <stdio.h>
#include <time.h>
#include <wiringPi.h>
#include <wiringPiI2C.h>

bool si1145_begin(si1145_t *dev) {
    dev->i2c_fd = wiringPiI2CSetup(SI1145_ADDR);
    dev->part_ID = si1145_read_from_register(dev, SI1145_REG_PART_ID);
    si1145_write_to_register(dev, SI1145_REG_HW_KEY, 0x17);
}



void si1145_reset(si1145_t *dev) {


}



void si1145_default_init(si1145_t *dev) {
    si1145_write_to_register(dev, SI1145_REG_UCOEF0, 0x29);
    si1145_write_to_register(dev, SI1145_REG_UCOEF1, 0x89);
    si1145_write_to_register(dev, SI1145_REG_UCOEF2, 0x02);
    si1145_write_to_register(dev, SI1145_REG_UCOEF3, 0x00);
    si1145_write_param_data(dev, SI1145_PARAM_CHLIST, SI1145_PARAM_CHLIST_ENUV 
                               | SI1145_PARAM_CHLIST_ENALSIR 
                               | SI1145_PARAM_CHLIST_ENALSVIS
                               | SI1145_PARAM_CHLIST_ENPS1);
    
    si1145_write_param_data(dev, SI1145_PARAM_PS1_ADCMUX, SI1145_ADCMUX_LARGE_IR);
    si1145_write_to_register(dev, SI1145_REG_PS_LED21, SI1145_SET_LED_CURRENT_22MA);
    
    si1145_write_param_data(dev, SI1145_PARAM_PS_ADC_GAIN, SI1145_SET_ADC_GAIN_DIV1);
    si1145_write_param_data(dev, SI1145_PARAM_PS_ADC_COUNTER, SI1145_SET_ADC_COUNTER_511ADCCLK);
    si1145_write_param_data(dev, SI1145_PARAM_PS_ADC_MISC, SI1145_SET_ADC_MISC_HIGHRANGE);
}



uint8_t si1145_read_from_register(si1145_t *dev, uint8_t reg) {

    return wiringPiI2CReadReg8(dev->i2c_fd, reg);
}



int si1145_write_to_register(si1145_t *dev, uint8_t reg, uint8_t value) {
    
    if(wiringPiI2CWriteReg8(dev->i2c_fd, reg, value) != 0) {
        fprintf(stderr, "Error: Failed to write to register");
        return -1;
    }
    return 0;
}



int si1145_exec_command(si1145_t *dev, uint8_t value) {
    int i = 0;
    int j = 0;
    struct timespec ts1 = {0, 25000000}; // 0 seconds, 25'000'000 nanoseconds (25ms)
    struct timespec ts2 = {0, 1000000};  // 0 seconds, 1'000'000 nanoseconds (1ms)
    while (j < 2)
    {
        
        while (i < 2)
        {
            //1. Write 0x00 to Command register to clear the Response register
            wiringPiI2CWriteReg8(dev->i2c_fd, SI1145_REG_COMMAND, SI1145_CMD_NOP);

            //2. Read Response register and verify contents are 0x00
            if(si1145_read_from_register(dev, SI1145_REG_RESPONSE) != 0x00) {
                fprintf(stderr, "Warning: Response register not 0x00 after 1. write to command register\n");
                i++;
            }
        }

        if (i > 1)
        {
            fprintf(stderr, "Error: Failed to write to command register\n");
            return -1;
        }

        i = 0;
        while (i < 2)
        {
            //3. Write Command value from Table 5 into Command register
            wiringPiI2CWriteReg8(dev->i2c_fd, SI1145_REG_COMMAND, value);

            //Step 4 is not applicable to the Reset Command because the device
            //will reset itself and does not increment the
            //Response register after reset. 
            //No Commands should be issued to the device for at least 1 ms after 
            //a Reset is issued.
            if (value == SI1145_CMD_RESET)
            {
                nanosleep(&ts2, NULL);
                break;
            }
            
            //4. Read the Response register and verify contents are now non-zero. 
            //If contents are still 0x00, repeat this step.
            //If the Response register remains 0x00 for over 25 ms after the Command write, 
            //the entire Command process should be repeated starting with Step 1.
            nanosleep(&ts1, NULL);
            if(si1145_read_from_register(dev, SI1145_REG_RESPONSE) == 0x00) {
                fprintf(stderr, "Warning: Failed to write command to command register\n");
            }
        }
        
        if (i > 1)
        {
            fprintf(stderr, "Error: Failed to write to command register\n");
            if (j > 1)
            {
                return -1;
            }
            fprintf(stderr, "Warning: Repeating command process\n");
            j++;
        } else {
            break;
        }
    }

    return 0;
}



int si1145_ps_als_force(si1145_t *dev) {


}



uint8_t si1145_read_param_data(si1145_t *dev, uint8_t param) {
    si1145_exec_command(dev, SI1145_CMD_PARAM_QUERY | param);
    return si1145_read_from_register(dev, SI1145_REG_PARAM_RD);
}



uint8_t si1145_write_param_data(si1145_t *dev, uint8_t param, uint8_t value) {
    si1145_write_to_register(dev, SI1145_REG_PARAM_WR, value);
    si1145_exec_command(dev, SI1145_CMD_PARAM_SET | param);
    return si1145_read_from_register(dev, SI1145_REG_PARAM_RD);
}



uint16_t si1145_read_visible(si1145_t *dev) {


}



uint16_t si1145_read_IR(si1145_t *dev) {


}



uint16_t si1145_read_proximity(si1145_t *dev, int PSn) {


}



uint16_t si1145_read_UV(si1145_t *dev) {


}
