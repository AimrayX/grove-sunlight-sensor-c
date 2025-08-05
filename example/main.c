#include "si1151.h"
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>

int main() {
    int fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[ERROR] %s:%d: Failed to open i2c\n", __FILE__, __LINE__);
        return 1;
    }

    si1151_t si1151;
    si1151_begin(&si1151, 1, fd);
    int result = 0;
    while (true) {    
        sleep(3);
        result = si1151_read_visible(&si1151);
        printf("The result is: %d\n", result);
    } 
    return 0;
}