#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f091xc.h"

typedef enum {
    INTERFACE_I2C_INIT,
    INTERFACE_I2C,
    INTERFACE_CLOCK_INIT,
    INTERFACE_CLOCK,
} InterfaceType;

void printDebug(const char* message, uint8_t size);

void debugRegisters(InterfaceType interface);
