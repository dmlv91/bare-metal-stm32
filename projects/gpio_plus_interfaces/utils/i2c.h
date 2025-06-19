#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_bus.h"

void I2C_Config(void);

void I2C_WriteRegister(uint8_t sensorAddress, uint8_t regAddress, uint8_t data);

uint8_t I2C_ReadRegister(uint8_t sensorAddress, uint8_t regAddress);

void scanBus(void);

void initI2C(void);