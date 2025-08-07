#include "i2c.h"
char buff[100];


void I2C_Config(void) {
    LL_I2C_Disable(I2C1);
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_SYSCLK);
    LL_I2C_SetMasterAddressingMode(I2C1, LL_I2C_ADDRESSING_MODE_7BIT);
    LL_I2C_SetMode(I2C1,LL_I2C_MODE_I2C);
    LL_I2C_SetOwnAddress1(I2C1, 0x3a, LL_I2C_OWNADDRESS1_7BIT);
    //Set up the I2C_TIMINGR value based on the reference manual 
    uint32_t timing_val = 0x13 | (0xF << 8) | (0x2 << 16) | (0x4 << 20) | (0xB << 28);
    LL_I2C_SetTiming(I2C1, timing_val);
    LL_I2C_EnableOwnAddress1(I2C1);
    LL_I2C_Enable(I2C1);

    while (!LL_I2C_IsEnabled(I2C1));

    // debugRegisters(INTERFACE_I2C);
   
}

void I2C_WriteRegister(uint8_t sensorAddress, uint8_t regAddress, uint8_t data) {
    // sprintf(buff, "Writing data (0x%02x)to register: 0x%02X \r\n", data, regAddress);
    // UART_Transmit((const char *)buff, strlen((char*)buff));
    LL_I2C_HandleTransfer(I2C1, sensorAddress, LL_I2C_ADDRESSING_MODE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    LL_I2C_TransmitData8(I2C1, regAddress);
    while (!LL_I2C_IsActiveFlag_TXE(I2C1));
   
    LL_I2C_TransmitData8(I2C1, data);
    while (!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);
}

uint8_t I2C_ReadRegister(uint8_t sensorAddress, uint8_t regAddress) {
    uint8_t data = 0;
    // sprintf(buff, "Reading data from register: 0x%02X \r\n", regAddress);
    // UART_Transmit((const char*)buff, strlen((char*)buff));
    LL_I2C_HandleTransfer(I2C1, sensorAddress << 1, LL_I2C_ADDRESSING_MODE_7BIT, 1, LL_I2C_MODE_SOFTEND, LL_I2C_GENERATE_START_WRITE);
    LL_I2C_TransmitData8(I2C1, regAddress);
    while (!LL_I2C_IsActiveFlag_TXE(I2C1));
    while (!LL_I2C_IsActiveFlag_TC(I2C1));

    LL_I2C_HandleTransfer(I2C1, sensorAddress << 1, LL_I2C_ADDRESSING_MODE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1));
    data = LL_I2C_ReceiveData8(I2C1);

    while (!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);

    return data;
}

void scanBus(void) {
    int hasAddress = 0;
    sprintf((char*)buff, "START I2C BUS SCAN!\r\n");
    UART_Transmit(buff, strlen((char*)buff));
    for (uint8_t address = 0; address < 128; address++) {
        // sprintf((char*)buff, "Address 0x%02X\r\n", address);
        // UART_Transmit(buff, strlen((char*)buff));
        if (hasAddress == 0) {
            LL_I2C_HandleTransfer(I2C1, (address << 1), LL_I2C_ADDRESSING_MODE_7BIT, 0, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
            if (address == 0x77) {

                UART_Transmit((const char*)"-----------------------------------------------\r\n",50);
                
                // debugRegisters(INTERFACE_I2C);
            }
            // Wait for the acknowledge response
            while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
                if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
                    // No device at this address
                    LL_I2C_ClearFlag_NACK(I2C1);
                    break;
                }
                if (LL_I2C_IsActiveFlag_ADDR(I2C1)) {
                    // Device found at this address
                    sprintf((char*)buff, "Device found at 0x%02X\n", address);
                    UART_Transmit((const char *)buff, strlen((char*)buff));
                    LL_I2C_ClearFlag_ADDR(I2C1);
                    hasAddress = 1;
                    break;
                }
            }
            LL_I2C_ClearFlag_STOP(I2C1); // Clear STOP flag for next address
            for (volatile int i = 0; i < 100000; i++);
        }
    }
    sprintf((char*)buff, "SCAN COMPLETED!\r\n");
    UART_Transmit(buff, strlen((char*)buff));
}

void initI2C(void) {
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    I2C_Config();
}
