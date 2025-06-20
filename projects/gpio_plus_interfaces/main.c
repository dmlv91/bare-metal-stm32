#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "utils/clock.h"
#include "utils/gpio.h"
#include "utils/i2c.h"
#include "utils/uart.h"
#include "utils/debug.h"
#include "sensor/BME680.h"

#define BME680_I2C_ADDR 0x77

uint8_t received_data;
char buffer[100];

void _init(void) {}

int main(void)
{
    
    // uint8_t typedef byte;
    initClock();
    initGPIO();
    initUART();
    initI2C();
    
    // init_i2c_timingr = I2C1 -> TIMINGR;
    // init_i2c_cr1 = I2C1 -> CR1;
    // init_i2c_cr2 = I2C1 -> CR2;
    // init_i2c_oar1 = I2C1 -> OAR1;
    // init_i2c_isr = I2C1 -> ISR;
    
    // debugRegisters(INTERFACE_CLOCK);
 

    // Configure outputs
    setGPIO(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_PULL_NO,0);
    
    //UART
    setGPIO(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_PULL_NO, LL_GPIO_AF_1);
    setGPIO(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE, 0,LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_PULL_NO,LL_GPIO_AF_1);
    
    //I2C
    setGPIO(GPIOB, LL_GPIO_PIN_9,LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_OPENDRAIN, LL_GPIO_SPEED_FREQ_HIGH,LL_GPIO_PULL_NO, LL_GPIO_AF_1); 
    setGPIO(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE, 0, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_PULL_NO, LL_GPIO_AF_1);
    
    //MCO clock debug pins
    setGPIO(GPIOA, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_PULL_NO, LL_GPIO_AF_0);
    
    //Configure inputs
    setGPIO(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT,0,LL_GPIO_SPEED_FREQ_LOW,LL_GPIO_PULL_NO,0);
    
    // scanBus();

    sprintf(buffer, "START SENSOR INIT \r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));

    if (initSensor(BME680_I2C_ADDR)) {

        sprintf(buffer, "SENSOR INIT COMPLETE. PROCEED WITH TEMPERATURE READING \r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        int currentTemp = getTemperature();

        sprintf(buffer, "Real temperature:  %d\r\n", currentTemp);
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        
    } else {
        sprintf(buffer, "Sensor initialization error. Check I2C data bus and sensor\r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
    }
    


    // Main loop
    while (1)
    {
        
        LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);

        // Simple delay
        for (volatile int i = 0; i < 1000000; i++);
    }

    return 0;
}
