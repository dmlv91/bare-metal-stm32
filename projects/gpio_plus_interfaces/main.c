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
    int degreesCel;
    int milidegreesCel;
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
 

    

    // sprintf(buffer, "SEARCHING FOR DISPLAY DRIVER PCB.....\r\n");
    // UART_Transmit((const char*)buffer, strlen((char*)buffer));

    // scanBus();

    sprintf(buffer, "###################################################################### \r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    sprintf(buffer, "START SENSOR INIT \r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));

    if (initSensor(BME680_I2C_ADDR)) {

        sprintf(buffer, "SENSOR INIT COMPLETE. PROCEED WITH TEMPERATURE READING \r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        
        while (1)
        {
            
            int temp = getTemperature();
            degreesCel = temp/100;
            milidegreesCel = temp % 100; 
            LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
            sprintf(buffer, "Current temperature:  %d.%02d%s\r\n", degreesCel, milidegreesCel,"C");
            UART_Transmit((const char*)buffer, strlen((char*)buffer));
    
            // Simple delay
            for (volatile int i = 0; i < 5000000; i++);
        }
    
        return 0;

        
    } else {
        sprintf(buffer, "Sensor initialization error. Check I2C data bus and sensor\r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
    }
    


    // Main loop
}
