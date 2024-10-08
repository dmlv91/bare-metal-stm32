

#include <inttypes.h>
#include <stdbool.h>
#include "stm32f0xx.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_rcc.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_i2c.h"



void SystemClock_Config(void)
{
    
    LL_RCC_PLL_Disable();
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);

    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PREDIV_DIV_2, LL_RCC_PLL_MUL_12);

    LL_RCC_PLL_Enable();
    while(LL_RCC_PLL_IsReady() != 1)
    {
    };

    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);

    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL)
    {
    };

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);

    //clock for UART
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_HSI);

    //clock for I2C
    LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_HSI);

    LL_InitTick(48000000U, 1000U);
}

void SystemInit(void)
{
  
    SystemClock_Config();
}

void _init(void) {}

void setGPIO(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode, uint32_t Type, uint32_t Speed, uint32_t Pull, uint32_t AltFunc) {
    LL_GPIO_SetPinMode(GPIOx, Pin, Mode);
    if (Mode != LL_GPIO_MODE_INPUT) {
        LL_GPIO_SetPinOutputType(GPIOx, Pin, Type);
    }
    if (Mode == LL_GPIO_MODE_ALTERNATE) {
        LL_GPIO_SetAFPin_8_15(GPIOx,Pin,AltFunc);
    }
    LL_GPIO_SetPinSpeed(GPIOx, Pin, Speed);
    LL_GPIO_SetPinPull(GPIOx, Pin, Pull);
}

void USART_config(void) {
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate(USART1,8000000U,LL_USART_OVERSAMPLING_16, 115200);

    LL_USART_Enable(USART1);

    while ((!(LL_USART_IsActiveFlag_TEACK(USART1))) || !(LL_USART_IsActiveFlag_REACK(USART1)));
}

void I2C_config(void) {
    LL_I2C_SetMasterAddressingMode(I2C1, LL_I2C_ADDRESSING_MODE_7BIT);
    LL_I2C_SetMode(I2C1,LL_I2C_MODE_I2C);
    LL_I2C_SetOwnAddress1(I2C1, 0x3a, LL_I2C_OWNADDRESS1_7BIT);

    LL_I2C_EnableOwnAddress1(I2C1);
    LL_I2C_Enable(I2C1);
    while (!LL_I2C_IsEnabled(I2C1));
}

void USART_Transmit(uint8_t *data, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        LL_USART_TransmitData8(USART1,data[i]);
        
    }
    while(!LL_USART_IsActiveFlag_TC(USART1));
}

unsigned char USART_Receive(void) {
    while(!LL_USART_IsActiveFlag_RXNE(USART1));
    return LL_USART_ReceiveData8(USART1);
}

// void bitbang_UART_Transmit(uint8_t*data, uint32_t size) {

// }

int main(void)
{

    SystemInit();
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);
    USART_config();


    // Configure outputs
    setGPIO(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_SPEED_FREQ_LOW, LL_GPIO_PULL_NO,0);
    
    //UART
    setGPIO(GPIOA, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_PUSHPULL, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_PULL_NO, LL_GPIO_AF_1);
    setGPIO(GPIOA, LL_GPIO_PIN_10, LL_GPIO_MODE_ALTERNATE, 0,LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_PULL_NO,LL_GPIO_AF_1);
    
    //I2C
    setGPIO(GPIOB, LL_GPIO_PIN_8,LL_GPIO_MODE_ALTERNATE, LL_GPIO_OUTPUT_OPENDRAIN, LL_GPIO_SPEED_FREQ_HIGH,LL_GPIO_PULL_NO, LL_GPIO_AF_1); 
    setGPIO(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_ALTERNATE, 0, LL_GPIO_SPEED_FREQ_HIGH, LL_GPIO_PULL_NO, LL_GPIO_AF_1);
    
    
    //Configure inputs
    setGPIO(GPIOC, LL_GPIO_PIN_13, LL_GPIO_MODE_INPUT,0,LL_GPIO_SPEED_FREQ_LOW,LL_GPIO_PULL_NO,0);


    // uint8_t message[] = "Hello, Test!\n";
    // USART_Transmit(message, sizeof(message)-1);
    int clicks = 0;

    // Main loop
    while (1)
    {
        // uint8_t recv = USART_Receive();
        // USART_Transmit(&recv,1);
        int delay;
        if (!LL_GPIO_IsInputPinSet(GPIOC,LL_GPIO_PIN_13)) {
            clicks += 1;
            // LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);

            // delay = 100000;

        } else {
            // USART_Transmit(notClick, sizeof(notClick)-1);
            // LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);
            // delay = 500000;
        }
        char count[3];
        count[0] = (char)((clicks/10)+'0');
        count[1] = (char)((clicks % 10) + '0');
        count[2] = '\0';
        USART_Transmit(count, sizeof(count)-1);


        if (clicks > 0 && clicks < 5) {
            delay = 1000000;
        } else if (clicks >= 5 && clicks < 10) {
            delay = 500000;
        } else {
            delay = 100000;
        }

        if (clicks == 15) {
            clicks = 0;
        }
        
        LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5);

        // Simple delay
        for (volatile int i = 0; i < delay; i++);
    }

    return 0;
}
