

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "stm32f091xc.h"
#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_i2c.h"
#include "stm32f0xx_ll_rcc.h"

uint8_t received_data;
char buffer[100];
uint32_t    init_clk_reg = 0, 
            init_clk_conf_reg = 0, 
            init_clk_conf_reg2 = 0, 
            init_clk_conf_reg3 = 0,
            cfgr_after_configDomain = 0,
            init_i2c_timingr = 0,
            init_i2c_cr1 = 0,
            init_i2c_cr2 = 0,
            init_i2c_oar1 = 0,
            init_i2c_isr = 0,
            i2c_timingr = 0,
            i2c_cr1 = 0,
            i2c_cr2 = 0,
            i2c_oar1 = 0,
            i2c_icr = 0,
            i2c_pecr = 0,
            i2c_rxdr = 0,
            i2c_txdr = 0,
            i2c_isr = 0;
// char *message;

void SystemClock_Config(void)
{
    init_clk_reg = RCC->CR;
    init_clk_conf_reg = RCC->CFGR;
    init_clk_conf_reg2 = RCC->CFGR2;
    init_clk_conf_reg3 = RCC->CFGR3;
    LL_RCC_PLL_Disable();
    LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
    // while(LL_RCC_PLL_IsReady() != 1);

    LL_RCC_HSI_Enable();
    while(LL_RCC_HSI_IsReady() != 1)
    {
    };

    LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PREDIV_DIV_2, LL_RCC_PLL_MUL_12);

    cfgr_after_configDomain = RCC -> CFGR;

    //set PLLXTPRE bit to 1 as the ConfigDomain_SYS function apparently does not.
    RCC->CFGR |= (1 << 17);
    RCC->CFGR |= (10 << 18);

    LL_RCC_PLL_Enable();
    while(LL_RCC_PLL_IsReady() != 1);


    LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

    while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL);

    LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
    LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);


    SystemCoreClockUpdate();
    LL_InitTick(SystemCoreClock, 1000U);
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
    LL_USART_SetBaudRate(USART1,SystemCoreClock,LL_USART_OVERSAMPLING_16, 115200);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);
    LL_USART_Enable(USART1);

    while ((!(LL_USART_IsActiveFlag_TEACK(USART1))) || !(LL_USART_IsActiveFlag_REACK(USART1)));
}

void I2C_config(void) {
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
    i2c_timingr = I2C1 -> TIMINGR;
    i2c_cr1 = I2C1 -> CR1;
    i2c_cr2 = I2C1 -> CR2;
    i2c_oar1 = I2C1 -> OAR1;
    i2c_isr = I2C1 -> ISR;
    i2c_icr = I2C1 -> ICR;
    i2c_pecr = I2C1 -> PECR;
    i2c_rxdr = I2C1 -> RXDR;
    i2c_txdr = I2C1 -> TXDR;
}

void USART_Transmit(uint8_t *data, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        LL_USART_TransmitData8(USART1,data[i]);
        
    };
    while(!LL_USART_IsActiveFlag_TC(USART1));
}

unsigned char USART_Receive(void) {
    while(!LL_USART_IsActiveFlag_RXNE(USART1));
    return LL_USART_ReceiveData8(USART1);
}

void I2C_Transmit(uint8_t *data, uint8_t address) {
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRESSING_MODE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    // uint32_t ADDCODE = LL_I2C_GetAddressMatchCode(I2C1);
    // while(!LL_I2C_IsActiveFlag_ADDR(I2C1));
    // USART_Transmit(ADDCODE, 4);
    LL_I2C_TransmitData8(I2C1, *data);
    while (!LL_I2C_IsActiveFlag_STOP(I2C1));
    // received_data = LL_I2C_IsActiveFlag_STOP(I2C1);
    // USART_Transmit(&received_data,sizeof(received_data)-1);
    LL_I2C_ClearFlag_STOP(I2C1);

}

void I2C_Receive(uint8_t address) {
    LL_I2C_HandleTransfer(I2C1, address, LL_I2C_ADDRESSING_MODE_7BIT, 1, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
    while (!LL_I2C_IsActiveFlag_RXNE(I2C1));
    received_data = LL_I2C_ReceiveData8(I2C1);
    while (!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);
    }


int main(void)
{
    // uint8_t BME_680 = 0x77;
    // uint8_t typedef byte;
    SystemClock_Config();
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
    LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

    init_i2c_timingr = I2C1 -> TIMINGR;
    init_i2c_cr1 = I2C1 -> CR1;
    init_i2c_cr2 = I2C1 -> CR2;
    init_i2c_oar1 = I2C1 -> OAR1;
    init_i2c_isr = I2C1 -> ISR;
    LL_RCC_ConfigMCO(LL_RCC_MCO1SOURCE_SYSCLK, LL_RCC_MCO1_DIV_1);
    USART_config();
    I2C_config();


 

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
    

    sprintf(buffer, "Initial Clock reg: 0x%08lX \r\n", init_clk_reg);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "Initial Clock config reg: 0x%08lX \r\n", init_clk_conf_reg);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "Initial Clock config reg2: 0x%08lX \r\n", init_clk_conf_reg2);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "Initial Clock config reg3: 0x%08lX \r\n", init_clk_conf_reg3);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    USART_Transmit((uint8_t*)"-----------------------------------------------\r\n",50);

    for (volatile int i = 0; i < 100000; i++);

    sprintf(buffer, "CFGR after configDomainSys fimcton: 0x%08lX \r\n", cfgr_after_configDomain);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    USART_Transmit((uint8_t*)"-----------------------------------------------\r\n",50);

    for (volatile int i = 0; i < 100000; i++);

    uint32_t clock_reg = RCC->CR;
    uint32_t clock_conf_reg = RCC->CFGR;
    uint32_t clock_conf_reg2 = RCC->CFGR2;
    uint32_t clock_conf_reg3 = RCC->CFGR3;

    sprintf(buffer, "Clock reg: 0x%08lX \r\n", clock_reg);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "Clock config reg: 0x%08lX \r\n", clock_conf_reg);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "Clock config reg2: 0x%08lX \r\n", clock_conf_reg2);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "Clock config reg3: 0x%08lX \r\n", clock_conf_reg3);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    USART_Transmit((uint8_t*)"-----------------------------------------------\r\n",50);

    for (volatile int i = 0; i < 100000; i++);

    // SystemCoreClockUpdate();
    uint32_t sys_clk = SystemCoreClock, sys_clk_man = 0;
    uint32_t sys_clk_status = 0, sys_clk_pllmull = 0, sys_clk_pllsrc = 0, sys_clk_prediv = 0;
    
    //HSI == 0x00000000U, HSE == 0x00000004U, PLL == 0x00000008U, HSI48 = 0x0000000CU;
    sys_clk_status = LL_RCC_GetSysClkSource(); 

    //RCC_CFGR_PLLMUL -> Maska -> (0xFUL << 18U)
    sys_clk_pllmull = LL_RCC_PLL_GetMultiplicator();
    
    //RCC_CFGR_PLLSRC -> Maska -> (0x3UL << 15U)
    sys_clk_pllsrc = LL_RCC_PLL_GetMainSource();
    
    //RCC_CFGR2_PREDIV -> Maska -> (0xFUL << 0U)
    sys_clk_prediv = LL_RCC_PLL_GetPrediv() +1;
    uint32_t modified_mull = (sys_clk_pllmull >> 18) + 2;

    sys_clk_man = (HSI_VALUE/sys_clk_prediv) * modified_mull;
    
    sprintf(buffer, "SYSCLK: %lu Hz\r\n", sys_clk);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "SYSCLK SRC: %lu \r\n", sys_clk_status);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "SYSCLK PLL MUL: %lu \r\n", sys_clk_pllmull);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "SYSCLK PLL SRC: %lu \r\n", sys_clk_pllsrc);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "SYSCLK PLL PREDIV: %lu \r\n", sys_clk_prediv);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "MOD MULL: %lu \r\n", modified_mull);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "MANUAL CLK: %lu \r\n", sys_clk_man);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));
    
    USART_Transmit((uint8_t*)"-----------------------------------------------\r\n",50);

    for (volatile int i = 0; i < 100000; i++);

    sprintf(buffer, "INIT I2C_TIMINGR: 0x%08lX \r\n", init_i2c_timingr);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "INIT I2C_CR1: 0x%08lX \r\n", init_i2c_cr1);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "INIT I2C_CR2: 0x%08lX \r\n", init_i2c_cr2);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "INIT I2C_OAR1: 0x%08lX \r\n", init_i2c_oar1);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "INIT I2C_ISR: 0x%08lX \r\n", init_i2c_isr);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    USART_Transmit((uint8_t*)"-----------------------------------------------\r\n",50);

    for (volatile int i = 0; i < 100000; i++);

    sprintf(buffer, "I2C_TIMINGR: 0x%08lX \r\n", i2c_timingr);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "I2C_CR1: 0x%08lX \r\n", i2c_cr1);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "I2C_CR2: 0x%08lX \r\n", i2c_cr2);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "I2C_OAR1: 0x%08lX \r\n", i2c_oar1);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "I2C_ISR: 0x%08lX \r\n", i2c_isr);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "I2C_ICR: 0x%08lX \r\n", i2c_icr);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "I2C_PECR: 0x%08lX \r\n", i2c_pecr);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "I2C_RXDR: 0x%08lX \r\n", i2c_rxdr);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    sprintf(buffer, "I2C_TXDR: 0x%08lX \r\n", i2c_txdr);
    USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

    USART_Transmit((uint8_t*)"-----------------------------------------------\r\n",50);

    for (volatile int i = 0; i < 100000; i++);

    // unsigned char message[] = "Found active address: \n";
    // uint8_t BME_680_ID = 0xD0;
    unsigned char message[50];
    int hasAddress = 0;
    for (uint8_t address = 1; address < 128; address++) {
        if (hasAddress == 0) {
            LL_I2C_HandleTransfer(I2C1, (address << 1), LL_I2C_ADDRESSING_MODE_7BIT, 0, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
            sprintf((char*)message, "Address 0x%02X registers above!\r\n", address);
            USART_Transmit(message, strlen((char*)message));
            if (address == 77) {

                i2c_timingr = I2C1 -> TIMINGR;
                i2c_cr1 = I2C1 -> CR1;
                i2c_cr2 = I2C1 -> CR2;
                i2c_oar1 = I2C1 -> OAR1;
                i2c_isr = I2C1 -> ISR;
                i2c_icr = I2C1 -> ICR;
                i2c_pecr = I2C1 -> PECR;
                i2c_rxdr = I2C1 -> RXDR;
                i2c_txdr = I2C1 -> TXDR;
                
                sprintf(buffer, "I2C_TIMINGR: 0x%08lX \r\n", i2c_timingr);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                sprintf(buffer, "I2C_CR1: 0x%08lX \r\n", i2c_cr1);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                sprintf(buffer, "I2C_CR2: 0x%08lX \r\n", i2c_cr2);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                sprintf(buffer, "I2C_OAR1: 0x%08lX \r\n", i2c_oar1);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                sprintf(buffer, "I2C_ISR: 0x%08lX \r\n", i2c_isr);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                sprintf(buffer, "I2C_ICR: 0x%08lX \r\n", i2c_icr);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                sprintf(buffer, "I2C_PECR: 0x%08lX \r\n", i2c_pecr);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                sprintf(buffer, "I2C_RXDR: 0x%08lX \r\n", i2c_rxdr);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                sprintf(buffer, "I2C_TXDR: 0x%08lX \r\n", i2c_txdr);
                USART_Transmit((uint8_t*)buffer, strlen((char*)buffer));

                USART_Transmit((uint8_t*)"-----------------------------------------------\r\n",50);

                sprintf((char*)message, "Address 0x%02X registers above!\r\n", address);
                USART_Transmit(message, strlen((char*)message));
            }
            // Wait for the acknowledge response
            while (!LL_I2C_IsActiveFlag_STOP(I2C1)) {
                if (LL_I2C_IsActiveFlag_NACK(I2C1)) {
                    // No device at this address
                    LL_I2C_ClearFlag_NACK(I2C1);
                    break;
                } else {
                    sprintf((char*)message, "Address 0x%02X responded with ACK!\r\n", address);
                    USART_Transmit(message, strlen((char*)message));
                    for (volatile int i = 0; i < 1000000; i++);
                }
                if (LL_I2C_IsActiveFlag_ADDR(I2C1)) {
                    // Device found at this address
                    sprintf((char*)message, "Device found at 0x%02X\n", address);
                    USART_Transmit(message, strlen((char*)message));
                    LL_I2C_ClearFlag_ADDR(I2C1);
                    hasAddress = 1;
                    break;
                }
            }
            LL_I2C_ClearFlag_STOP(I2C1); // Clear STOP flag for next address
            for (volatile int i = 0; i < 100000; i++);
        }
    }
    // USART_Transmit(message, sizeof(message)-1);
    int clicks = 0;
    // I2C_Transmit(&BME_680_ID,BME_680);
    // while(!LL_I2C_IsActiveFlag_NACK(I2C1));
    // USART_Transmit(message, sizeof(message)-1);
    // I2C_Receive(BME_680);
    // USART_Transmit(&received_data,sizeof(received_data)-1);
    // USART_Transmit(message, sizeof(message)-1);

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
        // char count[3];
        // count[0] = (char)((clicks/10)+'0');
        // count[1] = (char)((clicks % 10) + '0');
        // count[2] = '\0';
        // USART_Transmit(count, sizeof(count)-1);


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
