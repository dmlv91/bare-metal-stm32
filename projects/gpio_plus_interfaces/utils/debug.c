#include "debug.h"
#include "uart.h"

char buffr[100];
uint32_t    init_i2c_timingr = 0,
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

void printDebug(const char* message, uint8_t size) {

}

void debugRegisters(InterfaceType interface) {

    UART_Transmit((const char*)"################################################### \r\n",50);
    sprintf(buffr, "Getting register states for interface: %d  \r\n", interface);
    UART_Transmit((const char*)buffr, strlen((char*)buffr));

    switch (interface) {
        case INTERFACE_I2C:

            i2c_timingr = I2C1 -> TIMINGR;
            i2c_cr1 = I2C1 -> CR1;
            i2c_cr2 = I2C1 -> CR2;
            i2c_oar1 = I2C1 -> OAR1;
            i2c_isr = I2C1 -> ISR;
            i2c_icr = I2C1 -> ICR;
            i2c_pecr = I2C1 -> PECR;
            i2c_rxdr = I2C1 -> RXDR;
            i2c_txdr = I2C1 -> TXDR;

            sprintf(buffr, "I2C_TIMINGR: 0x%08lX \r\n", i2c_timingr);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "I2C_CR1: 0x%08lX \r\n", i2c_cr1);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "I2C_CR2: 0x%08lX \r\n", i2c_cr2);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "I2C_OAR1: 0x%08lX \r\n", i2c_oar1);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "I2C_ISR: 0x%08lX \r\n", i2c_isr);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "I2C_ICR: 0x%08lX \r\n", i2c_icr);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "I2C_PECR: 0x%08lX \r\n", i2c_pecr);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "I2C_RXDR: 0x%08lX \r\n", i2c_rxdr);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "I2C_TXDR: 0x%08lX \r\n", i2c_txdr);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            UART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            for (volatile int i = 0; i < 100000; i++);
            break;

        case INTERFACE_I2C_INIT:
            sprintf(buffr, "INIT I2C_TIMINGR: 0x%08lX \r\n", init_i2c_timingr);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "INIT I2C_CR1: 0x%08lX \r\n", init_i2c_cr1);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "INIT I2C_CR2: 0x%08lX \r\n", init_i2c_cr2);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "INIT I2C_OAR1: 0x%08lX \r\n", init_i2c_oar1);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "INIT I2C_ISR: 0x%08lX \r\n", init_i2c_isr);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            UART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            for (volatile int i = 0; i < 100000; i++);

            break;
        
        case INTERFACE_CLOCK:

            uint32_t clock_reg = RCC->CR;
            uint32_t clock_conf_reg = RCC->CFGR;
            uint32_t clock_conf_reg2 = RCC->CFGR2;
            uint32_t clock_conf_reg3 = RCC->CFGR3;

            sprintf(buffr, "Clock reg: 0x%08lX \r\n", clock_reg);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "Clock config reg: 0x%08lX \r\n", clock_conf_reg);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "Clock config reg2: 0x%08lX \r\n", clock_conf_reg2);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            sprintf(buffr, "Clock config reg3: 0x%08lX \r\n", clock_conf_reg3);
            UART_Transmit((const char*)buffr, strlen((char*)buffr));

            UART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            for (volatile int i = 0; i < 100000; i++);

            //##########################################################################################
            //############THE SECTION BELOW REPLICATES SystemCoreClockUpdate calculations.##############
            //##########################################################################################

            // SystemCoreClockUpdate();
            // uint32_t sys_clk = SystemCoreClock, sys_clk_man = 0;
            // uint32_t sys_clk_status = 0, sys_clk_pllmull = 0, sys_clk_pllsrc = 0, sys_clk_prediv = 0;
            
            // //HSI == 0x00000000U, HSE == 0x00000004U, PLL == 0x00000008U, HSI48 = 0x0000000CU;
            // sys_clk_status = LL_RCC_GetSysClkSource(); 

            // //RCC_CFGR_PLLMUL -> Maska -> (0xFUL << 18U)
            // sys_clk_pllmull = LL_RCC_PLL_GetMultiplicator();
            
            // //RCC_CFGR_PLLSRC -> Maska -> (0x3UL << 15U)
            // sys_clk_pllsrc = LL_RCC_PLL_GetMainSource();
            
            // //RCC_CFGR2_PREDIV -> Maska -> (0xFUL << 0U)
            // sys_clk_prediv = LL_RCC_PLL_GetPrediv() +1;
            // uint32_t modified_mull = (sys_clk_pllmull >> 18) + 2;

            // sys_clk_man = (HSI_VALUE/sys_clk_prediv) * modified_mull;
            
            // sprintf(buffr, "SYSCLK: %lu Hz\r\n", sys_clk);
            // UART_Transmit((const char*)buffr, strlen((char*)buffr));

            // sprintf(buffr, "SYSCLK SRC: %lu \r\n", sys_clk_status);
            // UART_Transmit((const char*)buffr, strlen((char*)buffr));

            // sprintf(buffr, "SYSCLK PLL MUL: %lu \r\n", sys_clk_pllmull);
            // UART_Transmit((const char*)buffr, strlen((char*)buffr));

            // sprintf(buffr, "SYSCLK PLL SRC: %lu \r\n", sys_clk_pllsrc);
            // UART_Transmit((const char*)buffr, strlen((char*)buffr));

            // sprintf(buffr, "SYSCLK PLL PREDIV: %lu \r\n", sys_clk_prediv);
            // UART_Transmit((const char*)buffr, strlen((char*)buffr));

            // sprintf(buffr, "MOD MULL: %lu \r\n", modified_mull);
            // UART_Transmit((const char*)buffr, strlen((char*)buffr));

            // sprintf(buffr, "MANUAL CLK: %lu \r\n", sys_clk_man);
            // UART_Transmit((const char*)buffr, strlen((char*)buffr));
            
            // UART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            // for (volatile int i = 0; i < 100000; i++);
            break;
        
        // case INTERFACE_CLOCK_INIT:

        //     sprintf(buffr, "Initial Clock reg: 0x%08lX \r\n", init_clk_reg);
        //     UART_Transmit((const char*)buffr, strlen((char*)buffr));

        //     sprintf(buffr, "Initial Clock config reg: 0x%08lX \r\n", init_clk_conf_reg);
        //     UART_Transmit((const char*)buffr, strlen((char*)buffr));

        //     sprintf(buffr, "Initial Clock config reg2: 0x%08lX \r\n", init_clk_conf_reg2);
        //     UART_Transmit((const char*)buffr, strlen((char*)buffr));

        //     sprintf(buffr, "Initial Clock config reg3: 0x%08lX \r\n", init_clk_conf_reg3);
        //     UART_Transmit((const char*)buffr, strlen((char*)buffr));

        //     UART_Transmit((const char*)"-----------------------------------------------\r\n",50);

        //     for (volatile int i = 0; i < 100000; i++);

        //     sprintf(buffr, "CFGR after configDomainSys fimcton: 0x%08lX \r\n", cfgr_after_configDomain);
        //     UART_Transmit((const char*)buffr, strlen((char*)buffr));

        //     UART_Transmit((const char*)"-----------------------------------------------\r\n",50);

        //     for (volatile int i = 0; i < 100000; i++);
        //     break;
        
        default:
            break;
    }
}