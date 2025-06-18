#include "clock.h"

uint32_t    init_clk_reg = 0, 
            init_clk_conf_reg = 0, 
            init_clk_conf_reg2 = 0, 
            init_clk_conf_reg3 = 0,
            cfgr_after_configDomain = 0;

            
void systemClockConfig() {
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
    
    // NOTE: manually setting PLLXTPRE bit to 1 as the ConfigDomain_SYS function apparently does not.
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


void initClock() {
    systemClockConfig();

}