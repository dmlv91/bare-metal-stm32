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

uint8_t hum_oversampling = 0x01;
uint8_t temp_n_pres_oversampling = (0x5 << 2) | (0x2 << 5);
uint8_t gas_heat_duration = 0x59;
uint8_t nb_conv = 0x0;
uint8_t run_gas_1 = 0x01;
uint8_t ctrl_hum = 0x72;
uint8_t ctrl_meas = 0x74;
uint8_t gas_wait_0 = 0x64;
uint8_t res_heat_0 = 0x5A;
uint8_t ctrl_gas_1 = 0x71;
int heaterTargetTemp = 300;
uint8_t dbg;

typedef enum {
    INTERFACE_I2C_INIT,
    INTERFACE_I2C,
    INTERFACE_CLOCK_INIT,
    INTERFACE_CLOCK,
} InterfaceType;

void debugRegisters(InterfaceType interface);

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

    debugRegisters(INTERFACE_I2C);
   
}

void USART_Transmit(const char *data, uint32_t size) {
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

void I2C_WriteRegister(uint8_t sensorAddress, uint8_t regAddress, uint8_t data) {
    sprintf(buffer, "Writing data (0x%02x)to register: 0x%02X \r\n", data, regAddress);
    USART_Transmit((const char *)buffer, strlen((char*)buffer));
    LL_I2C_HandleTransfer(I2C1, sensorAddress, LL_I2C_ADDRESSING_MODE_7BIT, 2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
    LL_I2C_TransmitData8(I2C1, regAddress);
    while (!LL_I2C_IsActiveFlag_TXE(I2C1));
   
    LL_I2C_TransmitData8(I2C1, data);
    while (!LL_I2C_IsActiveFlag_STOP(I2C1));
    LL_I2C_ClearFlag_STOP(I2C1);
}

uint8_t I2C_ReadRegister(uint8_t sensorAddress, uint8_t regAddress) {
    uint8_t data = 0;
    sprintf(buffer, "Reading data from register: 0x%02X \r\n", regAddress);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
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
    sprintf((char*)buffer, "START I2C BUS SCAN!\r\n");
    USART_Transmit(buffer, strlen((char*)buffer));
    for (uint8_t address = 0; address < 128; address++) {
        // sprintf((char*)buffer, "Address 0x%02X\r\n", address);
        // USART_Transmit(buffer, strlen((char*)buffer));
        if (hasAddress == 0) {
            LL_I2C_HandleTransfer(I2C1, (address << 1), LL_I2C_ADDRESSING_MODE_7BIT, 0, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
            if (address == 0x77) {

                USART_Transmit((const char*)"-----------------------------------------------\r\n",50);
                
                debugRegisters(INTERFACE_I2C);
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
                    sprintf((char*)buffer, "Device found at 0x%02X\n", address);
                    USART_Transmit((const char *)buffer, strlen((char*)buffer));
                    LL_I2C_ClearFlag_ADDR(I2C1);
                    hasAddress = 1;
                    break;
                }
            }
            LL_I2C_ClearFlag_STOP(I2C1); // Clear STOP flag for next address
            for (volatile int i = 0; i < 100000; i++);
        }
    }
    sprintf((char*)buffer, "SCAN COMPLETED!\r\n");
    USART_Transmit(buffer, strlen((char*)buffer));
}

void debugRegisters(InterfaceType interface) {

    USART_Transmit((const char*)"################################################### \r\n",50);
    sprintf(buffer, "Getting register states for interface: %d  \r\n", interface);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));

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

            sprintf(buffer, "I2C_TIMINGR: 0x%08lX \r\n", i2c_timingr);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "I2C_CR1: 0x%08lX \r\n", i2c_cr1);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "I2C_CR2: 0x%08lX \r\n", i2c_cr2);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "I2C_OAR1: 0x%08lX \r\n", i2c_oar1);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "I2C_ISR: 0x%08lX \r\n", i2c_isr);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "I2C_ICR: 0x%08lX \r\n", i2c_icr);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "I2C_PECR: 0x%08lX \r\n", i2c_pecr);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "I2C_RXDR: 0x%08lX \r\n", i2c_rxdr);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "I2C_TXDR: 0x%08lX \r\n", i2c_txdr);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            USART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            for (volatile int i = 0; i < 100000; i++);
            break;

        case INTERFACE_I2C_INIT:
            sprintf(buffer, "INIT I2C_TIMINGR: 0x%08lX \r\n", init_i2c_timingr);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "INIT I2C_CR1: 0x%08lX \r\n", init_i2c_cr1);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "INIT I2C_CR2: 0x%08lX \r\n", init_i2c_cr2);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "INIT I2C_OAR1: 0x%08lX \r\n", init_i2c_oar1);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "INIT I2C_ISR: 0x%08lX \r\n", init_i2c_isr);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            USART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            for (volatile int i = 0; i < 100000; i++);

            break;
        
        case INTERFACE_CLOCK:

            uint32_t clock_reg = RCC->CR;
            uint32_t clock_conf_reg = RCC->CFGR;
            uint32_t clock_conf_reg2 = RCC->CFGR2;
            uint32_t clock_conf_reg3 = RCC->CFGR3;

            sprintf(buffer, "Clock reg: 0x%08lX \r\n", clock_reg);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "Clock config reg: 0x%08lX \r\n", clock_conf_reg);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "Clock config reg2: 0x%08lX \r\n", clock_conf_reg2);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "Clock config reg3: 0x%08lX \r\n", clock_conf_reg3);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            USART_Transmit((const char*)"-----------------------------------------------\r\n",50);

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
            
            // sprintf(buffer, "SYSCLK: %lu Hz\r\n", sys_clk);
            // USART_Transmit((const char*)buffer, strlen((char*)buffer));

            // sprintf(buffer, "SYSCLK SRC: %lu \r\n", sys_clk_status);
            // USART_Transmit((const char*)buffer, strlen((char*)buffer));

            // sprintf(buffer, "SYSCLK PLL MUL: %lu \r\n", sys_clk_pllmull);
            // USART_Transmit((const char*)buffer, strlen((char*)buffer));

            // sprintf(buffer, "SYSCLK PLL SRC: %lu \r\n", sys_clk_pllsrc);
            // USART_Transmit((const char*)buffer, strlen((char*)buffer));

            // sprintf(buffer, "SYSCLK PLL PREDIV: %lu \r\n", sys_clk_prediv);
            // USART_Transmit((const char*)buffer, strlen((char*)buffer));

            // sprintf(buffer, "MOD MULL: %lu \r\n", modified_mull);
            // USART_Transmit((const char*)buffer, strlen((char*)buffer));

            // sprintf(buffer, "MANUAL CLK: %lu \r\n", sys_clk_man);
            // USART_Transmit((const char*)buffer, strlen((char*)buffer));
            
            // USART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            // for (volatile int i = 0; i < 100000; i++);
            break;
        
        case INTERFACE_CLOCK_INIT:

            sprintf(buffer, "Initial Clock reg: 0x%08lX \r\n", init_clk_reg);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "Initial Clock config reg: 0x%08lX \r\n", init_clk_conf_reg);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "Initial Clock config reg2: 0x%08lX \r\n", init_clk_conf_reg2);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            sprintf(buffer, "Initial Clock config reg3: 0x%08lX \r\n", init_clk_conf_reg3);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            USART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            for (volatile int i = 0; i < 100000; i++);

            sprintf(buffer, "CFGR after configDomainSys fimcton: 0x%08lX \r\n", cfgr_after_configDomain);
            USART_Transmit((const char*)buffer, strlen((char*)buffer));

            USART_Transmit((const char*)"-----------------------------------------------\r\n",50);

            for (volatile int i = 0; i < 100000; i++);
            break;
        
        default:
            break;
    }
}

//res_heat value calculation corresponding to BME680 datasheet  
uint8_t calc_res_heat_val(uint16_t targetTemp, uint8_t sensorAddress) {
    int amb_temp = 25;
    float var1;
    float var2;
    float var3;
    float var4;
    float var5;
    uint8_t par_g1 = I2C_ReadRegister(sensorAddress, 0xED);
    sprintf(buffer, "PAR_G1: 0x%02X \r\n", par_g1);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));

    uint8_t par_g3 = I2C_ReadRegister(sensorAddress,0xEE);
    sprintf(buffer, "PAR_G3: 0x%02X \r\n", par_g3);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));

    uint8_t par_g2_msb = I2C_ReadRegister(sensorAddress,0xEC);
    sprintf(buffer, "PAR_G2_MSB: 0x%02X \r\n", par_g2_msb);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));

    uint8_t par_g2_lsb = I2C_ReadRegister(sensorAddress,0xEB);
    sprintf(buffer, "PAR_G2_LSB: 0x%02X \r\n", par_g2_lsb);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));

    uint8_t res_heat_range = I2C_ReadRegister(sensorAddress,(0x02<<4));
    sprintf(buffer, "RES_HEAT_RANGE: 0x%02X \r\n", res_heat_range);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    
    uint8_t res_heat_val = I2C_ReadRegister(sensorAddress,0x00);
    sprintf(buffer, "RES_HEAT_VAL: 0x%02X \r\n", res_heat_val);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    
    uint16_t par_g2 = (par_g2_msb << 8) | par_g2_lsb;
    var1 = ((double)par_g1/16.0f) + 49.0f;
    var2 = (((double)par_g2/32768.0f)*0.0005f) + 0.00235f;
    var3 = (double)par_g3/1024.0f;
    var4 = var1 * (1.0f + (var2*(double)targetTemp));
    var5 = var4 + (var3*(double)amb_temp);
    uint8_t res_heat = (uint8_t)(3.4f*((var5*(4.0f/(4.0f+(double)res_heat_range))*(1.0f/(1.0f+((double)res_heat_val*0.002f))))-25));

    sprintf(buffer, "CALCULATED_RES_HEAT: 0x%02X \r\n", res_heat);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    return res_heat;

}

void setForcedMode(uint8_t sensorAddress) {

    //Set humidity oversampling to 1x by writing 0x01 to 0x72
    sprintf(buffer, "Write 0x01 to 0x72.\r\n");
    USART_Transmit((const char*)buffer, strlen((char*)buffer));

    I2C_WriteRegister((sensorAddress << 1), ctrl_hum, hum_oversampling);
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set temperature oversampling to 2x by writing 0x2 to 0x74 << 5 
    //and set pressure oversampling to 16x by writing 0x5 to 0x74 << 2
    sprintf(buffer, "Write (0x5 << 2) | (0x2 << 5) to 0x74.\r\n");
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    I2C_WriteRegister((sensorAddress << 1), ctrl_meas, temp_n_pres_oversampling);
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set gas sensor hot plate temperature and heating duration.
    sprintf(buffer, "Write 0x59 to 0x64.\r\n");
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    I2C_WriteRegister((sensorAddress << 1), gas_wait_0,gas_heat_duration );
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set heater set-point with target resistance
    sprintf(buffer, "Read registers and calculate res_heat value.\r\n");
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    I2C_WriteRegister((sensorAddress << 1), res_heat_0, calc_res_heat_val(heaterTargetTemp,sensorAddress));
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);
    sprintf(buffer, "Write calculated res_heat to 0x5A.\r\n");
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    
    //set nb_conv value to confimr gas heater settings and set run_gas_1 to 1 to enable the heater
    sprintf(buffer, "Write (0x00 | (1 << 4)) to 0x71.\r\n");
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    I2C_WriteRegister((sensorAddress << 1), ctrl_gas_1,(nb_conv | (run_gas_1 << 4)));
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    sprintf(buffer, "Read all control registers:\r\n");
    USART_Transmit((const char*)buffer, strlen((char*)buffer));


    dbg = I2C_ReadRegister(sensorAddress,ctrl_hum);
    sprintf(buffer, "ctrl_hum: 0x%02X \r\n", dbg);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    dbg = I2C_ReadRegister(sensorAddress,ctrl_meas);
    sprintf(buffer, "ctrl_meas: 0x%02X \r\n", dbg);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    dbg = I2C_ReadRegister(sensorAddress,gas_wait_0);
    sprintf(buffer, "gas_wait_0: 0x%02X \r\n", dbg);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    dbg = I2C_ReadRegister(sensorAddress,res_heat_0);
    sprintf(buffer, "res_heat_0: 0x%02X \r\n", dbg);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    dbg = I2C_ReadRegister(sensorAddress,ctrl_gas_1);
    sprintf(buffer, "ctrl_gas_1: 0x%02X \r\n", dbg);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));


}

bool initSensor(uint8_t sensorAddress) {
    uint8_t BME_680_ID = 0xD0;
    received_data = I2C_ReadRegister(sensorAddress,BME_680_ID);
    // debugRegisters(INTERFACE_I2C);
    sprintf(buffer, "INIT SENSOR DEBUG 0xD0: 0x%02X \r\n", received_data);
    USART_Transmit((const char*)buffer, strlen((char*)buffer));
    if (received_data == 0x61) {
        return true;
    } else {
        return false;
    }

}

int main(void)
{
    uint8_t BME_680 = 0x77;
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
    
    debugRegisters(INTERFACE_CLOCK);
 

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
    USART_Transmit((const char*)buffer, strlen((char*)buffer));

    if (initSensor(BME_680)) {

        sprintf(buffer, "SENSOR INIT COMPLETE. START FORCED MODE \r\n");
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        setForcedMode(BME_680);
    
        sprintf(buffer, "FORCED MODE COMPLETE. MAKE 1 MEASUREMENT.\r\n");
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
    
        sprintf(buffer, "Write 0x01 to 0x74.\r\n");
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
        //change measuring mode to enable one forced measurment cycle (Set only first bit).
        uint8_t ctrl_meas_val = I2C_ReadRegister(BME_680,ctrl_meas);
        I2C_WriteRegister((BME_680 << 1), ctrl_meas, (ctrl_meas_val |= (1 << 0)));
        while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

        dbg = I2C_ReadRegister(BME_680,ctrl_meas);
        sprintf(buffer, "ctrl_meas--------->: 0x%02X \r\n", dbg);
        USART_Transmit((const char*)buffer, strlen((char*)buffer));

        sprintf(buffer, "MODE CHANGED. CHECK NEW DATA FLAG VALUE.\r\n");
        USART_Transmit((const char*)buffer, strlen((char*)buffer));

        uint8_t meas_status_0 = 0x1D;
        uint8_t newDataFlag = I2C_ReadRegister(BME_680, meas_status_0);
        //print only 7th bit. If new data exists, 0x8 will be returned, otherwise 0.
        sprintf(buffer, "New Data: 0x%02X \r\n", (newDataFlag & (1<<7)));
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
    
        sprintf(buffer, "MODE CHANGED. READ TEMP VALUE.\r\n");
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
    
        uint8_t temp_msb_add = 0x22;
        uint8_t temp_lsb_add = 0x23;
        uint8_t temp_xlsb_add = 0x24;
    
        uint8_t temp_msb = I2C_ReadRegister(BME_680, temp_msb_add);
        sprintf(buffer, "DEBUG temp_msb: 0x%02X \r\n", temp_msb);
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        uint8_t temp_lsb = I2C_ReadRegister(BME_680, temp_lsb_add);
        sprintf(buffer, "DEBUG temp_lsb: 0x%02X \r\n", temp_lsb);
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        uint8_t temp_xlsb = I2C_ReadRegister(BME_680, temp_xlsb_add);
        sprintf(buffer, "DEBUG temp_xlsb: 0x%02X \r\n", temp_xlsb);
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        uint32_t temperature = ((uint32_t)temp_msb << 12) | ((uint32_t)temp_lsb << 4) | (temp_xlsb >> 4);
        
        sprintf(buffer, "DEBUG temperature (full 20 bits): 0x%08lX \r\n", temperature);
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        //As the data resolution depends on osrs_t value (16bits + (osrs_t - 1)) 3 lower bits are discarded because osrs_t is set to 2. 
        temperature = temperature >> (20-17);
    
        sprintf(buffer, "DEBUG temperature: 0x%08lX \r\n", temperature);
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
        
    } else {
        sprintf(buffer, "Sensor initialization error. Check I2C data bus and sensor\r\n");
        USART_Transmit((const char*)buffer, strlen((char*)buffer));
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
