#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "utils/clock.h"
#include "utils/gpio.h"
#include "utils/i2c.h"
#include "utils/uart.h"
#include "utils/debug.h"
#define BME680_I2C_ADDR 0x77

const uint8_t sensorAddress = 0x77;
uint8_t received_data;
char buffer[100];


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

void _init(void) {}


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
    UART_Transmit((const char*)buffer, strlen((char*)buffer));

    uint8_t par_g3 = I2C_ReadRegister(sensorAddress,0xEE);
    sprintf(buffer, "PAR_G3: 0x%02X \r\n", par_g3);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));

    uint8_t par_g2_msb = I2C_ReadRegister(sensorAddress,0xEC);
    sprintf(buffer, "PAR_G2_MSB: 0x%02X \r\n", par_g2_msb);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));

    uint8_t par_g2_lsb = I2C_ReadRegister(sensorAddress,0xEB);
    sprintf(buffer, "PAR_G2_LSB: 0x%02X \r\n", par_g2_lsb);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));

    uint8_t res_heat_range = I2C_ReadRegister(sensorAddress,(0x02<<4));
    sprintf(buffer, "RES_HEAT_RANGE: 0x%02X \r\n", res_heat_range);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    
    uint8_t res_heat_val = I2C_ReadRegister(sensorAddress,0x00);
    sprintf(buffer, "RES_HEAT_VAL: 0x%02X \r\n", res_heat_val);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    
    uint16_t par_g2 = (par_g2_msb << 8) | par_g2_lsb;
    var1 = ((double)par_g1/16.0f) + 49.0f;
    var2 = (((double)par_g2/32768.0f)*0.0005f) + 0.00235f;
    var3 = (double)par_g3/1024.0f;
    var4 = var1 * (1.0f + (var2*(double)targetTemp));
    var5 = var4 + (var3*(double)amb_temp);
    uint8_t res_heat = (uint8_t)(3.4f*((var5*(4.0f/(4.0f+(double)res_heat_range))*(1.0f/(1.0f+((double)res_heat_val*0.002f))))-25));

    sprintf(buffer, "CALCULATED_RES_HEAT: 0x%02X \r\n", res_heat);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    return res_heat;

}

void setForcedMode(uint8_t sensorAddress) {

    //Set humidity oversampling to 1x by writing 0x01 to 0x72
    sprintf(buffer, "Write 0x01 to 0x72.\r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));

    I2C_WriteRegister((sensorAddress << 1), ctrl_hum, hum_oversampling);
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set temperature oversampling to 2x by writing 0x2 to 0x74 << 5 
    //and set pressure oversampling to 16x by writing 0x5 to 0x74 << 2
    sprintf(buffer, "Write (0x5 << 2) | (0x2 << 5) to 0x74.\r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    I2C_WriteRegister((sensorAddress << 1), ctrl_meas, temp_n_pres_oversampling);
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set gas sensor hot plate temperature and heating duration.
    sprintf(buffer, "Write 0x59 to 0x64.\r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    I2C_WriteRegister((sensorAddress << 1), gas_wait_0,gas_heat_duration );
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set heater set-point with target resistance
    sprintf(buffer, "Read registers and calculate res_heat value.\r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    I2C_WriteRegister((sensorAddress << 1), res_heat_0, calc_res_heat_val(heaterTargetTemp,sensorAddress));
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);
    sprintf(buffer, "Write calculated res_heat to 0x5A.\r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    
    //set nb_conv value to confimr gas heater settings and set run_gas_1 to 1 to enable the heater
    sprintf(buffer, "Write (0x00 | (1 << 4)) to 0x71.\r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    I2C_WriteRegister((sensorAddress << 1), ctrl_gas_1,(nb_conv | (run_gas_1 << 4)));
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    sprintf(buffer, "Read all control registers:\r\n");
    UART_Transmit((const char*)buffer, strlen((char*)buffer));


    dbg = I2C_ReadRegister(sensorAddress,ctrl_hum);
    sprintf(buffer, "ctrl_hum: 0x%02X \r\n", dbg);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    dbg = I2C_ReadRegister(sensorAddress,ctrl_meas);
    sprintf(buffer, "ctrl_meas: 0x%02X \r\n", dbg);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    dbg = I2C_ReadRegister(sensorAddress,gas_wait_0);
    sprintf(buffer, "gas_wait_0: 0x%02X \r\n", dbg);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    dbg = I2C_ReadRegister(sensorAddress,res_heat_0);
    sprintf(buffer, "res_heat_0: 0x%02X \r\n", dbg);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    dbg = I2C_ReadRegister(sensorAddress,ctrl_gas_1);
    sprintf(buffer, "ctrl_gas_1: 0x%02X \r\n", dbg);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));


}

float calcTemperature(uint32_t temp_adc) {
    float var1;
    float var2;
    float t_fine;
    float temp_comp;

    uint8_t par_t1_msb = I2C_ReadRegister(sensorAddress,0xEA);
    uint8_t par_t1_lsb = I2C_ReadRegister(sensorAddress,0xE9);
    uint8_t par_t2_msb = I2C_ReadRegister(sensorAddress,0x8B);
    uint8_t par_t2_lsb = I2C_ReadRegister(sensorAddress,0x8A);
    uint16_t par_t1 = par_t1_lsb | (par_t1_msb << 8);
    uint16_t par_t2 = par_t2_lsb | (par_t2_msb << 8);
    uint8_t par_t3 = I2C_ReadRegister(sensorAddress,0x8C);

    var1 = (((double)temp_adc/16384.0f) - ((double)par_t1/1024.0f))*(double)par_t2;
    var2 = ((((double)temp_adc/131072.0f)-((double)par_t1/8192.0f))
        *(((double)temp_adc/131072.0f)- ((double)par_t1/8192.0f)))
        *((double)par_t3*16.0f);
    t_fine = var1+var2;
    temp_comp = t_fine/5120.0;

    return temp_comp;
}

int calcIntTemperature(uint32_t temp_adc) {
    int var1;
    int var2;
    int var3;

    uint8_t par_t1_msb = I2C_ReadRegister(sensorAddress,0xEA);
    uint8_t par_t1_lsb = I2C_ReadRegister(sensorAddress,0xE9);
    uint8_t par_t2_msb = I2C_ReadRegister(sensorAddress,0x8B);
    uint8_t par_t2_lsb = I2C_ReadRegister(sensorAddress,0x8A);
    uint16_t par_t1 = par_t1_lsb | (par_t1_msb << 8);
    uint16_t par_t2 = par_t2_lsb | (par_t2_msb << 8);
    uint8_t par_t3 = I2C_ReadRegister(sensorAddress,0x8C);

    sprintf(buffer, "par_t1: 0x%02X \r\n", par_t1);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    sprintf(buffer, "par_t2: 0x%02X \r\n", par_t2);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    sprintf(buffer, "par_t3: 0x%02X \r\n", par_t3);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1*(int32_t)par_t2) >> 11;
    var3 = ((((var1 >> 1)* (var1 >> 1))>>12)*((int32_t)par_t3 << 4)) >> 14;
    int t_fine = var2+var3;
    int temp_comp = ((t_fine*5)+128) >> 8;

    return temp_comp;

}

bool initSensor(uint8_t sensorAddress) {
    uint8_t BME_680_ID = 0xD0;
    received_data = I2C_ReadRegister(sensorAddress,BME_680_ID);
    // debugRegisters(INTERFACE_I2C);
    sprintf(buffer, "INIT SENSOR DEBUG 0xD0: 0x%02X \r\n", received_data);
    UART_Transmit((const char*)buffer, strlen((char*)buffer));
    if (received_data == 0x61) {
        return true;
    } else {
        return false;
    }

}

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

    if (initSensor(sensorAddress)) {

        sprintf(buffer, "SENSOR INIT COMPLETE. START FORCED MODE \r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        setForcedMode(sensorAddress);
    
        sprintf(buffer, "FORCED MODE COMPLETE. MAKE 1 MEASUREMENT.\r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
    
        sprintf(buffer, "Write 0x01 to 0x74.\r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        //change measuring mode to enable one forced measurment cycle (Set only first bit).
        uint8_t ctrl_meas_val = I2C_ReadRegister(sensorAddress,ctrl_meas);
        I2C_WriteRegister((sensorAddress << 1), ctrl_meas, (ctrl_meas_val |= (1 << 0)));
        while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

        dbg = I2C_ReadRegister(sensorAddress,ctrl_meas);
        sprintf(buffer, "ctrl_meas--------->: 0x%02X \r\n", dbg);
        UART_Transmit((const char*)buffer, strlen((char*)buffer));

        sprintf(buffer, "MODE CHANGED. CHECK NEW DATA FLAG VALUE.\r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));

        uint8_t meas_status_0 = 0x1D;
        uint8_t newDataFlag = I2C_ReadRegister(sensorAddress, meas_status_0);
        //print only 7th bit. If new data exists, 0x8 will be returned, otherwise 0.
        sprintf(buffer, "New Data: 0x%02X \r\n", newDataFlag );
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
    
        sprintf(buffer, "MODE CHANGED. READ TEMP VALUE.\r\n");
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
    
        uint8_t temp_msb_add = 0x22;
        uint8_t temp_lsb_add = 0x23;
        uint8_t temp_xlsb_add = 0x24;
    
        uint8_t temp_msb = I2C_ReadRegister(sensorAddress, temp_msb_add);
        sprintf(buffer, "DEBUG temp_msb: 0x%02X \r\n", temp_msb);
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        uint8_t temp_lsb = I2C_ReadRegister(sensorAddress, temp_lsb_add);
        sprintf(buffer, "DEBUG temp_lsb: 0x%02X \r\n", temp_lsb);
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        uint8_t temp_xlsb = I2C_ReadRegister(sensorAddress, temp_xlsb_add);
        sprintf(buffer, "DEBUG temp_xlsb: 0x%02X \r\n", temp_xlsb);
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        uint32_t temperature = ((uint32_t)temp_msb << 12) | ((uint32_t)temp_lsb << 4) | (temp_xlsb >> 4);
        
        sprintf(buffer, "DEBUG temperature (full 20 bits): 0x%08lX \r\n", temperature);
        UART_Transmit((const char*)buffer, strlen((char*)buffer));
        
        //As the data resolution depends on osrs_t value (16bits + (osrs_t - 1)) 3 lower bits are discarded because osrs_t is set to 2. 
        temperature = temperature >> 3;
    
        sprintf(buffer, "DEBUG temperature: 0x%08lX \r\n", temperature);
        UART_Transmit((const char*)buffer, strlen((char*)buffer));

        int realTemp = calcIntTemperature(temperature);
        sprintf(buffer, "Real temperature:  %d\r\n", realTemp);
        UART_Transmit((const char*)buffer, strlen((char*)buffer));

        uint8_t config_val = I2C_ReadRegister(sensorAddress,0x75);
        sprintf(buffer, "DEBUG config: 0x%02X \r\n", config_val);
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
