#include "BME680.h"

uint8_t dbg;
char writeBuffer[100];
uint8_t sensorAddress;
//res_heat value calculation corresponding to BME680 datasheet  
uint8_t calcResHeatVal(uint16_t targetTemp, uint8_t sensorAddress) {
    int amb_temp = 25;
    float var1;
    float var2;
    float var3;
    float var4;
    float var5;
    uint8_t par_g1 = I2C_ReadRegister(sensorAddress, BME680_REG_PAR_G1);
    uint8_t par_g3 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_G3);
    uint8_t par_g2_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_G2_MSB);
    uint8_t par_g2_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_G2_LSB);
    uint8_t res_heat_range = I2C_ReadRegister(sensorAddress, BME680_REG_RES_HEAT_RANGE_MASK);
    uint8_t res_heat_val = I2C_ReadRegister(sensorAddress,BME680_REG_RES_HEAT_VAL);
    uint16_t par_g2 = (par_g2_msb << 8) | par_g2_lsb;
    var1 = ((double)par_g1/16.0f) + 49.0f;
    var2 = (((double)par_g2/32768.0f)*0.0005f) + 0.00235f;
    var3 = (double)par_g3/1024.0f;
    var4 = var1 * (1.0f + (var2*(double)targetTemp));
    var5 = var4 + (var3*(double)amb_temp);
    uint8_t res_heat = (uint8_t)(3.4f*((var5*(4.0f/(4.0f+(double)res_heat_range))*(1.0f/(1.0f+((double)res_heat_val*0.002f))))-25));

    // sprintf(writeBuffer, "CALCULATED_RES_HEAT: 0x%02X \r\n", res_heat);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // sprintf(writeBuffer, "Write calculated res_heat to 0x5A.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    return res_heat;

}

void setForcedMode(uint8_t sensorAddress) {

    //Set humidity oversampling to 1x by writing 0x01 to 0x72

    I2C_WriteRegister((sensorAddress << 1), BME680_REG_CTRL_HUM, BME680_HUM_OVERSAMPLING);
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set temperature oversampling to 2x by writing 0x2 to 0x74 << 5 
    //and set pressure oversampling to 16x by writing 0x5 to 0x74 << 2
    I2C_WriteRegister((sensorAddress << 1), BME680_REG_CTRL_MEAS, BME680_TEMP_N_PRES_OVERSAMPLING);
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set gas sensor hot plate temperature and heating duration.
    I2C_WriteRegister((sensorAddress << 1), BME680_REG_GAS_WAIT_0,BME680_GAS_HEAT_DURATION);
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set heater set-point with target resistance
    // sprintf(writeBuffer, "Read registers and calculate res_heat value.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // sprintf(writeBuffer, "--------------------------------------------------------------------\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    

    I2C_WriteRegister((sensorAddress << 1), BME680_REG_RES_HEAT_0, calcResHeatVal(HEATER_TARGET_TEMP,sensorAddress));
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    //set nb_conv value to confimr gas heater settings and set run_gas_1 to 1 to enable the heater
    I2C_WriteRegister((sensorAddress << 1), BME680_REG_CTRL_GAS_1,(BME680_NB_CONV | (BME680_RUN_GAS_1 << 4)));
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    // sprintf(writeBuffer, "--------------------------------------------------------------------\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // sprintf(writeBuffer, "Read all control registers:\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));


    // dbg = I2C_ReadRegister(sensorAddress,BME680_REG_CTRL_HUM);
    // sprintf(writeBuffer, "ctrl_hum: 0x%02X \r\n", dbg);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // dbg = I2C_ReadRegister(sensorAddress,BME680_REG_CTRL_MEAS);
    // sprintf(writeBuffer, "ctrl_meas: 0x%02X \r\n", dbg);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // dbg = I2C_ReadRegister(sensorAddress,BME680_REG_GAS_WAIT_0);
    // sprintf(writeBuffer, "gas_wait_0: 0x%02X \r\n", dbg);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // dbg = I2C_ReadRegister(sensorAddress,BME680_REG_RES_HEAT_0);
    // sprintf(writeBuffer, "res_heat_0: 0x%02X \r\n", dbg);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // dbg = I2C_ReadRegister(sensorAddress,BME680_REG_CTRL_GAS_1);
    // sprintf(writeBuffer, "ctrl_gas_1: 0x%02X \r\n", dbg);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));


}



int calcIntTemperature(uint32_t temp_adc) {
    int var1;
    int var2;
    int var3;

    uint8_t par_t1_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T1_MSB);
    uint8_t par_t1_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T1_LSB);
    uint8_t par_t2_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T2_MSB);
    uint8_t par_t2_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T2_LSB);
    uint16_t par_t1 = par_t1_lsb | (par_t1_msb << 8);
    uint16_t par_t2 = par_t2_lsb | (par_t2_msb << 8);
    uint8_t par_t3 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T3);

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1*(int32_t)par_t2) >> 11;
    var3 = ((((var1 >> 1)* (var1 >> 1))>>12)*((int32_t)par_t3 << 4)) >> 14;
    int t_fine = var2+var3;
    int temp_comp = ((t_fine*5)+128) >> 8;

    return temp_comp;

}

int calcIntPressure(uint32_t press_adc) {
    
}

int getTemperature(void) {

    // sprintf(writeBuffer, "--------------------------------------------------------------------\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    setForcedMode(sensorAddress);

    // sprintf(writeBuffer, "--------------------------------------------------------------------\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // sprintf(writeBuffer, "FORCED MODE COMPLETE. MAKE 1 MEASUREMENT.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    // sprintf(writeBuffer, "Write 0x01 to 0x74.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    //change measuring mode to enable one forced measurment cycle (Set only first bit).
    uint8_t ctrl_meas_val = I2C_ReadRegister(sensorAddress,BME680_REG_CTRL_MEAS);
    I2C_WriteRegister((sensorAddress << 1), BME680_REG_CTRL_MEAS, (ctrl_meas_val |= (1 << 0)));
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);

    // dbg = I2C_ReadRegister(sensorAddress,BME680_REG_CTRL_MEAS);
    // sprintf(writeBuffer, "ctrl_meas--------->: 0x%02X \r\n", dbg);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    // sprintf(writeBuffer, "MODE CHANGED. CHECK NEW DATA FLAG VALUE.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    uint8_t meas_status_0 = 0x1D;
    uint8_t newDataFlag = I2C_ReadRegister(sensorAddress, meas_status_0);
    //print only 7th bit. If new data exists, 0x8 will be returned, otherwise 0.
    // sprintf(writeBuffer, "New Data: 0x%02X \r\n", newDataFlag );
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    // sprintf(writeBuffer, "MODE CHANGED. READ TEMP VALUE.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // sprintf(writeBuffer, "--------------------------------------------------------------------\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    uint8_t temp_msb = I2C_ReadRegister(sensorAddress, BME680_REG_TEMP_MSB);
    // sprintf(writeBuffer, "DEBUG temp_msb: 0x%02X \r\n", temp_msb);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    
    uint8_t temp_lsb = I2C_ReadRegister(sensorAddress, BME680_REG_TEMP_LSB);
    // sprintf(writeBuffer, "DEBUG temp_lsb: 0x%02X \r\n", temp_lsb);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    
    uint8_t temp_xlsb = I2C_ReadRegister(sensorAddress, BME680_REG_TEMP_XLSB);
    // sprintf(writeBuffer, "DEBUG temp_xlsb: 0x%02X \r\n", temp_xlsb);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    
    uint32_t temperature = ((uint32_t)temp_msb << 12) | ((uint32_t)temp_lsb << 4) | (temp_xlsb >> 4);
    int realTemp = calcIntTemperature(temperature);
    
    return realTemp;

}

int getPressure(void) {

    setForcedMode(sensorAddress);

    uint8_t press_msb = I2C_ReadRegister(sensorAddress, BME680_REG_PRESS_MSB);
    uint8_t press_lsb = I2C_ReadRegister(sensorAddress, BME680_REG_PRESS_LSB);
    uint8_t press_xlsb = I2C_ReadRegister(sensorAddress, BME680_REG_PRESS_XLSB);

    uint32_t press_adc = ((uint32_t)press_msb << 12) | ((uint32_t)press_lsb << 4) | (press_xlsb >> 4);
    int realPress = calcIntPressure(press_adc);

    return realPress;
}

bool initSensor(uint8_t sensor_addr) {
    uint8_t BME_680_ID = 0xD0;
    uint8_t chip_ID = I2C_ReadRegister(sensor_addr,BME_680_ID);
    // debugRegisters(INTERFACE_I2C);
    sprintf(writeBuffer, "INIT SENSOR DEBUG 0xD0: 0x%02X \r\n", chip_ID);
    UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    if (chip_ID == BME680_CHIP_ID) {
        sensorAddress = sensor_addr;
        return true;
    } else {
        return false;
    }
}
