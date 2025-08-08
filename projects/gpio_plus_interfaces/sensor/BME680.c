#include "BME680.h"

uint8_t dbg;
char writeBuffer[100];
uint8_t sensorAddress;
int t_fine;
int temp_comp;
typedef struct {
    uint8_t par_g1;
    uint8_t par_g3;
    uint8_t par_g2_msb;
    uint8_t par_g2_lsb;
    uint8_t par_t1_msb;
    uint8_t par_t1_lsb;
    uint8_t par_t2_msb;
    uint8_t par_t2_lsb;
    uint8_t par_t3;
    uint8_t par_p1_msb;
    uint8_t par_p1_lsb;
    uint8_t par_p2_msb;
    uint8_t par_p2_lsb;
    uint8_t par_p3;
    uint8_t par_p4_msb;
    uint8_t par_p4_lsb;
    uint8_t par_p5_msb;
    uint8_t par_p5_lsb;
    uint8_t par_p6;
    uint8_t par_p7;
    uint8_t par_p8_msb;
    uint8_t par_p8_lsb;
    uint8_t par_p9_msb;
    uint8_t par_p9_lsb;
    uint8_t par_p10;
    uint8_t par_h1_msb;
    uint8_t par_h1_lsb;
    uint8_t par_h2_msb;
    uint8_t par_h2_lsb;
    uint8_t par_h3;
    uint8_t par_h4;
    uint8_t par_h5;
    uint8_t par_h6;
    uint8_t par_h7;
    uint8_t res_heat_range;

} bme680_calib_data;

bme680_calib_data calib;
//res_heat value calculation corresponding to BME680 datasheet  
static uint8_t calcResHeatVal(uint16_t targetTemp, uint8_t sensorAddress) {
    int amb_temp = 25;
    float var1;
    float var2;
    float var3;
    float var4;
    float var5;
    
    uint8_t res_heat_val = I2C_ReadRegister(sensorAddress,BME680_REG_RES_HEAT_VAL);
    uint16_t par_g2 = (calib.par_g2_msb << 8) | calib.par_g2_lsb;
    var1 = ((double)calib.par_g1/16.0f) + 49.0f;
    var2 = (((double)par_g2/32768.0f)*0.0005f) + 0.00235f;
    var3 = (double)calib.par_g3/1024.0f;
    var4 = var1 * (1.0f + (var2*(double)targetTemp));
    var5 = var4 + (var3*(double)amb_temp);
    uint8_t res_heat = (uint8_t)(3.4f*((var5*(4.0f/(4.0f+(double)calib.res_heat_range))*(1.0f/(1.0f+((double)res_heat_val*0.002f))))-25));

    // sprintf(writeBuffer, "CALCULATED_RES_HEAT: 0x%02X \r\n", res_heat);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // sprintf(writeBuffer, "Write calculated res_heat to 0x5A.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    return res_heat;

}

static void setForcedMode(uint8_t sensorAddress) {

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



static int calcIntTemperature(uint32_t temp_adc) {
    int var1;
    int var2;
    int var3;

    
    uint16_t par_t1 = calib.par_t1_lsb | (calib.par_t1_msb << 8);
    uint16_t par_t2 = calib.par_t2_lsb | (calib.par_t2_msb << 8);
    

    var1 = ((int32_t)temp_adc >> 3) - ((int32_t)par_t1 << 1);
    var2 = (var1*(int32_t)par_t2) >> 11;
    var3 = ((((var1 >> 1)* (var1 >> 1))>>12)*((int32_t)calib.par_t3 << 4)) >> 14;
    t_fine = var2+var3;
    temp_comp = ((t_fine*5)+128) >> 8;

    return (uint32_t)temp_comp;

}

static uint32_t calcIntPressure(uint32_t press_adc) {

    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t press_comp;

    uint16_t par_p1 = calib.par_p1_lsb | (calib.par_p1_msb << 8);
    uint16_t par_p2 = calib.par_p2_lsb | (calib.par_p2_msb << 8);
    uint16_t par_p4 = calib.par_p4_lsb | (calib.par_p4_msb << 8);
    uint16_t par_p5 = calib.par_p5_lsb | (calib.par_p5_msb << 8);
    uint16_t par_p8 = calib.par_p8_lsb | (calib.par_p8_msb << 8);
    uint16_t par_p9 = calib.par_p9_lsb | (calib.par_p9_msb << 8);

    var1 = (((int32_t)t_fine) >> 1) - 64000;
    var2 = ((((var1 >> 2)*(var1 >> 2)) >> 11)* (int32_t)calib.par_p6) >> 2;
    var2 = var2 + ((var1 * (int32_t)par_p5) << 1);
    var2 = (var2 >> 2) + ((int32_t)par_p4 << 16);
    var1 = (((((var1 >> 2)*(var1 >> 2)) >> 13)*((int32_t)calib.par_p3 << 5)) >> 3) +
        (((int32_t)par_p2 * var1) >> 1);
    var1 = var1 >> 18;
    var1 = ((32768 + var1)*(int32_t)par_p1) >> 15;
    press_comp = 1048576 - press_adc;
    press_comp = (uint32_t)((press_comp - (var2 >> 12))*((uint32_t)3125));
    if (press_comp >= (1 << 30)) {
        press_comp = ((press_comp/(uint32_t)var1) << 1);
    } else {
        press_comp = ((press_comp << 1)/(uint32_t)var1);
    }
    var1 = ((int32_t)par_p9*(int32_t)(((press_comp >> 3) * (press_comp >> 3))>>13)) >> 12;
    var2 = ((int32_t)(press_comp >> 2)*(int32_t)par_p8) >> 13;
    var3 = ((int32_t)(press_comp >> 8)*(int32_t)(press_comp >> 8) *
        (int32_t)(press_comp >> 8) *(int32_t)calib.par_p10) >> 17;
    press_comp = (int32_t)(press_comp)+((var1 + var2 + var3 + ((int32_t)calib.par_p7 << 7)) >> 4);

    return (uint32_t)press_comp;

}

static uint32_t calcIntHumidity(uint16_t hum_adc) {
    int32_t var1;
    int32_t var2;
    int32_t var3;
    int32_t var4;
    int32_t var5;
    int32_t var6;
    int32_t temp_scaled;
    int32_t hum_comp;

    uint16_t par_h1 = calib.par_h1_lsb | (calib.par_h1_msb << 8);
    uint16_t par_h2 = calib.par_h2_lsb | (calib.par_h2_msb << 8);

    temp_scaled = (int32_t)temp_comp;
    var1 = (int32_t)(hum_adc - ((int32_t)((int32_t)par_h1 << 4))) -
           (((temp_scaled * (int32_t)calib.par_h3) / ((int32_t)100)) >> 1);
    var2 = ((int32_t)par_h2 * (((temp_scaled * (int32_t)calib.par_h4) / ((int32_t)100)) +
          (((temp_scaled * ((temp_scaled * (int32_t)calib.par_h5) / ((int32_t)100))) >> 6) / ((int32_t)100)) +
          ((int32_t)(1 << 14)))) >> 10;
    var3 = var1 * var2;
    var4 = (((int32_t)calib.par_h6 << 7) + ((temp_scaled * (int32_t)calib.par_h7) / ((int32_t)100))) >> 4;
    var5 = ((var3 >> 14) * (var3 >> 14)) >> 10;
    var6 = (var4 * var5) >> 1;
    hum_comp = (((var3 + var6) >> 10) * ((int32_t)1000)) >> 12;
    sprintf(writeBuffer, "temp_scaled %d;  hum_comp %d\r\n", temp_scaled,hum_comp);
    UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // if (hum_comp > 100000) /* Cap at 100%rH */
    // {
    //     hum_comp = 100000;
    // }
    // else if (hum_comp < 0)
    // {
    //     hum_comp = 0;
    // }
    return (uint32_t)hum_comp;
}

static void triggerMeasurementCycle(void) {
    //change measuring mode to enable one forced measurment cycle (Set only first bit).
    uint8_t ctrl_meas_val = I2C_ReadRegister(sensorAddress,BME680_REG_CTRL_MEAS);
    I2C_WriteRegister((sensorAddress << 1), BME680_REG_CTRL_MEAS, (ctrl_meas_val |= (1 << 0)));
    while(!LL_I2C_IsActiveFlag_TXE && LL_I2C_IsActiveFlag_RXNE);
}

int getTemperature(void) {

    // sprintf(writeBuffer, "--------------------------------------------------------------------\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    
    triggerMeasurementCycle();

    // sprintf(writeBuffer, "--------------------------------------------------------------------\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    // sprintf(writeBuffer, "FORCED MODE COMPLETE. MAKE 1 MEASUREMENT.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    // sprintf(writeBuffer, "Write 0x01 to 0x74.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    

    // dbg = I2C_ReadRegister(sensorAddress,BME680_REG_CTRL_MEAS);
    // sprintf(writeBuffer, "ctrl_meas--------->: 0x%02X \r\n", dbg);
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    // sprintf(writeBuffer, "MODE CHANGED. CHECK NEW DATA FLAG VALUE.\r\n");
    // UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));

    // uint8_t meas_status_0 = 0x1D;
    // uint8_t newDataFlag = I2C_ReadRegister(sensorAddress, meas_status_0);
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

    triggerMeasurementCycle();
    uint8_t press_msb = I2C_ReadRegister(sensorAddress, BME680_REG_PRESS_MSB);
    uint8_t press_lsb = I2C_ReadRegister(sensorAddress, BME680_REG_PRESS_LSB);
    uint8_t press_xlsb = I2C_ReadRegister(sensorAddress, BME680_REG_PRESS_XLSB);

    uint32_t press_adc = ((uint32_t)press_msb << 12) | ((uint32_t)press_lsb << 4) | (press_xlsb >> 4);
    uint32_t realPress = calcIntPressure(press_adc);

    return realPress;
}

int getHumidity(void) {

    triggerMeasurementCycle();
    uint8_t hum_msb = I2C_ReadRegister(sensorAddress, BME680_REG_PRESS_MSB);
    uint8_t hum_lsb = I2C_ReadRegister(sensorAddress, BME680_REG_PRESS_LSB);

    uint16_t hum_adc = hum_lsb | (hum_msb << 8);
    uint32_t realHumidity = calcIntHumidity(hum_adc);

    return realHumidity;
}

static void readCalibrationData() {
    calib.par_g1 = I2C_ReadRegister(sensorAddress, BME680_REG_PAR_G1);
    calib.par_g3 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_G3);
    calib.par_g2_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_G2_MSB);
    calib.par_g2_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_G2_LSB);
    calib.res_heat_range = I2C_ReadRegister(sensorAddress, BME680_REG_RES_HEAT_RANGE_MASK);
    calib.par_t1_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T1_MSB);
    calib.par_t1_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T1_LSB);
    calib.par_t2_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T2_MSB);
    calib.par_t2_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T2_LSB);
    calib.par_t3 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_T3);
    calib.par_p1_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P1_MSB);
    calib.par_p1_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P1_LSB);
    calib.par_p2_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P2_MSB);
    calib.par_p2_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P2_LSB);
    calib.par_p3 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P3);
    calib.par_p4_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P4_MSB);
    calib.par_p4_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P4_LSB);
    calib.par_p5_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P5_MSB);
    calib.par_p5_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P5_LSB);
    calib.par_p6 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P6);
    calib.par_p7 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P7);
    calib.par_p8_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P8_MSB);
    calib.par_p8_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P8_LSB);
    calib.par_p9_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P9_MSB);
    calib.par_p9_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P9_LSB);
    calib.par_p10 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_P10);
    calib.par_h1_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H1_MSB);
    calib.par_h1_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H1_LSB);
    calib.par_h2_msb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H2_MSB);
    calib.par_h2_lsb = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H2_LSB);
    calib.par_h3 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H3);
    calib.par_h4 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H4);
    calib.par_h5 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H5);
    calib.par_h6 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H6);
    calib.par_h7 = I2C_ReadRegister(sensorAddress,BME680_REG_PAR_H7);
}

bool initSensor(uint8_t sensor_addr) {
    uint8_t BME_680_ID = 0xD0;
    uint8_t chip_ID = I2C_ReadRegister(sensor_addr,BME_680_ID);
    // debugRegisters(INTERFACE_I2C);
    sprintf(writeBuffer, "INIT SENSOR DEBUG 0xD0: 0x%02X \r\n", chip_ID);
    UART_Transmit((const char*)writeBuffer, strlen((char*)writeBuffer));
    if (chip_ID == BME680_CHIP_ID) {
        sensorAddress = sensor_addr;
        readCalibrationData();
        setForcedMode(sensorAddress);
        return true;
    } else {
        return false;
    }
}
