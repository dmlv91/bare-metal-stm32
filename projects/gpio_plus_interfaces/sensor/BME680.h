#ifndef BME680_H
#define BME680_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "../utils/i2c.h"
#include "../utils/uart.h"

#ifdef USE_I2C

#define BME680_REG_CHIP_ID 0xD0
#define BME680_REG_CTRL_HUM 0X72
#define BME680_REG_CTRL_MEAS 0X74
#define BME680_REG_GAS_WAIT_0 0X64
#define BME680_REG_RES_HEAT_0 0X5A
#define BME680_REG_CTRL_GAS_1 0X71
#define BME680_REG_PAR_G1 0XED
#define BME680_REG_PAR_G2_MSB 0XEC
#define BME680_REG_PAR_G2_LSB 0XEB
#define BME680_REG_PAR_G3 0XEE
#define BME680_REG_RES_HEAT_RANGE_MASK (0X02 << 4)
#define BME680_REG_RES_HEAT_VAL 0X00

#define BME680_REG_PAR_T1_MSB 0XEA
#define BME680_REG_PAR_T1_LSB 0XE9
#define BME680_REG_PAR_T2_MSB 0X8B
#define BME680_REG_PAR_T2_LSB 0X8A
#define BME680_REG_PAR_T3 0X8C
#define BME680_REG_CONFIG 0x75
#define BME680_REG_TEMP_MSB 0x22
#define BME680_REG_TEMP_LSB 0x23
#define BME680_REG_TEMP_XLSB 0x24

#define BME680_REG_PRESS_MSB 0x1F
#define BME680_REG_PRESS_LSB 0x20
#define BME680_REG_PRESS_XLSB 0x21
#define BME680_REG_PAR_P1_MSB 0X8F
#define BME680_REG_PAR_P1_LSB 0X8E
#define BME680_REG_PAR_P2_MSB 0X91
#define BME680_REG_PAR_P2_LSB 0X90
#define BME680_REG_PAR_P3 0X92
#define BME680_REG_PAR_P4_MSB 0X95
#define BME680_REG_PAR_P4_LSB 0X94
#define BME680_REG_PAR_P5_MSB 0X97
#define BME680_REG_PAR_P5_LSB 0X96
#define BME680_REG_PAR_P6 0X99
#define BME680_REG_PAR_P7 0X98
#define BME680_REG_PAR_P8_MSB 0X9D
#define BME680_REG_PAR_P8_LSB 0X9C
#define BME680_REG_PAR_P9_MSB 0X9F
#define BME680_REG_PAR_P9_LSB 0X9E
#define BME680_REG_PAR_P10 0XA0

#define BME680_REG_HUM_MSB 0x25
#define BME680_REG_HUM_LSB 0x26
#define BME680_REG_PAR_H1_MSB 0XE3
#define BME680_REG_PAR_H1_LSB 0XE2
#define BME680_REG_PAR_H2_MSB 0XE1
#define BME680_REG_PAR_H2_LSB 0XE2
#define BME680_REG_PAR_H3_LSB 0XE4
#define BME680_REG_PAR_H4_LSB 0XE5
#define BME680_REG_PAR_H5_LSB 0XE6
#define BME680_REG_PAR_H6_LSB 0XE7
#define BME680_REG_PAR_H7_LSB 0XE8

#define BME680_HUM_OVERSAMPLING 0X01
#define BME680_TEMP_N_PRES_OVERSAMPLING (0X5 << 2) | (0X2 << 5)
#define BME680_GAS_HEAT_DURATION 0X59
#define BME680_NB_CONV 0X00
#define BME680_RUN_GAS_1 0X01

#define BME680_CHIP_ID 0x61
#define HEATER_TARGET_TEMP 300

bool initSensor (uint8_t sensor_addr);

void setForcedMode (uint8_t sensor_addr);

uint8_t calcResHeatVal (uint16_t targetTemp, uint8_t sensor_addr);

int calcIntTemperature (uint32_t temp_adc);
int calcIntPressure (uint32_t press_adc);
int calcIntHumidity (uint32_t hum_adc);

int getTemperature (void);
int getPressure (void);
int getHumidity (void);

#endif

#ifdef USE_SPI

#endif

#endif