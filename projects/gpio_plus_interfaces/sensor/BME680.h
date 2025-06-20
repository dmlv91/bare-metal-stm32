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

int getTemperature (void);

#endif

#ifdef USE_SPI

#endif

#endif