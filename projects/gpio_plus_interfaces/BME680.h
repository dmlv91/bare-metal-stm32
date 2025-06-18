#ifndef BME680_H
#define BME680_H

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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

#define BME680_CHIP_ID 0x61
#define HEATER_TARGET_TEMP 300



bool initSensor (uint8_t sensor_addr);

void setForcedMode (uint8_t sensor_addr);

uint8_t calcResHeatVal (uint16_t targetTemp, uint8_t sensor_addr);

int calcIntTemperature (uint32_t temp_adc);

#endif

#ifdef USE_SPI

#endif

#endif