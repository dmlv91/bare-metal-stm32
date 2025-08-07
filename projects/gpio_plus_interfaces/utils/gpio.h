#include "stm32f0xx_ll_gpio.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f091xc.h"

#define GPIO_I2C_SCL LL_GPIO_PIN_9
#define GPIO_I2C_SDA LL_GPIO_PIN_8
#define GPIO_UART_TX LL_GPIO_PIN_9
#define GPIO_UART_RX LL_GPIO_PIN_10
#define GPIO_MCO LL_GPIO_PIN_8
#define GPIO_LED2 LL_GPIO_PIN_5

void initGPIO(void);

void setGPIO(GPIO_TypeDef *GPIOx, uint32_t Pin, uint32_t Mode, uint32_t Type, uint32_t Speed, uint32_t Pull, uint32_t AltFunc);