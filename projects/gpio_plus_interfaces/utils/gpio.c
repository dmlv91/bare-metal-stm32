#include "gpio.h"


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


void initGPIO(void) {
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOC);
}