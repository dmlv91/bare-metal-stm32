#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_usart.h"
#include "stm32f0xx_ll_bus.h"
#include "stm32f0xx_ll_rcc.h"

void UART_Config(void);

void UART_Transmit(const char *data, uint32_t size);

unsigned char UART_Receive(void);

void initUART(void);