#include "uart.h"


void UART_Config(void) {
    LL_USART_SetTransferDirection(USART1, LL_USART_DIRECTION_TX_RX);
    LL_USART_ConfigCharacter(USART1, LL_USART_DATAWIDTH_8B, LL_USART_PARITY_NONE, LL_USART_STOPBITS_1);
    LL_USART_SetBaudRate(USART1,SystemCoreClock,LL_USART_OVERSAMPLING_16, 115200);
    LL_RCC_SetUSARTClockSource(LL_RCC_USART1_CLKSOURCE_SYSCLK);
    LL_USART_Enable(USART1);

    while ((!(LL_USART_IsActiveFlag_TEACK(USART1))) || !(LL_USART_IsActiveFlag_REACK(USART1)));
}

void UART_Transmit(const char *data, uint32_t size) {
    for (uint32_t i = 0; i < size; i++) {
        while (!LL_USART_IsActiveFlag_TXE(USART1));
        LL_USART_TransmitData8(USART1,data[i]);
        
    };
    while(!LL_USART_IsActiveFlag_TC(USART1));
}

unsigned char UART_Receive(void) {
    while(!LL_USART_IsActiveFlag_RXNE(USART1));
    return LL_USART_ReceiveData8(USART1);
}

void initUART(void) {
    LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_USART1);
    UART_Config();
}