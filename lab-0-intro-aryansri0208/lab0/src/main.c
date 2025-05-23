#include "stm32f0xx.h"
extern void autotest();
extern void internal_clock();
void setup_serial(void)
{
    RCC->AHBENR |= 0x00180000;
    GPIOC->MODER  |= 0x02000000;
    GPIOC->AFR[1] |= 0x00020000;
    GPIOD->MODER  |= 0x00000020;
    GPIOD->AFR[0] |= 0x00000200;
    RCC->APB1ENR |= 0x00100000;
    USART5->CR1 &= ~0x00000001;
    USART5->CR1 |= 0x00008000;
    USART5->BRR = 0x340;
    USART5->CR1 |= 0x0000000c;
    USART5->CR1 |= 0x00000001;
}
int main(void)
{
    internal_clock();   // Never comment this out!

    autotest();         // Only comment this out when you are done testing.
    
    setup_serial();
    while(1) {
        if ((USART5->ISR & USART_ISR_RXNE))  // if receive buffer has some data in it
            USART5->TDR = USART5->RDR;       // copy that data into transmit buffer.
    }
}