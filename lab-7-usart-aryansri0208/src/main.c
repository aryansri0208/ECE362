/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Feb 7, 2024
  * @brief   ECE 362 Lab 7 student template
  ******************************************************************************
*/

/*******************************************************************************/

// Fill out your username!  Even though we're not using an autotest, 
// it should be a habit to fill out your username in this field now.
const char* username = "sriva115";

/*******************************************************************************/ 

#include "stm32f0xx.h"
#include <stdint.h>

#define FIFOSIZE 16
char serfifo[FIFOSIZE];
int seroffset = 0;
void internal_clock();


#define SD_CS_PIN  2    // PB2
#define LCD_CS_PIN 8    // PB8
#define LCD_DC_PIN 14   // PB14
#define LCD_RST_PIN 11  // PB11


void init_spi1_slow() {
    // Enable clock for GPIOB and SPI1
    RCC->AHBENR  |= RCC_AHBENR_GPIOBEN;
    RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

    // PB3 (SCK), PB4 (MISO), PB5 (MOSI) => Alternate Function
    GPIOB->MODER &= ~(GPIO_MODER_MODER3 | GPIO_MODER_MODER4 | GPIO_MODER_MODER5);
    GPIOB->MODER |=  (GPIO_MODER_MODER3_1 | GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1);

    // AF0 for SPI1 (the default), so clear AFR bits
    GPIOB->AFR[0] &= ~((0xF << (4*3)) | (0xF << (4*4)) | (0xF << (4*5)));

    // Configure SPI1
    SPI1->CR1 = 0;
    // Master, Software NSS, Internal slave select
    SPI1->CR1 |= SPI_CR1_MSTR | SPI_CR1_SSM | SPI_CR1_SSI;
    // Slowest baud rate => fPCLK/256 => BR[2:0] = 111
    SPI1->CR1 |= (SPI_CR1_BR_2 | SPI_CR1_BR_1 | SPI_CR1_BR_0);
    // 8-bit data => FRXTH in CR2
    SPI1->CR2 = SPI_CR2_FRXTH;
    // Enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;
}


// Uncomment only one of the following to test each step
//#define STEP1
//#define STEP2
//#define STEP3
#define STEP4

void init_usart5() {
    // TODO
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    RCC->AHBENR |= RCC_AHBENR_GPIODEN;
    RCC->APB1ENR |= RCC_APB1ENR_USART5EN;

    GPIOC->MODER &= ~GPIO_MODER_MODER12;
    GPIOC->MODER |= GPIO_MODER_MODER12_1;
    GPIOC->AFR[1] |= (2 << (4 * (12 - 8)));

    GPIOD->MODER &= ~GPIO_MODER_MODER2;
    GPIOD->MODER |= GPIO_MODER_MODER2_1;
    GPIOD->AFR[0] |= (2 << (4 * 2));

    USART5->CR1 &= ~USART_CR1_UE;
    USART5->CR1 &= ~USART_CR1_M;
    USART5->CR2 &= ~USART_CR2_STOP;
    USART5->CR1 &= ~USART_CR1_PCE;
    USART5->CR1 &= ~USART_CR1_OVER8;
    USART5->BRR = 48000000 / 115200;
    USART5->CR1 |= USART_CR1_TE | USART_CR1_RE;
    USART5->CR1 |= USART_CR1_UE;

    while (!(USART5->ISR & USART_ISR_TEACK));
    while (!(USART5->ISR & USART_ISR_REACK));
}

void enable_sdcard() {
    // PB2 low => SD card enabled
    GPIOB->ODR &= ~(1 << SD_CS_PIN);
}

void disable_sdcard() {
    // PB2 high => SD card disabled
    GPIOB->ODR |= (1 << SD_CS_PIN);
}

void init_sdcard_io() {
    // PB2 for SD Card CS => output
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER2);
    GPIOB->MODER |=  (GPIO_MODER_MODER2_0);

    // Start SPI slow
    init_spi1_slow();

    // Keep card disabled by default
    disable_sdcard();
}

void sdcard_io_high_speed() {
    // 1) Disable SPI
    SPI1->CR1 &= ~SPI_CR1_SPE;

    // 2) Set Baud = fPCLK / 4 => BR = 0b010
    // Clear old BR bits
    SPI1->CR1 &= ~(SPI_CR1_BR);
    SPI1->CR1 |= (SPI_CR1_BR_1);  // 0b010 => /4 => ~12 MHz if PCLK=48MHz

    // 3) Re-enable SPI
    SPI1->CR1 |= SPI_CR1_SPE;
}


void init_lcd_spi() {
    // 1) PB8, PB11, PB14 => outputs
    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
    GPIOB->MODER &= ~(GPIO_MODER_MODER8  | 
                      GPIO_MODER_MODER11 | 
                      GPIO_MODER_MODER14);
    GPIOB->MODER |=  (GPIO_MODER_MODER8_0  | 
                      GPIO_MODER_MODER11_0 | 
                      GPIO_MODER_MODER14_0);

    // Optional: set idle states
    GPIOB->ODR |= (1 << LCD_CS_PIN);  // CS high
    GPIOB->ODR |= (1 << LCD_RST_PIN); // RST high
    GPIOB->ODR |= (1 << LCD_DC_PIN);  // D/C high

    // 2) SPI1 slow
    init_spi1_slow();

    // 3) Switch SPI1 to higher speed for LCD
    sdcard_io_high_speed();
}


#ifdef STEP1
int main(void){
    internal_clock();
    init_usart5();
    for(;;) {
        while (!(USART5->ISR & USART_ISR_RXNE)) { }
        char c = USART5->RDR;
        while(!(USART5->ISR & USART_ISR_TXE)) { }
        USART5->TDR = c;
    }
}
#endif

#ifdef STEP2
#include <stdio.h>

// TODO Resolve the echo and carriage-return problem

int __io_putchar(int c) {
    if (c == '\n') {
        while (!(USART5->ISR & USART_ISR_TXE));
        USART5->TDR = '\r';
    }
    while (!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
	//Step 2
    while (!(USART5->ISR & USART_ISR_RXNE));
    char c = USART5->RDR;
    if (c == '\r') {
        c = '\n';
    }
    __io_putchar(c);
    return c;

	//Step 3
	//return line_buffer_getchar();
}

int main() {
    internal_clock();
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP3
#include <stdio.h>
#include "fifo.h"
#include "tty.h"
int __io_putchar(int c) {
    if (c == '\n') {
        while (!(USART5->ISR & USART_ISR_TXE));
        USART5->TDR = '\r';
    }
    while (!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}


int __io_getchar(void) {
    // TODO
    return line_buffer_getchar();
}

int main() {
    internal_clock();
    init_usart5();
    setbuf(stdin,0);
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: ");
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n");
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif

#ifdef STEP4

#include <stdio.h>
#include "fifo.h"
#include "tty.h"

// TODO DMA data structures

void enable_tty_interrupt(void) {
    // TODO
    RCC->AHBENR |= RCC_AHBENR_DMA2EN;

    USART5->CR1 |= USART_CR1_RXNEIE;
    USART5->CR3 |= USART_CR3_DMAR;
    NVIC_EnableIRQ(USART3_8_IRQn);

    DMA2->CSELR |= DMA2_CSELR_CH2_USART5_RX;
    DMA2_Channel2->CCR &= ~DMA_CCR_EN;

    DMA2_Channel2->CMAR = (uint32_t)serfifo;
    DMA2_Channel2->CPAR = (uint32_t)&USART5->RDR;
    DMA2_Channel2->CNDTR = FIFOSIZE;

    DMA2_Channel2->CCR = DMA_CCR_MINC | DMA_CCR_CIRC | DMA_CCR_PL | DMA_CCR_EN;
}

// Works like line_buffer_getchar(), but does not check or clear ORE nor wait on new characters in USART
char interrupt_getchar() {
    // TODO
    while (!fifo_newline(&input_fifo)) {
        asm volatile ("wfi");
    }
    return fifo_remove(&input_fifo);
}

int __io_putchar(int c) {
    if (c == '\n') {
        while (!(USART5->ISR & USART_ISR_TXE));
        USART5->TDR = '\r';
    }
    while (!(USART5->ISR & USART_ISR_TXE));
    USART5->TDR = c;
    return c;
}

int __io_getchar(void) {
    // TODO Use interrupt_getchar() instead of line_buffer_getchar()
    return interrupt_getchar();
}

// TODO Copy the content for the USART5 ISR here
void USART3_8_IRQHandler(void) {
    while (DMA2_Channel2->CNDTR != (FIFOSIZE - seroffset)) {
        if (!fifo_full(&input_fifo)) {
            insert_echo_char(serfifo[seroffset]);
        }
        seroffset = (seroffset + 1) % FIFOSIZE;
    }
}
// TODO Remember to look up for the proper name of the ISR function

int main() {
    internal_clock();
    init_usart5();
    enable_tty_interrupt();

    setbuf(stdin,0); // These turn off buffering; more efficient, but makes it hard to explain why first 1023 characters not dispalyed
    setbuf(stdout,0);
    setbuf(stderr,0);
    printf("Enter your name: "); // Types name but shouldn't echo the characters; USE CTRL-J to finish
    char name[80];
    fgets(name, 80, stdin);
    printf("Your name is %s", name);
    printf("Type any characters.\n"); // After, will type TWO instead of ONE
    for(;;) {
        char c = getchar();
        putchar(c);
    }
}
#endif