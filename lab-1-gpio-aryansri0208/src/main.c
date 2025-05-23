/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 5 2024
  * @brief   ECE 362 Lab 1 template
  ******************************************************************************
*/


/**
******************************************************************************/

// Fill out your username, otherwise your completion code will have the 
// wrong username!
const char* username = "sriva115";

/******************************************************************************
*/ 

#include "stm32f0xx.h"
#include <stdint.h>

void initb();
void initc();
void setn(int32_t pin_num, int32_t val);
int32_t readpin(int32_t pin_num);
void buttons(void);
void keypad(void);
void autotest(void);
extern void internal_clock(void);
extern void nano_wait(unsigned int n);

int main(void) {
    internal_clock(); // do not comment!
    // Comment until most things have been implemented
    autotest();
    initb();
    initc();

    // uncomment one of the loops, below, when ready
    while(1) {
      buttons();
    }

    while(1) {
      keypad();
    }

    for(;;);
    
    return 0;
}

/**
 * @brief Init GPIO port B
 *        Pin 0: input
 *        Pin 4: input
 *        Pin 8-11: output
 *
 */
void initb() {
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
  GPIOB->MODER &= ~((3U << (8 * 2)) | (3U << (9 * 2)) | (3U << (10 * 2)) | (3U << (11 * 2)));
  GPIOB->MODER |= ((1U << (8 * 2)) | (1U << (9 * 2)) | (1U << (10 * 2)) | (1U << (11 * 2)));
  GPIOB->MODER &= ~((3U << (0 * 2)) | (3U << (4 * 2)));
}

/**
 * @brief Init GPIO port C
 *        Pin 0-3: inputs with internal pull down resistors
 *        Pin 4-7: outputs
 *
 */
void initc() {
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
  GPIOC->MODER &= ~((3U << (4 * 2)) | (3U << (5 * 2)) | (3U << (6 * 2)) | (3U << (7 * 2)));
  GPIOC->MODER |= ((1U << (4 * 2)) | (1U << (5 * 2)) | (1U << (6 * 2)) | (1U << (7 * 2)));
  GPIOC->MODER &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (2 * 2)) | (3U << (3 * 2)));
  GPIOC->PUPDR &= ~((3U << (0 * 2)) | (3U << (1 * 2)) | (3U << (2 * 2)) | (3U << (3 * 2)));
  GPIOC->PUPDR |= ((2U << (0 * 2)) | (2U << (1 * 2)) | (2U << (2 * 2)) | (2U << (3 * 2)));
}

/**
 * @brief Set GPIO port B pin to some value
 *
 * @param pin_num: Pin number in GPIO B
 * @param val    : Pin value, if 0 then the
 *                 pin is set low, else set high
 */
void setn(int32_t pin_num, int32_t val) {
  if (val == 0) { 
    GPIOB->BSRR = (1U << (pin_num + 16));
  } else {
    GPIOB->BSRR = (1U << pin_num);
  }
}

/**
 * @brief Read GPIO port B pin values
 *
 * @param pin_num   : Pin number in GPIO B to be read
 * @return int32_t  : 1: the pin is high; 0: the pin is low
 */
int32_t readpin(int32_t pin_num) {
  if (GPIOB->IDR & (1U << pin_num)) {
    return 0x1;
  } else {
    return 0x0;
  }
}

/**
 * @brief Control LEDs with buttons
 *        Use PB0 value for PB8
 *        Use PB4 value for PB9
 *
 */
void buttons(void) {
  int32_t pb0_value = readpin(0); 
  setn(8, pb0_value);
  int32_t pb4_value = readpin(4);
  setn(9, pb4_value);
}

/**
 * @brief Control LEDs with keypad
 * 
 */

void keypad(void) {
  int x[4] = {8, 9, 10, 11}; 
  int y[4] = {4, 5, 6, 7};   
  int z = 0xF;             
  for (int i = 0; i < 4; i++) {
    GPIOC->ODR = (1U << y[i]);
    nano_wait(1000000);
    int row_values = GPIOC->IDR & z;
    if (row_values & (1U << i)) {
      setn(x[i], 1);
    } else {
      setn(x[i], 0);
    }
  }
}

