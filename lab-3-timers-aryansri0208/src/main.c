/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An, Niraj Menon
  * @date    Jan 19, 2024
  * @brief   ECE 362 Lab 3 Student template
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
#include <stdio.h>
#include <stdint.h>

// Global data structure
char disp[9]         = "Hello...";
uint8_t col          = 0;
uint8_t mode         = 'A';
uint8_t thrust       = 0;
int16_t fuel         = 800;
int16_t alt          = 4500;
int16_t velo         = 0;

// Keymap is in `font.S` to match up what autotester expected
extern char keymap;
extern char disp[9];
extern uint8_t col;
extern uint8_t mode;
extern uint8_t thrust;
extern int16_t fuel;
extern int16_t alt;
extern int16_t velo;

// Make it easier to access keymap
char* keymap_arr = &keymap;

// Font array in assembly file
// as I am too lazy to convert it into C array
extern uint8_t font[];

// The functions we should implement
void enable_ports();
void show_char(int n, char c);
void drive_column(int c);
int read_rows();
char rows_to_key(int rows);
void handle_key(char key);
void setup_tim7();
void write_display();
void update_variables();
void setup_tim14();

// Auotest functions
void internal_clock();
extern void check_wiring();
extern void autotest();
extern void fill_alpha();

int main(void) {
    internal_clock();

    // Uncomment when you are ready to test wiring.
    //check_wiring();
    
    // Uncomment when you are ready to test everything.
    autotest();
    
    enable_ports();
    // Comment out once you are checked off for fill_alpha
    // fill_alpha();

    setup_tim7();
    setup_tim14();

    for(;;) {
        // enter low power sleep mode 
        // and Wait For Interrupt (WFI)
        asm("wfi");
    }
}

/**
 * @brief Enable the ports and configure pins as described
 *        in lab handout
 * 
 */
void enable_ports() {
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN;
  GPIOB->MODER &= ~(0x3FFFFF);   
  GPIOB->MODER |= (0x155555);   

  GPIOC->MODER &= ~(0xFF << (4 * 2)); 
  GPIOC->MODER |= (0x55 << (4 * 2));  

  GPIOC->MODER &= ~(0xFF);  

  GPIOC->PUPDR &= ~(0xFF);  
  GPIOC->PUPDR |= (0xAA);   
}

/**
 * @brief Show a character `c` on column `n`
 *        of the segment LED display
 * 
 * @param n 
 * @param c 
 */
void show_char(int n, char c) {
  if (n < 0 || n > 7) return;
  
  uint8_t pattern = font[(uint8_t)c];
  
  GPIOB->BSRR = (0x7FF << 16) | pattern;
  
  GPIOB->BSRR = ((0x7 << 8) << 16) | (n << 8);
}

/**
 * @brief Drive the column pins of the keypad
 *        First clear the keypad column output
 *        Then drive the column represented by `c`
 * 
 * @param c 
 */
void drive_column(int c) {
  c &= 0x3;
  GPIOC->BSRR = (0xF << 4) << 16;
  GPIOC->BSRR = (1 << (c + 4));
}

/**
 * @brief Read the rows value of the keypad
 * 
 * @return int 
 */
int read_rows() {
  return (GPIOC->IDR & 0xF);
}

/**
 * @brief Convert the pressed key to character
 *        Use the rows value and the current `col`
 *        being scanning to compute an offset into
 *        the character map array
 * 
 * @param rows 
 * @return char 
 */
char rows_to_key(int rows) {
  if (rows == 0) return 0;
  int row_index = 0;
  while ((rows & 1) == 0) {
      rows >>= 1;
      row_index++;
  } 

  int column = col & 0x3;
  int offset = (column * 4) + row_index;

  return keymap_arr[offset]; 
}

/**
 * @brief Handle key pressed in the game
 * 
 * @param key 
 */
void handle_key(char key) {
  if (key == 'A' || key == 'B' || key == 'D') {
      mode = key;
  } else if (key >= '0' && key <= '9') {
      thrust = key - '0';
  }
}

//-------------------------------
// Timer 7 ISR goes here
//-------------------------------
// TODO
void TIM7_IRQHandler() {
  TIM7->SR &= ~TIM_SR_UIF;
  
  int rows = read_rows();
  if (rows != 0) {
      char key = rows_to_key(rows);
      handle_key(key);
  }
  
  show_char(col, disp[col]);
  
  col = (col + 1) & 0x7;
  
  drive_column(col);
}

/**
 * @brief Setup timer 7 as described in lab handout
 * 
 */
void setup_tim7() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;
  TIM7->PSC = 47; // Prescaler to divide 48MHz by 48
  TIM7->ARR = 999; // Auto-reload to generate interrupt every 1ms
  TIM7->DIER |= TIM_DIER_UIE;
  NVIC->ISER[0] |= (1 << TIM7_IRQn);
  TIM7->CR1 |= TIM_CR1_CEN;
}


/**
 * @brief Write the display based on game's mode
 * 
 */
void write_display() {
  if (mode == 'C') {
      snprintf(disp, sizeof(disp), "Crashed");
  } else if (mode == 'L') {
      snprintf(disp, sizeof(disp), "Landed ");
  } else if (mode == 'A') {
      snprintf(disp, sizeof(disp), "ALt%5d", alt);
  } else if (mode == 'B') {
      snprintf(disp, sizeof(disp), "FUEL %3d", fuel);
  } else if (mode == 'D') {
      snprintf(disp, sizeof(disp), "Spd %4d", velo);
  }
}

/**
 * @brief Game logic
 * 
 */
void update_variables() {
  fuel -= thrust;
  if (fuel <= 0) {
      fuel = 0;
      thrust = 0;
  }
  
  alt += velo;
  if (alt <= 0) {
      if (-velo < 10) {
          mode = 'L';
      } else {
          mode = 'C';
      }
      return;
  }
  
  velo += thrust - 5;
}


//-------------------------------
// Timer 14 ISR goes here
//-------------------------------
// TODO
void TIM14_IRQHandler() {
  TIM14->SR &= ~TIM_SR_UIF;
  update_variables();
  write_display();
}


/**
 * @brief Setup timer 14 as described in lab
 *        handout
 * 
 */
void setup_tim14() {
  RCC->APB1ENR |= RCC_APB1ENR_TIM14EN;
  TIM14->PSC = 23999; 
  TIM14->ARR = 999; 
  TIM14->DIER |= TIM_DIER_UIE;
  NVIC->ISER[0] |= (1 << TIM14_IRQn);
  TIM14->CR1 |= TIM_CR1_CEN;
}

