/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Nov 15, 2022
  * @brief   ECE 362 Lab 10 Student template
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Be sure to change this to your login...
const char login[] = "xyz";

void set_char_msg(int, char);
void nano_wait(unsigned int);

char* count_oled();


//===========================================================================
// Configure GPIOC
//===========================================================================
void enable_ports(void)
{
    // Only enable port C for the keypad. Given.
    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
    GPIOC->MODER &= ~0xffff;
    GPIOC->MODER |= 0x55 << (4*2);
    GPIOC->OTYPER &= ~0xff;
    GPIOC->OTYPER |= 0xf0;
    GPIOC->PUPDR &= ~0xff;
    GPIOC->PUPDR |= 0x55;
}

uint8_t col; // the column being scanned

void drive_column(int);   // energize one of the column outputs
int  read_rows();         // read the four row inputs
void update_history(int col, int rows); // record the buttons of the driven column
char get_key_event(void); // wait for a button event (press or release)
char get_keypress(void);  // wait for only a button press event.
float getfloat(void);     // read a floating-point number from keypad
void show_keys(void);     // demonstrate get_key_event()

//===========================================================================
// Configure timer 7 to invoke the update interrupt at 1kHz
// Copy from lab 8 or 9.
//===========================================================================
void init_tim7()
{
  // Turn on clock, can write
  // 0x20 instead
  RCC->APB1ENR |= RCC_APB1ENR_TIM7EN;

  // Configure to run at 1kHz
  TIM7->PSC = 48-1;
  TIM7->ARR = 1000-1;

  // Enable the interrupt,
  // can write 0x01 instead
  TIM7->DIER |= TIM_DIER_UIE;

  // Unmask the interrupt
  NVIC->ISER[0] |= 1<<TIM7_IRQn;

  // Enable timer, can write
  // 0x01 instead
  TIM7->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// Copy the Timer 7 ISR from lab 9
//===========================================================================
// TODO To be copied
void TIM7_IRQHandler(void)
{
  // Code is given in lab 9
  TIM7->SR &= ~TIM_SR_UIF;
  int rows = read_rows();
  update_history(col, rows);
  col = (col + 1) & 3;
  drive_column(col);
}

//===========================================================================
// 4.1 Bit Bang SPI LED Array
//===========================================================================
// Given
int msg_index = 0;
uint16_t msg[8] = { 0x0000,0x0100,0x0200,0x0300,0x0400,0x0500,0x0600,0x0700 };
extern const char font[];

//===========================================================================
// Configure PB12 (NSS), PB13 (SCK), and PB15 (MOSI) for outputs
//===========================================================================
void setup_bb(void)
{
  // Turn on GPIOB
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // Set 12, 13, 15 to output mode,
  // Can also clear out and write in
  // GPIO_MODER_MODER15_0 |
  // GPIO_MODER_MODER13_0 |
  // GPIO_MODER_MODER12_0 instead.
  // Should come out to 0x45000000 at
  // the end either way.
  GPIOB->MODER &= ~0xcf000000;
  GPIOB->MODER |=  0x45000000;

  // Force PB12 high, force PB13
  // and PB15 low. PB15 does not
  // "need" to be low,
  // but it's good practice. They
  // can also write 0xa0001000 to
  // this to get the
  // same effect.
  GPIOB->BSRR |= GPIO_BSRR_BS_12 | GPIO_BSRR_BR_13 | GPIO_BSRR_BR_15;


  // **** NOTE ****
  // I'm going to use the BSRR for
  //all of these operations. This is
  // the "better practice" way to do
  // it, but they can also use the ODR.
  // In that case, the hex codes will
  // come out very similarly, just
  // shifted down 16 bits in the
  // case of bit reset operations.
  // For example, GPIO->BSRR |= 0x100000000; is
  // Equivalent to GPIO->ODER &- ~0x1000;
  // **** END ****
}

void small_delay(void)
{
  // Given.
  nano_wait(50000);
}

//===========================================================================
// Set the MOSI bit, then set the clock high and low.
// Pause between doing these steps with small_delay().
//===========================================================================
void bb_write_bit(int val)
{
  // NSS (PB12)
  // SCK (PB13)
  // MOSI (PB15)

  // If value is zero, force PB15 low.
  // If value is nonzero, force PB15 high.

  if(val == 0)
  {
    // Can also write in 0x80000000
    GPIOB->BSRR |= GPIO_BSRR_BR_15;
  }
  else
  {
    // Can also write in 0x00008000
    GPIOB->BSRR |= GPIO_BSRR_BS_15;
  }

  // Force PB13 high, wait, force PB13 low,
  // wait. Can also use
  // 0x00002000 and 0x20000000
  GPIOB->BSRR |= GPIO_BSRR_BS_13;
  small_delay();
  GPIOB->BSRR |= GPIO_BSRR_BR_13;
  small_delay();

}

//===========================================================================
// Set NSS (PB12) low,
// write 16 bits using bb_write_bit,
// then set NSS high.
//===========================================================================
void bb_write_halfword(int halfword)
{
  // They can do this two ways. Shift
  // right and count down, or shift left
  // and count up. The lab does tell them
  // to shift right, but it works either way.
  // For this example, I shift right. If
  // they shift left, change i to 0
  // and do i++ instead of i--, then
  // AND with 0x8000000 instead of 0x1.

  // Start by forcing PB12 low. Can also use
  // 0x100000000
  GPIOB->BSRR |= GPIO_BSRR_BR_12;

  // For loop that goes through the half word.
  for(int i = 15 ; i >=0 ; i--)
  {
      // Shift by (15-loop_iteration), take
      // leftover bit into function
      bb_write_bit(((halfword>>i) & 0x1));
  }

  // Force PB12 high again. Can also use
  // 0x00001000
  GPIOB->BSRR |= GPIO_BSRR_BS_12;
}

//===========================================================================
// Continually bitbang the msg[] array.
//===========================================================================
void drive_bb(void)
{
  // Given.
    for(;;)
        for(int d=0; d<8; d++) {
            bb_write_halfword(msg[d]);
            nano_wait(1000000); // wait 1 ms between digits
        }
}

//============================================================================
// setup_dma()
// Copy this from lab 8 or lab 9.
// Write to SPI2->DR instead of GPIOB->ODR.
//============================================================================
void setup_dma(void)
{
  // Turn on DMA1
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  // Configure DMA. They should be using channel 5.

  // Turn off DMA
  // CPAR to SPI2 DR adr
  // CMAR to msg adr
  // 8 units in array
  // mem to per direction
  // Increment mem adr
  // no inc per adr
  // Set data size to 16b
  // Operate circularly
  DMA1_Channel5->CCR &= ~DMA_CCR_EN;
  DMA1_Channel5->CPAR = (uint32_t) &(SPI2->DR);
  DMA1_Channel5->CMAR = (uint32_t) &(msg);
  DMA1_Channel5->CNDTR = 8;
  DMA1_Channel5->CCR |= DMA_CCR_DIR;
  DMA1_Channel5->CCR |= DMA_CCR_MINC;
  DMA1_Channel5->CCR &= ~DMA_CCR_PINC;
  DMA1_Channel5->CCR |= 0x0500;
  DMA1_Channel5->CCR |= DMA_CCR_CIRC;

  //**** NOTE ****
  // After this task, these hex codes should be visible if you put
  // in a breakpoint:
  // CCR  -> 0x05b0
  // CPAR -> 0x4000380C
  // CNDTR-> 0x00008
  // CMAR -> Varies depending on how the compiler organizes it. No set address.
  //         Should be in the 0x20000000 range though, because that's where
  //         the memory is.
  //**** END ****
}

//============================================================================
// enable_dma()
// Copy this from lab 8 or lab 9.
//============================================================================
void enable_dma(void)
{
  // Enable the DMA.
  DMA1_Channel5->CCR |= DMA_CCR_EN;

  //**** NOTE ****
  // After this task, the CCR hex code
  // should change to 0x05b1 if you put a
  // breakpoint here.
  //**** END ****
}

//============================================================================
// Configure Timer 15 for an update rate of 1 kHz.
// Trigger the DMA channel on each update.
// Copy this from lab 8 or lab 9.
//============================================================================
void init_tim15(void)
{
  // Turn timer's clock on,
  // can write 0x0001000 instead
  RCC->APB2ENR |= RCC_APB2ENR_TIM15EN;

  // Turn timer off, can write
  // 0x0001 instead.
  TIM15->CR1 &= ~TIM_CR1_CEN;

  // Configure PSC and ARR to
  // 1kHz
  TIM15->PSC = 48-1;
  TIM15->ARR = 1000-1;

  // Trigger the DMA transfer,
  // can write 0x0100 instead.
  TIM15->DIER |= TIM_DIER_UDE;

  // Turn timer on, can write
  // 0x0001 instead.
  TIM15->CR1 |= TIM_CR1_CEN;
}

//===========================================================================
// Initialize the SPI2 peripheral.
//===========================================================================
void init_spi2(void)
{
  // Enable port B, can write
  // 0x00004000 instead
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN;

  // Set PB12, 13, 15 into AF mode.
  // Like earlier, they can use the
  // CMSIS calls.
  GPIOB->MODER &= ~0xcf000000;
  GPIOB->MODER |=  0x8a000000;

  // These pins default alternate
  // functions are the SPI function.
  // However, they can clear them out
  // if they want. It doesn't make a
  // difference.
  // These CMSIS calls look like
  // (GPIO_AFRH_AFR12 | GPIO_AFRH_AFR13
  // | GPIO_AFRH_AFR15).
  GPIOB->AFR[1] &= ~0xf0ff0000;

  // Turn on RCC clock,
  // can write 0x00004000 instead.
  RCC->APB1ENR |= RCC_APB1ENR_SPI2EN;

  // Turn off the channel
  // for configuration, can write 0x0040
  SPI2->CR1 &= ~SPI_CR1_SPE;

  // Turn baud rate to lowest,
  // can write 0x0038 instead
  SPI2->CR1 |= SPI_CR1_BR;

  // Set data size to 16 bit,
  // can write 0x0f00 instead
  SPI2->CR2 |= SPI_CR2_DS;

  // Turn on master mode,
  // can write 0x0004 instead
  SPI2->CR1 |= SPI_CR1_MSTR;

  // Enable SSOE and NSSP,
  // can write 0x000c instead
  SPI2->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;

  // Trigger a DMA transfer,
  // can write 0x0002 instead
  SPI2->CR2 |= SPI_CR2_TXDMAEN;

  // Turn the spi channel back on,
  // can write 0x0040 instead
  SPI2->CR1 |= SPI_CR1_SPE;

  //**** NOTE ****
  // After this block, if you put a
  // breakpoint here the registers should look
  // like this:
  // CR1 -> 0x007c
  // CR2 -> 0x0f0e
  //**** END ****
}

//===========================================================================
// Configure the SPI2 peripheral to trigger the DMA channel when the
// transmitter is empty.
//===========================================================================
void spi2_setup_dma(void)
{
  // Given.
  setup_dma();
  SPI2->CR2 |= SPI_CR2_TXDMAEN; // Transfer register empty DMA enable
}

//===========================================================================
// Enable the DMA channel.
//===========================================================================
void spi2_enable_dma(void)
{
  // Given.
  enable_dma();
}

//===========================================================================
// 4.4 SPI OLED Display
//===========================================================================
void init_spi1() {
  // PA5  SPI1_SCK
  // PA6  SPI1_MISO
  // PA7  SPI1_MOSI
  // PA15 SPI1_NSS

  // Enable port B, can write
  // 0x00004000 instead
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;

  // Set PA5, 6, 7, 15 into AF mode.
  // Like earlier, they can use the
  // CMSIS calls.
  GPIOA->MODER &= ~0xc000fc00;
  GPIOA->MODER |=  0x8a00a800;

  // These pins default alternate
  // functions are the SPI function.
  // However, they can clear them out
  // if they want. It doesn't make a
  // difference.
  // Like earlier, they can use the
  // same CMSIS calls.
  GPIOA->AFR[0] &= ~0xfff00000;
  GPIOA->AFR[1] &= ~0xf0000000;

  // Turn on RCC clock, can write
  // 0x00001000 instead.
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  // Turn off the channel for
  // configuration, can write 0x0040
  SPI1->CR1 &= ~SPI_CR1_SPE;

  // Turn baud rate to lowest,
  // can write 0x0038 instead
  SPI1->CR1 |= SPI_CR1_BR;

  // Set data size to 10 bit. Pay careful
  // attention to how they do it, as
  // they cannot write 0x0900 or clear
  // 0x0f00 since the chip is finnicky
  // with it's default here. They need
  // to write 0x0900 and THEN clear 0x0600.
  // It's possible they're using the CMSIS
  // identities, which looks like
  // ORRING (SPI_CR2_DS_3 | SPI_CR2_DS_0) and
  // clearing (SPI_CR2_DS_1 | SPI_CR2_DS_2).
  SPI1->CR2 |= 0x0900;
  SPI1->CR2 &= ~0x0600;

  // Turn on master mode, can write
  // 0x0004 instead
  SPI1->CR1 |= SPI_CR1_MSTR;

  // Enable SSOE and NSSP, can write
  // 0x000c instead
  SPI1->CR2 |= SPI_CR2_SSOE | SPI_CR2_NSSP;

  // Trigger a DMA transfer, can write
  // 0x0002 instead
  SPI1->CR2 |= SPI_CR2_TXDMAEN;

  // Turn the spi channel back on,
  // can write 0x0040 instead
  SPI1->CR1 |= SPI_CR1_SPE;

  //**** NOTE ****
  // After this block, if you put a
  // breakpoint here the registers should look
  // like this:
  // CR1 -> 0x007c
  // CR2 -> 0x090e
  //**** END ****

}

void spi_cmd(unsigned int data)
{
  // Given.
  while(!(SPI1->SR & SPI_SR_TXE)) {}
  SPI1->DR = data;
}
void spi_data(unsigned int data)
{
  // Given.
  spi_cmd(data | 0x200);
}
void spi1_init_oled()
{
  // Given.
  nano_wait(1000000);
  spi_cmd(0x38);
  spi_cmd(0x08);
  spi_cmd(0x01);
  nano_wait(2000000);
  spi_cmd(0x06);
  spi_cmd(0x02);
  spi_cmd(0x0c);
}
void spi1_display1(const char *string)
{
  // Given.
  spi_cmd(0x02);
  while(*string != '\0')
  {
      spi_data(*string);
      string++;
  }
}
void spi1_display2(const char *string)
{
  // Given.
  spi_cmd(0xc0);
  while(*string != '\0')
  {
      spi_data(*string);
      string++;
  }
}

//===========================================================================
// This is the 34-entry buffer to be copied into SPI1.
// Each element is a 16-bit value that is either character data or a command.
// Element 0 is the command to set the cursor to the first position of line 1.
// The next 16 elements are 16 characters.
// Element 17 is the command to set the cursor to the first position of line 2.
//===========================================================================
uint16_t display[34] = {
        0x002, // Command to set the cursor at the first position line 1
        0x200+'E', 0x200+'C', 0x200+'E', 0x200+'3', 0x200+'6', + 0x200+'2', 0x200+' ', 0x200+'i',
        0x200+'s', 0x200+' ', 0x200+'t', 0x200+'h', + 0x200+'e', 0x200+' ', 0x200+' ', 0x200+' ',
        0x0c0, // Command to set the cursor at the first position line 2
        0x200+'c', 0x200+'l', 0x200+'a', 0x200+'s', 0x200+'s', + 0x200+' ', 0x200+'f', 0x200+'o',
        0x200+'r', 0x200+' ', 0x200+'y', 0x200+'o', + 0x200+'u', 0x200+'!', 0x200+' ', 0x200+' ',
};

//===========================================================================
// Configure the proper DMA channel to be triggered by SPI1_TX.
// Set the SPI1 peripheral to trigger a DMA when the transmitter is empty.
//===========================================================================
void spi1_setup_dma(void)
{
  // Turn on DMA1
  RCC->AHBENR |= RCC_AHBENR_DMA1EN;

  // Turn off DMA
  // CPAR to SPI1 DR adr
  // CMAR to msg adr
  // 34 units in array
  // mem to per direction
  // Increment mem adr
  // no inc per adr
  // Set data size to 16b
  // Operate circularly
  // Configure DMA. They should be using channel 3.
  DMA1_Channel3->CCR &= ~DMA_CCR_EN;
  DMA1_Channel3->CPAR = (uint32_t) &(SPI1->DR);
  DMA1_Channel3->CMAR = (uint32_t) &(display);
  DMA1_Channel3->CNDTR = 34;
  DMA1_Channel3->CCR |= DMA_CCR_DIR;
  DMA1_Channel3->CCR |= DMA_CCR_MINC;
  DMA1_Channel3->CCR &= ~DMA_CCR_PINC;
  DMA1_Channel3->CCR |= 0x0500;
  DMA1_Channel3->CCR |= DMA_CCR_CIRC;

  //**** NOTE ****
  // After this task, these
  // hex codes should be visible if you put
  // in a breakpoint:
  // CCR  -> 0x05b0
  // CPAR -> 0x4001300C
  // CNDTR-> 0x00034
  // CMAR -> Varies depending on how the
  //         compiler organizes it. No set
  //         address. Should be in the
  //         0x20000000 range though, because
  //         that's where the memory is.
  //**** END ****
}

//===========================================================================
// Enable the DMA channel triggered by SPI1_TX.
//===========================================================================
void spi1_enable_dma(void)
{
  // Enable the DMA.
  DMA1_Channel3->CCR |= DMA_CCR_EN;

  //**** NOTE ****
  // After this task, the CCR hex code
  // should change to 0x05b1 if you put a
  // breakpoint here.
  //**** END ****
}


char* count_oled(char* string, int* count) {
		nano_wait(50000000);
		int num = atoi(string);

		if(num < 100) {
			num++;

			sprintf(string, "%d", num);
			(*count)++;
			return string;
		}
		else {
			return "no numbers";
		}

}

//===========================================================================
// Main function
//===========================================================================

int main(void) {

  // This time, autotest always runs as an invisible aid to you.
  //autotest();

  // GPIO enable
  enable_ports();
  // setup keyboard
  init_tim7();

//  // LED array Bit Bang
////#define BIT_BANG
//#if defined(BIT_BANG)
//    setup_bb();
//    drive_bb();
//#endif
//
//    // Direct SPI peripheral to drive LED display
////#define SPI_LEDS
//#if defined(SPI_LEDS)
//    init_spi2();
//    setup_dma();
//    enable_dma();
//    init_tim15();
//    show_keys();
//#endif
//
//    // LED array SPI
////#define SPI_LEDS_DMA
//#if defined(SPI_LEDS_DMA)
//    init_spi2();
//    spi2_setup_dma();
//    spi2_enable_dma();
//    show_keys();
//#endif

    // SPI OLED direct drive
#define SPI_OLED
#if defined(SPI_OLED)
    init_spi1();
    spi1_init_oled();

    // i changed this stuff
    spi1_display1("position:");

    char myString[10];
    int count = 0;

    while(count < 100) {
    	char* result = count_oled(myString, &count);
//    	small_delay();
    	spi1_display2(result);

    }

//    spi1_display2(login);
#endif

    // SPI
//#define SPI_OLED_DMA
#if defined(SPI_OLED_DMA)
    init_spi1();
    spi1_init_oled();
    spi1_setup_dma();
    spi1_enable_dma();
#endif

    // Game on!  The goal is to score 100 points.
//    game();
}
