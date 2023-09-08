/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f0xx.h"
#include "stm32f0xx_gpio.h"

int turnLED(int dir, int currLED);
			

int main(void)
{
	RCC->AHBENR |= RCC_AHBENR_GPIOCEN;	// Turning on the clock for GPIO channel C
	RCC->AHBENR |= RCC_AHBENR_GPIOAEN;	// Turning on the clock for GPIO channel A

	GPIO_InitTypeDef Init_GPIO;

//	Init_GPIO.GPIO_Pin = GPIO_Pin_0;
//	Init_GPIO.GPIO_Mode = GPIO_Mode_IN;
//	Init_GPIO.GPIO_Speed = GPIO_Speed_Level_1;
//	Init_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
//	GPIO_Init(GPIOB, &Init_GPIO);	// B0 is the clk for the encoder (Is not working)

	Init_GPIO.GPIO_Pin = GPIO_Pin_13;
	Init_GPIO.GPIO_Mode = GPIO_Mode_IN;
	Init_GPIO.GPIO_Speed = GPIO_Speed_Level_3;
	Init_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &Init_GPIO);	// C13 is the clk for the encoder

	Init_GPIO.GPIO_Pin = GPIO_Pin_12;
	Init_GPIO.GPIO_Mode = GPIO_Mode_IN;
	Init_GPIO.GPIO_Speed = GPIO_Speed_Level_3;
	Init_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOC, &Init_GPIO);	// C12 is the dt for the encoder

	Init_GPIO.GPIO_Pin = GPIO_Pin_1;
	GPIO_Init(GPIOB, &Init_GPIO);	// B1 is the dt for the encoder (Is not working)

	Init_GPIO.GPIO_Pin = GPIO_Pin_2;
	GPIO_Init(GPIOB, &Init_GPIO);	// B2 is the switch for the encoder (Not sure if working)

	Init_GPIO.GPIO_Pin = GPIO_Pin_0;
	Init_GPIO.GPIO_Mode = GPIO_Mode_OUT;
	Init_GPIO.GPIO_Speed = GPIO_Speed_2MHz;
	Init_GPIO.GPIO_OType = GPIO_OType_PP;
	Init_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &Init_GPIO);	// A0 Output pin for the yellow LED

	Init_GPIO.GPIO_Pin = GPIO_Pin_1;
	Init_GPIO.GPIO_Mode = GPIO_Mode_OUT;
	Init_GPIO.GPIO_Speed = GPIO_Speed_2MHz;
	Init_GPIO.GPIO_OType = GPIO_OType_PP;
	Init_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &Init_GPIO);	// A1 Output pin for the green LED

	Init_GPIO.GPIO_Pin = GPIO_Pin_2;
	Init_GPIO.GPIO_Mode = GPIO_Mode_OUT;
	Init_GPIO.GPIO_Speed = GPIO_Speed_2MHz;
	Init_GPIO.GPIO_OType = GPIO_OType_PP;
	Init_GPIO.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_Init(GPIOA, &Init_GPIO);	// A2 Output pin for the red LED

	// GPIOC->MODER |= 0x55 << 12;		// Switching the mode for pin 12 to output

	GPIO_SetBits(GPIOA, GPIO_Pin_1);	// Starting at the green LED in the middle
	int currLED = 1;

	while(1)
	{
		int state = 0;
		int clk = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
		int dt = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_12);

		if (state == 0)	// IDLE STATE
		{
			if (!clk && dt)
			{
				state = 1;
			}
			else if (!dt && clk)
			{
				state = 4;
			}
			else
			{
				state = 0;
			}
		}
		if (state == 1) // INITIAL CCW
		{
			// currLED = turnLED(0, currLED);
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);
			if (!dt && !clk)
			{
				state = 2;
			}
			if (clk)
			{
				state = 0;
			}
			else
			{
				state = 1;
			}
		}
		if (state == 2) // MID CCW
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);
			if (clk && !dt)
			{
				state = 3;
			}
			if (dt)
			{
				state = 1;
			}
			else
			{
				state = 2;
			}
		}
		if (state == 3) // END CCW
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_0);
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			GPIO_ResetBits(GPIOA, GPIO_Pin_2);
			// currLED = turnLED(0, currLED);
			if (dt && clk)
			{
				state = 0;
			}
			if (!clk)
			{
				state = 2;
			}
			else
			{
				state = 3;
			}
		}
		if (state == 4) // INITIAL CW
		{
			// currLED = turnLED(1, currLED);
			GPIO_SetBits(GPIOA, GPIO_Pin_2);
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
			if (!clk && !dt)
			{
				state = 5;
			}
			else
			{
				state = 4;
			}
		}
		if (state == 5) // MID CW
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_2);
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
			if (dt && !clk)
			{
				state = 6;
			}
			else
			{
				state = 5;
			}
		}
		if (state == 6) // END CW
		{
			GPIO_SetBits(GPIOA, GPIO_Pin_2);
			GPIO_ResetBits(GPIOA, GPIO_Pin_1);
			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
			// currLED = turnLED(1, currLED);
			if (clk && dt)
			{
				state = 0;
			}
			else
			{
				state = 6;
			}
		}
	}

//		GPIO_SetBits(GPIOA, GPIO_Pin_2);
//		GPIO_SetBits(GPIOA, GPIO_Pin_1);
//		for (int i = 0; i <= 500000; i++) ;
//		GPIO_ResetBits(GPIOA, GPIO_Pin_2);
//		GPIO_ResetBits(GPIOA, GPIO_Pin_1);
//		for (int i = 0; i <= 500000; i++) ;

		// int clk = GPIO_ReadInputDataBit(GPIOC, GPIO_Pin_13);
//		if (dt == 0)
//		{
//			GPIO_SetBits(GPIOA, GPIO_Pin_0);
//		}
//		else
//		{
//			GPIO_ResetBits(GPIOA, GPIO_Pin_0);
//		}
//	}

	// GPIOA->MODER &= ~0b111111;		// Switching the mode for pin A0, A1, A2 to input
//	for(;;)
//	{
//		GPIO_SetBits(GPIOB, GPIO_Pin_3);	// Turning on LED connected to B3
//		int clk = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
//		// printf("%d", clk);
//		if (clk == 0)
//		{
//			GPIOC->ODR ^= 0x3c0;	// Toggling the output value to flash the LED
//
//			for (int x = 0; x<1000000; x++)
//			{
//				;	// Simple waiting loop so you can see the LED flash
//			}
//		}
//	}
}

int turnLED(int dir, int currLED)
{
	GPIO_ResetBits(GPIOA, GPIO_Pin_0);
	GPIO_ResetBits(GPIOA, GPIO_Pin_1);
	GPIO_ResetBits(GPIOA, GPIO_Pin_2);
	if (currLED == 0 && dir == 0)
	{
		currLED = 2;
	}
	else if (currLED == 2 && dir == 1)
	{
		currLED = 0;
	}
	else if (dir == 0)
	{
		currLED--;
	}
	else
	{
		currLED++;
	}

	if (currLED == 0)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_0);
	}
	else if (currLED == 1)
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_1);
	}
	else
	{
		GPIO_SetBits(GPIOA, GPIO_Pin_2);
	}

	return(currLED);

}


