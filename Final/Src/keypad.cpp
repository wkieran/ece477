#include "keypad.h"
#include "stm32f4xx_it.h"

extern uint8_t switching;
extern uint8_t escaping;
extern TIM_HandleTypeDef htim11;
extern int LED_Buffer[16];

void setuppins()
{
  // Initialize the debounce counter array
  for (uint8_t i = 0; i < NUM_BTN_COLUMNS; i++)
  {
    for (uint8_t j = 0; j < NUM_BTN_ROWS; j++)
    {
      debounce_count[i][j] = 0;
    }
  }
}

void flash(uint8_t spot){
	if(spot & 0x1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
	}
	else{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
	}
	if((spot >> 1) & 0x1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_SET);
	}
	else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, GPIO_PIN_RESET);
		}
	if((spot >> 2) & 0x1){
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	}
	else{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);
		}
	if((spot >> 3) & 0x1){
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else{
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_RESET);
		}
	return;
}

void fullflash(){
	int dark = 1;
	for(uint8_t i = 0; i < 16; i++){
		if(LED_Buffer[i] == 1){
			if(dark == 1){
				dark = 0;
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
			}
			flash(i);
		}
	}
	if(dark) HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
}

uint8_t rowscan(uint8_t readval, uint16_t current){
	uint8_t val;
	HAL_GPIO_WritePin(GPIOC, btncolumnpins[current], GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, btncolumnpins[(current + 1) % 4], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, btncolumnpins[(current + 2) % 4], GPIO_PIN_SET);
	HAL_GPIO_WritePin(GPIOC, btncolumnpins[(current + 3) % 4], GPIO_PIN_SET);
	for(int j = 0; j < MAX_DEBOUNCE + 1; j++){
		for(int i = 0; i < 4; i++){
			val = HAL_GPIO_ReadPin(GPIOC, btnrowpins[i]);
			if (val == 0)
			{
				// active low: val is low when btn is pressed
				if ( debounce_count[current][i] < MAX_DEBOUNCE)
			    {
					debounce_count[current][i]++;
			        if ( debounce_count[current][i] == MAX_DEBOUNCE )
			        {
			          // Do whatever you want to with the button press here:
			          // toggle the current LED state
			          //LED_buffer[current][j] = !LED_buffer[current][j];
			        	return i;
			        }
			      }
			    }
			    else
			    {
			      // otherwise, button is released
			      if ( debounce_count[current][i] > 0)
			      {
			        debounce_count[current][i]--;
			        if ( debounce_count[current][i] == 0 )
			        {
			          // If you want to do something when a key is released, do it here:

			        }
			      }
			    }
		}
	}
	return readval;
}

void turnon(){ // power on keypad and start flashing with timer
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_TIM_Base_Start_IT(&htim11);
}

void poweron(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_RESET); // For when you want to enable the grid but without continuous flashing
}

void poweroff(){
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
}

void binflash(int intake){
	turnon();
	if(intake == 0){
		turnoff();
		return;
	}
	for(int i = 0; i < 16; i++){
		if((intake >> (15 - i)) & 0x1) LED_Buffer[i] = 1;
		else LED_Buffer[i] = 0;
	}
	return;
}

void turnoff(){ // turn off keypad and also stop timer
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_9, GPIO_PIN_SET);
	HAL_TIM_Base_Stop_IT(&htim11);
}

uint16_t fullscan(){
	uint8_t retval = 19;
	uint16_t checker = retval;
	uint8_t rowval = 0;
	int i;
	for(i = 0; i < 4; i++){
		retval = rowscan(retval, i);
		if(retval != checker){
			HAL_Delay(5);
			return retval + i * 4;
		}
	}
	return 19;
}

uint16_t waitscan(){ // forever loops until a valid value is read from the keypad, or until the "escaping" flag is set. This can be done via interupt in main.
	int retval = 19;
	int old = retval;
	while((switching == 0) & (escaping == 0)){
		retval = fullscan();
		if(retval != 19){
			return retval;
		}
	}
	escaping = 0;
	return old;
}

uint16_t waitscanInt(int feed){ // waitscan(), except when it's interrupted it returns the value which is fed into it; used to preserve values instead of overwrite with an invalid (19).
	int retval = waitscan();
	if (retval == 19) return feed;
	else return retval;
}
