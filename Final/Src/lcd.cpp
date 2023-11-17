#include "lcd.h"

void lcd_setquad(int quad){
	lcd_send_cmd(0x02);
	HAL_Delay(1);
	int cap = 0;
	if(quad == 2){
		cap = 8;
	}
	if(quad == 3){
		cap = 40;
	}
	if(quad == 4){
		cap = 48;
	}
	for(int i = 0; i < cap; i++){
		lcd_send_cmd(0x14);
	}
}

void lcd_clear(void){
	lcd_send_cmd(0x01);
}

void lcd_setcursor(int set){
	int quad = set / 8 + 1;
	int pos = set % 8;
	lcd_setquad(quad);
	for(int i = 0; i < pos; i++){
		lcd_send_cmd(0x14);
	}
}

void lcd_send_int(int val){
	char buff[10];
	sprintf(buff, "%d", val);
	lcd_send_string(buff);
}
