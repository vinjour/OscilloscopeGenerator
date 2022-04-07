#include "LED.h"


extern int LED_NbTimes;
extern TIM_HandleTypeDef htim2;


void LED_SetFreqGreen(int f){
	if (f==1){
		htim2.Init.Period = 3599; // f = 1 Hz
	}
	else if (f==2){
		htim2.Init.Period = 1799; // f = 2 Hz
	}
	else if (f==3){
		htim2.Init.Period = 1199; // f = 3 Hz
	}
	else if (f==4){
		htim2.Init.Period = 899; // f = 4 Hz
	}
}

void LED_DispGreen(int val){
	if (val==1){
		GPIOI->ODR |= 0x2;	// if the LED is off we put the first bit of GPIOI_ODR at 1
	}																//= We switch on the LED
	else{
		GPIOI->ODR &= ~0x2; // if the LED is on we put the first bit of GPIOI_ODR at 0
	}																			//= We switch off the LED
}
