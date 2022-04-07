#include "GENE.h"

extern TIM_HandleTypeDef htim5;
extern int toggle;

void GENE_SetFreqPin(int f){
	htim5.Init.Period = 36000/f-1;
}

void GENE_TogglePin(void){
	if(toggle%2 == 0){
		GPIOA->ODR |= (1<<0);
	}
	else if(toggle%2 == 1){
		GPIOA->ODR &= 0xFFFFFFFE;
	}
	toggle += 1;
}
