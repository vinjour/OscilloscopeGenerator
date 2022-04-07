#include "HMI.h"


extern char touch_buf[50];
extern int f;
extern int mode;
extern TS_StateTypeDef g_ts;

void HMI_changeFreqLed(){
	if (mode == 1){
		BSP_TS_GetState(&g_ts);

		if(g_ts.touchDetected == 1){
			if ((g_ts.touchX[0] < 200) && (f>1) && (f<=4)){
				f -= 1;
				LED_SetFreqGreen(f);
				sprintf(touch_buf,"%d Hz",f);
				BSP_LCD_DisplayStringAt(10,200,(uint8_t *)touch_buf,CENTER_MODE);
			}
			else if ((g_ts.touchX[0] >= 200) && (f>=1) && (f<4)){
				f += 1;
				LED_SetFreqGreen(f);
				sprintf(touch_buf,"%d Hz",f);
				BSP_LCD_DisplayStringAt(10,200,(uint8_t *)touch_buf,CENTER_MODE);
			}
		}

		else if(g_ts.touchDetected == 0){
			sprintf(touch_buf,"%d Hz",f);
			BSP_LCD_DisplayStringAt(10,200,(uint8_t *)touch_buf,CENTER_MODE);
		}
	}

	else if (mode == 0){
			BSP_TS_GetState(&g_ts);

			if(g_ts.touchDetected == 1){
				f = g_ts.touchX[1]*1000/480;
				GENE_SetFreqPin(f);
				sprintf(touch_buf,"%d Hz",f);
				BSP_LCD_DisplayStringAt(10,200,(uint8_t *)touch_buf,CENTER_MODE);
			}

			else if(g_ts.touchDetected == 0){
				sprintf(touch_buf,"%d Hz",f);
				BSP_LCD_DisplayStringAt(10,200,(uint8_t *)touch_buf,CENTER_MODE);
			}
		}
}

void setMode(){
	if(mode == 1){
		mode = 0;
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(10,100,(uint8_t *)"Mode Reglages",CENTER_MODE);
	}
	else if(mode == 0){
		mode = 1;
		BSP_LCD_Clear(LCD_COLOR_WHITE);
		BSP_LCD_DisplayStringAt(10,100,(uint8_t *)"Mode Neutre",CENTER_MODE);
	}
}
