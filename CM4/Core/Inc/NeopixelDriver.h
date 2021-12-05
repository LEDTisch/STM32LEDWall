/*
 * NeopixelDriver.h
 *
 *  Created on: Dec 5, 2021
 *      Author: felix
 */

#ifndef NEOPIXELDRIVER_H_
#define NEOPIXELDRIVER_H_
#include "stm32h7xx_hal.h"
#define PI 3.14159265

typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t TimerChannel;
	uint16_t NUMLED;
	uint8_t usebrightness;
	uint8_t datasentflag;
	uint16_t *pwmData;
	uint8_t *LED_Data[4];
}NeopixelChannel;


	void newChannel(NeopixelChannel *nchannel, TIM_HandleTypeDef *htim, uint32_t TimerChannel, uint16_t NUMLED, uint8_t usebrightness);



	void Set_LED (NeopixelChannel *nchannel, int LEDnum, int Red, int Green, int Blue);
	void WS2812_Send (NeopixelChannel *nchannel, uint8_t blocking);
	void Reset_LED (NeopixelChannel *nchannel);



#endif /* SRC_DRIVER_NEOPIXELDRIVER_H_ */
