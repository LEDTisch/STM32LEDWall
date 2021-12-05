/*
 * NeopixelDriver.h
 *
 *  Created on: Dec 5, 2021
 *      Author: felix
 */

#ifndef NEOPIXELDRIVER_H_
#define NEOPIXELDRIVER_H_
#include "stm32h7xx_hal.h"

typedef struct{
	TIM_HandleTypeDef *htim;
	uint32_t TimerChannel;
}NeopixelChannel;

	void configTimer(TIM_HandleTypeDef *htim);
	void addNeopixelChannel(NeopixelChannel nc);
	NeopixelChannel newChannel(TIM_HandleTypeDef *htim, uint32_t TimerChannel);
#endif /* SRC_DRIVER_NEOPIXELDRIVER_H_ */
