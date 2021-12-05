/*
 * NeopixelDriver.cpp
 *
 *  Created on: Dec 5, 2021
 *      Author: felix
 */

#include "NeopixelDriver.h"




	void configTimer(TIM_HandleTypeDef *htim){

	}
	void addNeopixelChannel(NeopixelChannel nc){

	}

	NeopixelChannel newChannel(TIM_HandleTypeDef *htim, uint32_t TimerChannel){
		NeopixelChannel nc;
		nc.htim=htim;
		nc.TimerChannel=TimerChannel;
	}
