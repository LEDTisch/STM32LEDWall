/*
 * NeopixelDriver.cpp
 *
 *  Created on: Dec 5, 2021
 *      Author: felix
 */

#include "NeopixelDriver.h"





	void newChannel(NeopixelChannel *nchannel, TIM_HandleTypeDef *htim, uint32_t TimerChannel, uint16_t NUMLED, uint8_t usebrightness){
		nchannel->htim=htim;
		nchannel->TimerChannel=TimerChannel;
		nchannel->NUMLED=NUMLED;
		nchannel->usebrightness=usebrightness;
		nchannel->datasentflag=1;
		nchannel->pwmData=(uint16_t *)malloc(((24*NUMLED)+51)*sizeof(uint16_t));
		for (int i=0;i<4;i++){
			nchannel->LED_Data[i] = (uint8_t*)malloc(NUMLED * sizeof(uint8_t));
		}
	}


	void Set_LED(NeopixelChannel *nchannel, int LEDnum, int Red, int Green, int Blue){
		nchannel->LED_Data[0][LEDnum] = LEDnum;
		nchannel->LED_Data[1][LEDnum] = Green;
		nchannel->LED_Data[2][LEDnum] = Red;
		nchannel->LED_Data[3][LEDnum] = Blue;
	}

	void WS2812_Send (NeopixelChannel *nchannel, uint8_t blocking){
		//if(nchannel->datasentflag==0)return;
		uint32_t indx=0;
		uint32_t color;


		for (int i= 0; i<nchannel->NUMLED; i++)
		{
	#if USE_BRIGHTNESS
			color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
	#else
			color = ((nchannel->LED_Data[1][i]<<16) | (nchannel->LED_Data[2][i]<<8) | (nchannel->LED_Data[3][i]));
	#endif

			for (int i=23; i>=0; i--)
			{
				if (color&(1<<i))
				{
					nchannel->pwmData[indx] = 58;  // 2/3 of 90
				}

				else nchannel->pwmData[indx] = 29;  // 1/3 of 90

				indx++;
			}

		}

		for (int i=0; i<50; i++)
		{
			nchannel->pwmData[indx] = 0;
			indx++;
		}
		//nchannel->datasentflag = 0;
		HAL_TIM_PWM_Stop_DMA(nchannel->htim, nchannel->TimerChannel);
		HAL_TIM_PWM_Start_DMA(nchannel->htim, nchannel->TimerChannel, (uint32_t *)nchannel->pwmData, indx);
		//if(blocking==1)while (!nchannel->datasentflag){};
	}

	void Reset_LED (NeopixelChannel *nchannel){
		for (int i=0; i<nchannel->NUMLED; i++)
		{
			nchannel->LED_Data[0][i] = i;
			nchannel->LED_Data[1][i]= 0;
			nchannel->LED_Data[2][i] = 0;
			nchannel->LED_Data[3][i] = 0;
		}
	}

	void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
	{
		//HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_1);
		//datasentflag=1;
	}
