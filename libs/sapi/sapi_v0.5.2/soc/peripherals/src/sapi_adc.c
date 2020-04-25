/* Copyright 2016, Ian Olivieri
 * Copyright 2016, Eric Pernia.
 * Copyright 2020, Nahuel Espinosa.
 * All rights reserved.
 *
 * This file is part sAPI library for microcontrollers.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/* Date: 2020-04-24 */

/*==================[inclusions]=============================================*/

#include "sapi_adc.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
ADC_HandleTypeDef hadc1;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

/*
 * @brief:  enable/disable the ADC peripheral
 * @param:  ADC_ENABLE, ADC_DISABLE
 * @return: none
*/
void adcInit( adcInit_t config )
{
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   
   switch(config) {

      case ADC_ENABLE:
         /* Enable ADC peripheral */
         __HAL_RCC_ADC1_CLK_ENABLE();
         __HAL_RCC_GPIOA_CLK_ENABLE();

         hadc1.Instance = ADC1;
         hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
         hadc1.Init.ContinuousConvMode = DISABLE;
         hadc1.Init.DiscontinuousConvMode = DISABLE;
         hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
         hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
         hadc1.Init.NbrOfConversion = 1;

         HAL_ADC_Init(&hadc1);

         /* Configure pins */
         GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3;
         GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
         HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
         break;

      case ADC_DISABLE:
         /* Disable ADC peripheral */
         __HAL_RCC_ADC1_CLK_DISABLE();
         HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);

         break;
      }
}


/*
 * @brief   Get the value of one ADC channel. Mode: BLOCKING
 * @param   CH0 ... CHn
 * @return  analog value
 */
uint16_t adcRead( adcMap_t analogInput )
{
   ADC_ChannelConfTypeDef sConfig;
   uint16_t analogValue = 0;

   switch(analogInput) {
      case CH0:
         sConfig.Channel = ADC_CHANNEL_0;
         break;
      case CH1:
         sConfig.Channel = ADC_CHANNEL_1;
         break;
      case CH2:
         sConfig.Channel = ADC_CHANNEL_2;
         break;
      case CH3:
         sConfig.Channel = ADC_CHANNEL_3;
         break;
   }

   // Enable channel
   sConfig.Rank = ADC_REGULAR_RANK_1;
   sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
   HAL_ADC_ConfigChannel(&hadc1, &sConfig);

   // Start conversion
   HAL_ADC_Start(&hadc1);

   // Wait for conversion complete
   if( HAL_ADC_PollForConversion(&hadc1, 200) == HAL_OK ) {
      analogValue = HAL_ADC_GetValue(&hadc1);
   }

   // Stop conversion
   HAL_ADC_Stop(&hadc1);

   return analogValue;
}

/*==================[end of file]============================================*/
