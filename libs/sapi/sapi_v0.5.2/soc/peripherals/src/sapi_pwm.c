/* Copyright 2016, Ian Olivieri
 * Copyright 2016, Eric Pernia.
 * Copyright 2020, Guillermo Ferrari.
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

/* Date: 2016-02-10 */

/* TODO: hacer una forma de buscar las funciones que tocan el modulo siguiente
 * All functions relative to the microcontroller */

/*==================[inclusions]=============================================*/

#include "sapi_pwm.h"
#include "sapi_gpio.h"


/*==================[macros and definitions]=================================*/

#ifndef EMPTY_POSITION
#define EMPTY_POSITION 255
#endif

#define PWM_TOTALNUMBER   16   /* 4 canales de 4 timers */




/*==================[internal data declaration]==============================*/

typedef struct{
   TIM_HandleTypeDef*       timer;
   pinInitGpioStm32f1xx_t   pwmCh1Pin;
   pinInitGpioStm32f1xx_t   pwmCh2Pin;
   pinInitGpioStm32f1xx_t   pwmCh3Pin;
   pinInitGpioStm32f1xx_t   pwmCh4Pin;
   bool_t					configured;
} timerStmInit_t;

TIM_HandleTypeDef htimer1;
TIM_HandleTypeDef htimer2;
TIM_HandleTypeDef htimer3;
TIM_HandleTypeDef htimer4;

static timerStmInit_t stmTimers[] = {
// { timerAddr, { {ch1Port, ch1pin}, ch1mode }, { {ch2Port, ch2pin}, ch2mode },  { {ch3Port, ch3pin}, ch3mode }, { {ch4Port, ch4pin}, ch4mode } },
   {
      &htimer1,
      {{GPIOA, GPIO_PIN_8 }, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      {{GPIOA, GPIO_PIN_9}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  {{GPIOA, GPIO_PIN_10}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  {{GPIOA, GPIO_PIN_11}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  FALSE,

   },
   {
      &htimer2,
      {{GPIOA, GPIO_PIN_0 }, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      {{GPIOA, GPIO_PIN_1}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  {{GPIOA, GPIO_PIN_2}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  {{GPIOA, GPIO_PIN_3}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  FALSE,
   },
   {
      &htimer3,
      {{GPIOA, GPIO_PIN_6 }, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      {{GPIOA, GPIO_PIN_7}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  {{GPIOB, GPIO_PIN_0}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  {{GPIOB, GPIO_PIN_1}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  FALSE,
   },
   {
      &htimer4,
      {{GPIOB, GPIO_PIN_6 }, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      {{GPIOB, GPIO_PIN_7}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  {{GPIOB, GPIO_PIN_8}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  {{GPIOB, GPIO_PIN_9}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
	  FALSE,
   },
};

#define NUMBER_OF_TIMERS (sizeof(stmTimers)/sizeof(timerStmInit_t)) /* calcula automaticamente la cantidad de timers
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	de acuerdo a lo llenado en el array anterior*/

uint32_t channel[]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};

/*==================[internal functions declaration]=========================*/

/*
 * @Brief: Initializes the pwm timers.
 * @param  none
 * @return nothing
 */
static void pwmInitTimers(pwmMap_t);
//
///*
// * @brief:   adds pwm to the the list of working pwms
// * @param:   pwmNumber:   ID of the pwm, from 0 to 10
// * @return:   True if pwm was successfully attached, False if not.
// */
static bool_t pwmAttach( pwmMap_t pwmNumber );
//
///*
// * @brief:   removes pwm (attached to pwmNumber) from the list
// * @param:   pwmNumber:   ID of the pwm, from 0 to 10
// * @return:    True if pwm was successfully detached, False if not.
// */
static bool_t pwmDetach( pwmMap_t pwmNumber );

static bool_t enablePwmFor(pwmMap_t pwmNumber);

//
///*==================[internal data definition]===============================*/
//
///*==================[external data definition]===============================*/
//
///* Enter a pwm number, get a sct number
// * Since this module works with pwm numbers, but uses sct channels to generate
// * the signal, its necessary to connect pwm number with the SctMap_t (sAPI_PeripheralMap.h).
// * This way the user sets "pwms", while using the sct peripheral internally*/
//static const uint8_t pwmMap[PWM_TOTALNUMBER] = {
//   /* PWM0 */  CTOUT1,  /* T_FIL1 */
//   /* PWM1 */  CTOUT12, /* T_COL2 */
//   /* PWM2 */  CTOUT10, /* T_COL0 */
//   /* PWM3 */  CTOUT0,  /* T_FIL2 */
//   /* PWM4 */  CTOUT3,  /* T_FIL3 */
//   /* PWM5 */  CTOUT13, /* T_COL1 */
//   /* PWM6 */  CTOUT7,  /* GPIO8  */
//   /* PWM7 */  CTOUT2,  /* LED1   */
//   /* PWM8 */  CTOUT5,  /* LED2   */
//   /* PWM9 */  CTOUT4,  /* LED3   */
//   /* PWM10 */ CTOUT6   /* GPIO2  */
//};
//
///*when the user adds a pwm with pwmAttach the list updates with the pin number of the element*/
static uint8_t AttachedPWMList[PWM_TOTALNUMBER] = {
   /*Position | Pwm Number*/
   /*0*/  EMPTY_POSITION,
   /*1*/  EMPTY_POSITION,
   /*2*/  EMPTY_POSITION,
   /*3*/  EMPTY_POSITION,
   /*4*/  EMPTY_POSITION,
   /*5*/  EMPTY_POSITION,
   /*6*/  EMPTY_POSITION,
   /*7*/  EMPTY_POSITION,
   /*8*/  EMPTY_POSITION,
   /*9*/	EMPTY_POSITION,
   /*10*/ EMPTY_POSITION,
   /*11*/  EMPTY_POSITION,
   /*12*/  EMPTY_POSITION,
   /*13*/  EMPTY_POSITION,
   /*14*/	EMPTY_POSITION,
   /*15*/ EMPTY_POSITION
};
//
///*==================[internal functions definition]==========================*/
//
///*
// * @Brief:   Initializes the pwm timers.
// * @param   none
// * @return   nothing
// */
static void pwmInitTimers(pwmMap_t pwmNumber)
{
	TIM_HandleTypeDef* aux;
	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};

	aux = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;
	switch(pwmNumber){

		case TIM1_CH1:
		case TIM1_CH2:
		case TIM1_CH3:
		case TIM1_CH4:
			 aux->Instance= TIM1;
			 aux->Init.Period =PWM_T1PERIOD;
             aux->Init.Prescaler = PWM_T1PRESCALER;
		   __HAL_RCC_TIM1_CLK_ENABLE();
			break;
		case TIM2_CH1:
		case TIM2_CH2:
		case TIM2_CH3:
		case TIM2_CH4:
			aux->Instance= TIM2;
			aux->Init.Period =PWM_T2PERIOD;
            aux->Init.Prescaler = PWM_T2PRESCALER;
		   __HAL_RCC_TIM2_CLK_ENABLE();
			break;
		case TIM3_CH1:
		case TIM3_CH2:
		case TIM3_CH3:
		case TIM3_CH4:
			aux->Instance= TIM3;
			aux->Init.Period =PWM_T3PERIOD;
	        aux->Init.Prescaler = PWM_T3PRESCALER;
		    __HAL_RCC_TIM3_CLK_ENABLE();
			break;
		case TIM4_CH1:
		case TIM4_CH2:
		case TIM4_CH3:
		case TIM4_CH4:
			aux->Instance= TIM4;
			aux->Init.Period =PWM_T4PERIOD;
		    aux->Init.Prescaler = PWM_T4PRESCALER;
		    __HAL_RCC_TIM4_CLK_ENABLE();
			break;
	}
	  aux->Init.CounterMode = TIM_COUNTERMODE_UP;
	  aux->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	  aux->Init.RepetitionCounter = 0;
	  aux->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	  if (HAL_TIM_Base_Init(aux) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	  if (HAL_TIM_ConfigClockSource(aux, &sClockSourceConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  if (HAL_TIM_PWM_Init(aux) != HAL_OK)
	  {
	    Error_Handler();
	  }
	  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	  if (HAL_TIMEx_MasterConfigSynchronization(aux, &sMasterConfig) != HAL_OK)
	  {
	    Error_Handler();
	  }

}
//
///*
// * @brief:   adds pwm to the the list of working pwms
// * @param:   pwmNumber:   ID of the pwm, from 0 to 10
// * @return:   True if pwm was successfully attached, False if not.
// */
static bool_t pwmAttach( pwmMap_t pwmNumber)
{

   bool_t success = FALSE;
   uint8_t position = 0;

   position = pwmIsAttached(pwmNumber);
   if(position==0) {
      position = pwmIsAttached(EMPTY_POSITION); /* Searches for the first empty position */
      if(position) { /* if position==0 => there is no room in the list for another pwm */
         AttachedPWMList[position-1] = pwmNumber;
         enablePwmFor(pwmNumber);
         success = TRUE;
      }
   }
   return success;
}
//
///*
// * @brief:   removes pwm (attached to pwmNumber) from the list
// * @param:   pwmNumber:   ID of the pwm, from 0 to 10
// * @return:    True if pwm was successfully detached, False if not.
// */
static bool_t pwmDetach( pwmMap_t pwmNumber )
{

   bool_t success = FALSE;
   uint8_t position = 0;

   position = pwmIsAttached(pwmNumber);

   if(position) {
      AttachedPWMList[position-1] = EMPTY_POSITION;
      success = TRUE;
   }
   return success;
}
//
///*==================[external functions definition]==========================*/
//
///*
// * @brief:   change the value of the pwm at the selected pin
// * @param:   pwmNumber:   ID of the pwm, from 0 to 15
// * @param:   value:   8bit value, from 0 to 255
// * @return:   True if the value was successfully changed, False if not.
// */
bool_t pwmWrite( pwmMap_t pwmNumber, uint8_t value )
{
   TIM_HandleTypeDef* aux = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;
   bool_t success = FALSE;
   uint8_t position = 0;
   uint16_t pwmPeriod = ((uint16_t)aux->Init.Period*value)/255;

   position = pwmIsAttached(pwmNumber);

   if(position) {
	   __HAL_TIM_SET_COMPARE(aux, channel[pwmNumber/NUMBER_OF_TIMERS], pwmPeriod);
      success = TRUE;
   }

   return success;
}
//
///*
// * @brief:   read the value of the pwm in the pin
// * @param:   pwmNumber:   ID of the pwm, from 0 to 15
// * @return:   value of the pwm in the pin (0 ~ 255).
// *   If an error ocurred, return = EMPTY_POSITION = 255
// */
bool_t pwmRead( pwmMap_t pwmNumber, uint8_t* rv )
{
   TIM_HandleTypeDef* aux = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;
   uint8_t position = 0;
   position = pwmIsAttached(pwmNumber);
   uint16_t pwmPeriod = (uint16_t)aux->Init.Period;
   uint32_t value=0;

   if(position) {
	value= (__HAL_TIM_GET_COMPARE(aux, channel[pwmNumber/NUMBER_OF_TIMERS]));
	value*=255;
	value/=pwmPeriod;
	if(value)
		value++;// por redondeo siempre devuelve una unidad menos
	*rv = (uint8_t)value;
   } else {
      return FALSE;
   }

   return TRUE;
}
//
//
///*
// * @Brief: Initializes the pwm peripheral.
// * @param  uint8_t pwmNumber
// * @param  uint8_t config
// * @return bool_t true (1) if config it is ok
// */
bool_t pwmInit( pwmMap_t pwmNumber, pwmInit_t config)
{
   timerStmInit_t* aux = &stmTimers[pwmNumber%4];
   bool_t ret_val = 1;


   switch(config) {

   case PWM_ENABLE:

	  if(aux->configured==FALSE){ // me fijo si el timer ya esta configurado, me ahorro configurarlo de nuevo
      pwmInitTimers(pwmNumber);
      aux->configured=TRUE;
	  }
      break;

   case PWM_DISABLE:
      ret_val = 0;
      break;

   case PWM_ENABLE_OUTPUT:
      ret_val = pwmAttach( pwmNumber );
      break;

   case PWM_DISABLE_OUTPUT:
      ret_val = pwmDetach( pwmNumber );
      break;

   default:
      ret_val = 0;
      break;
   }

   return ret_val;
}

///*
// * @brief:   Tells if the pwm is currently active, and its position
// * @param:   pwmNumber:   ID of the pwm, from 0 to 10
// * @return:   position (1 ~ PWM_TOTALNUMBER), 0 if the element was not found.
// */
uint8_t pwmIsAttached( pwmMap_t pwmNumber )
{
   uint8_t position = 0, positionInList = 0;
   while ( (position < PWM_TOTALNUMBER) &&
           (pwmNumber != AttachedPWMList[position]) ) {
      position++;
   }

   if (position < PWM_TOTALNUMBER) {
      positionInList = position + 1;
   } else {
      positionInList = 0;
   }

   return positionInList;
}



static bool_t enablePwmFor(pwmMap_t pwmNumber){
		TIM_HandleTypeDef* aux;
		TIM_OC_InitTypeDef sConfigOC = {0};
		TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
		GPIO_InitTypeDef GPIO_InitStruct = {0};
		aux = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;

		  sConfigOC.OCMode = TIM_OCMODE_PWM1;

		  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
		  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
		  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
		  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
		  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;

		  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
		  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
		  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
		  sBreakDeadTimeConfig.DeadTime = 0;
		  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
		  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
		  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
		  if (HAL_TIMEx_ConfigBreakDeadTime(aux, &sBreakDeadTimeConfig) != HAL_OK)
		  {
		      Error_Handler();
		  }


		  if(aux->Instance==TIM1){
			  sConfigOC.Pulse =PWM_T1PERIOD/2;  // arranca en la mitad de ciclo de actividad
			  __HAL_RCC_GPIOA_CLK_ENABLE();
		  }
		  if(aux->Instance==TIM2){
			  sConfigOC.Pulse =PWM_T2PERIOD/2;
			  __HAL_RCC_GPIOA_CLK_ENABLE();
		  }
		  if(aux->Instance==TIM3){
			  sConfigOC.Pulse =PWM_T3PERIOD/2;
			  __HAL_RCC_GPIOA_CLK_ENABLE();
			  __HAL_RCC_GPIOB_CLK_ENABLE();
		  }
		  if(aux->Instance==TIM4){
			sConfigOC.Pulse =PWM_T4PERIOD/2;
			__HAL_RCC_GPIOB_CLK_ENABLE();
		  }
		  if (HAL_TIM_PWM_ConfigChannel(aux, &sConfigOC, channel[pwmNumber/NUMBER_OF_TIMERS]) != HAL_OK)
		  {
		    Error_Handler();
		  }

		  switch(pwmNumber){

			case TIM1_CH1:
			case TIM2_CH1:
			case TIM3_CH1:
			case TIM4_CH1:
			    GPIO_InitStruct.Pin = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh1Pin.gpio.pin;
			    GPIO_InitStruct.Mode = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh1Pin.mode;
			    GPIO_InitStruct.Speed = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh1Pin.speed;
			    HAL_GPIO_Init(stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh1Pin.gpio.port, &GPIO_InitStruct);
				break;
			case TIM1_CH2:
			case TIM2_CH2:
			case TIM3_CH2:
			case TIM4_CH2:
			    GPIO_InitStruct.Pin = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh2Pin.gpio.pin;
			    GPIO_InitStruct.Mode = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh2Pin.mode;
			    GPIO_InitStruct.Speed = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh2Pin.speed;
			    HAL_GPIO_Init(stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh2Pin.gpio.port, &GPIO_InitStruct);
				break;
			case TIM1_CH3:
			case TIM2_CH3:
			case TIM3_CH3:
			case TIM4_CH3:
			    GPIO_InitStruct.Pin = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh3Pin.gpio.pin;
			    GPIO_InitStruct.Mode = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh3Pin.mode;
			    GPIO_InitStruct.Speed = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh3Pin.speed;
			    HAL_GPIO_Init(stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh3Pin.gpio.port, &GPIO_InitStruct);
				break;
			case TIM1_CH4:
			case TIM2_CH4:
			case TIM3_CH4:
			case TIM4_CH4:
			    GPIO_InitStruct.Pin = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh4Pin.gpio.pin;
			    GPIO_InitStruct.Mode = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh4Pin.mode;
			    GPIO_InitStruct.Speed = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh4Pin.speed;
			    HAL_GPIO_Init(stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh4Pin.gpio.port, &GPIO_InitStruct);
				break;

		  }
		  HAL_TIM_Base_Start(aux);
		  HAL_TIM_PWM_Start(aux,channel[pwmNumber/NUMBER_OF_TIMERS]);

//		  if(aux->Instance==TIM1)
//		  {
//		  /* USER CODE BEGIN TIM1_MspPostInit 0 */
//
//		  /* USER CODE END TIM1_MspPostInit 0 */
//
//		    __HAL_RCC_GPIOA_CLK_ENABLE();
//
//
//		    /**TIM1 GPIO Configuration
//		    PA8     ------> TIM1_CH1
//		    PA9     ------> TIM1_CH2
//		    PA10     ------> TIM1_CH3
//		    PA11     ------> TIM1_CH4
//		    */
//		    GPIO_InitStruct.Pin = stmTimers[pwmNumber%NUMBER_OF_TIMERS].pwmCh1Pin;
//		    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
//		    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
//		    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
//
//		  }
}
/*==================[end of file]============================================*/
