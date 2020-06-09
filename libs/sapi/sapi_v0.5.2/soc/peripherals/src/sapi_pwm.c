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

/* Date: 2020-06-08 */

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
   bool_t                   configured;
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
                                                                       de acuerdo a lo llenado en el array anterior */

uint32_t channel[]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};

/*==================[internal functions declaration]=========================*/

/*
 * @Brief: Initializes the pwm timers.
 * @param  none
 * @return nothing
 */
static void pwmInitTimers(pwmMap_t);

/*
 * @brief:   adds pwm to the the list of working pwms
 * @param:   pwmNumber:   ID of the pwm, from 0 to 10
 * @return:   True if pwm was successfully attached, False if not.
 */
static bool_t pwmAttach( pwmMap_t pwmNumber );

/*
 * @brief:   removes pwm (attached to pwmNumber) from the list
 * @param:   pwmNumber:   ID of the pwm, from 0 to 10
 * @return:    True if pwm was successfully detached, False if not.
 */
static bool_t pwmDetach( pwmMap_t pwmNumber );


/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*when the user adds a pwm with pwmAttach the list updates with the pin number of the element*/
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
   /*9*/  EMPTY_POSITION,
   /*10*/ EMPTY_POSITION,
   /*11*/ EMPTY_POSITION,
   /*12*/ EMPTY_POSITION,
   /*13*/ EMPTY_POSITION,
   /*14*/ EMPTY_POSITION,
   /*15*/ EMPTY_POSITION
};


/*==================[internal functions definition]==========================*/

/*
 * @Brief:   Initializes the pwm timers.
 * @param    none
 * @return   nothing
 */
static void pwmInitTimers(pwmMap_t pwmNumber)
{
    TIM_HandleTypeDef* handle;
    TIM_ClockConfigTypeDef sClockSourceConfig = {0};
    TIM_MasterConfigTypeDef sMasterConfig = {0};

    handle = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;
    switch(pwmNumber){

        case TIM1_CH1:
        case TIM1_CH2:
        case TIM1_CH3:
        case TIM1_CH4:
             handle->Instance= TIM1;
             handle->Init.Period =PWM_T1PERIOD;
             __HAL_RCC_TIM1_CLK_ENABLE();
            break;
        case TIM2_CH1:
        case TIM2_CH2:
        case TIM2_CH3:
        case TIM2_CH4:
            handle->Instance= TIM2;
            handle->Init.Period =PWM_T2PERIOD;
            __HAL_RCC_TIM2_CLK_ENABLE();
            break;
        case TIM3_CH1:
        case TIM3_CH2:
        case TIM3_CH3:
        case TIM3_CH4:
            handle->Instance= TIM3;
            handle->Init.Period =PWM_T3PERIOD;
            __HAL_RCC_TIM3_CLK_ENABLE();
            break;
        case TIM4_CH1:
        case TIM4_CH2:
        case TIM4_CH3:
        case TIM4_CH4:
            handle->Instance= TIM4;
            handle->Init.Period =PWM_T4PERIOD;
            __HAL_RCC_TIM4_CLK_ENABLE();
            break;
    }
    handle->Init.Prescaler = 0;
    handle->Init.CounterMode = TIM_COUNTERMODE_UP;
    handle->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    handle->Init.RepetitionCounter = 0;
    handle->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    HAL_TIM_Base_Init(handle);
    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    HAL_TIM_ConfigClockSource(handle, &sClockSourceConfig);

    HAL_TIM_PWM_Init(handle);

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    HAL_TIMEx_MasterConfigSynchronization(handle, &sMasterConfig);
}

static bool_t enablePwmFor(pwmMap_t pwmNumber){
   TIM_HandleTypeDef* handle;
   TIM_OC_InitTypeDef sConfigOC = {0};
   TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};
   GPIO_InitTypeDef GPIO_InitStruct = {0};
   handle = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;

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

   if (HAL_TIMEx_ConfigBreakDeadTime(handle, &sBreakDeadTimeConfig) != HAL_OK)
   {
      Error_Handler();
   }


   if(handle->Instance==TIM1){
      sConfigOC.Pulse =PWM_T1PERIOD/2;  // arranca en la mitad de ciclo de actividad
      __HAL_RCC_GPIOA_CLK_ENABLE();
   }

   if(handle->Instance==TIM2){
      sConfigOC.Pulse =PWM_T2PERIOD/2;
      __HAL_RCC_GPIOA_CLK_ENABLE();
   }

   if(handle->Instance==TIM3){
      sConfigOC.Pulse =PWM_T3PERIOD/2;
      __HAL_RCC_GPIOA_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
   }

   if(handle->Instance==TIM4){
      sConfigOC.Pulse =PWM_T4PERIOD/2;
      __HAL_RCC_GPIOB_CLK_ENABLE();
   }

   if (HAL_TIM_PWM_ConfigChannel(handle, &sConfigOC, channel[pwmNumber/NUMBER_OF_TIMERS]) != HAL_OK)
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

   HAL_TIM_Base_Start(handle);
   HAL_TIM_PWM_Start(handle,channel[pwmNumber/NUMBER_OF_TIMERS]);
}

/*
 * @brief:   adds pwm to the the list of working pwms
 * @param:   pwmNumber:   ID of the pwm
 * @return:   True if pwm was successfully attached, False if not.
 */
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


/*
 * @brief:   removes pwm (attached to pwmNumber) from the list
 * @param:   pwmNumber:   ID of the pwm
 * @return:    True if pwm was successfully detached, False if not.
 */
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

/*==================[external functions definition]==========================*/

/*
 * @brief:   change the value of the pwm at the selected pin
 * @param:   pwmNumber:   ID of the pwm, from 0 to 15
 * @param:   value:   8bit value, from 0 to 255
 * @return:   True if the value was successfully changed, False if not.
 */
bool_t pwmWrite( pwmMap_t pwmNumber, uint8_t value )
{
   TIM_HandleTypeDef* handle = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;
   bool_t success = FALSE;
   uint8_t position = 0;
   uint16_t pwmPeriod = ((uint16_t)handle->Init.Period*value)/255;

   position = pwmIsAttached(pwmNumber);

   if(position) {
      __HAL_TIM_SET_COMPARE(handle, channel[pwmNumber/NUMBER_OF_TIMERS], pwmPeriod);
      success = TRUE;
   }

   return success;
}

/*
 * @brief:   read the value of the pwm in the pin
 * @param:   pwmNumber:   ID of the pwm, from 0 to 15
 * @return:   value of the pwm in the pin (0 ~ 255).
 *   If an error ocurred, return = EMPTY_POSITION = 255
 */
bool_t pwmRead( pwmMap_t pwmNumber, uint8_t* rv )
{
   TIM_HandleTypeDef* handle = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;
   uint8_t position = 0;
   position = pwmIsAttached(pwmNumber);
   uint16_t pwmPeriod = (uint16_t)handle->Init.Period;
   uint32_t value=0;

   if(position) {
       value= (__HAL_TIM_GET_COMPARE(handle, channel[pwmNumber/NUMBER_OF_TIMERS]));
       value*=255;
       value/=pwmPeriod;
       if(value)
           value++;    // por redondeo siempre devuelve una unidad menos
       *rv = (uint8_t)value;
   } else {
       return FALSE;
   }

   return TRUE;
}

/*
 * @Brief: Initializes the pwm peripheral.
 * @param  uint8_t pwmNumber
 * @param  uint8_t config
 * @return bool_t true (1) if config it is ok
 */
bool_t pwmInit( pwmMap_t pwmNumber, pwmInit_t config)
{
   timerStmInit_t* timer = &stmTimers[pwmNumber%4];
   bool_t ret_val = 1;

   switch(config) {

   case PWM_ENABLE:
      if( timer->configured==FALSE ) {
          pwmInitTimers(pwmNumber);
          timer->configured=TRUE;
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

/*
 * @brief:   Tells if the pwm is currently active, and its position
 * @param:   pwmNumber:   ID of the pwm, from 0 to 10
 * @return:   position (1 ~ PWM_TOTALNUMBER), 0 if the element was not found.
 */
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

/*==================[end of file]============================================*/
