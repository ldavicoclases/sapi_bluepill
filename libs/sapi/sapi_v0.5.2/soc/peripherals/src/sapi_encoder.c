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

/* Date: 2020-21-08 */


/*==================[   WARNING!!!   ]=======================================*/
/*      This peripheral share the hardware timers with PWM peripheral.
 *      You should not use an PWM timer and same Encoder Timer at once
 *      Is user's responsibility to check that.
 */
/*==================[macros and definitions]=================================*/

/*==================[inclusions]=============================================*/

#include "sapi_encoder.h"
#include "sapi_gpio.h"

/*==================[macros and definitions]=================================*/

#ifndef EMPTY_POSITION
#define EMPTY_POSITION 255
#endif

#define ENCODER_TOTALNUMBER   4   /* 4 timers                                */

/*==================[internal data declaration]==============================*/

typedef struct{
   TIM_HandleTypeDef*       timer;
   pinInitGpioStm32f1xx_t   encCh12Pin;
} timerEncStmInit_t;

TIM_HandleTypeDef htimer1;
TIM_HandleTypeDef htimer2;
TIM_HandleTypeDef htimer3;
TIM_HandleTypeDef htimer4;

static const timerEncStmInit_t stmEncTimers[] = {
// { timerAddr, { {ch1Port, ch1pin}, ch1mode }, { {ch2Port, ch2pin}, ch2mode },  { {ch3Port, ch3pin}, ch3mode }, { {ch4Port, ch4pin}, ch4mode } },
   {
      &htimer1,
      {{GPIOA, GPIO_PIN_8|GPIO_PIN_8 }, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
   },
   {
      &htimer2,
      {{GPIOA, GPIO_PIN_0|GPIO_PIN_1 }, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
   },
   {
      &htimer3,
      {{GPIOA, GPIO_PIN_6|GPIO_PIN_7 }, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
   },
   {
      &htimer4,
      {{GPIOB, GPIO_PIN_6|GPIO_PIN_7 }, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
   },
};
//
//#define NUMBER_OF_TIMERS (sizeof(stmTimers)/sizeof(timerStmInit_t)) /* calcula automaticamente la cantidad de timers
//                                                                       de acuerdo a lo llenado en el array anterior */
//
//uint32_t channel[]={TIM_CHANNEL_1,TIM_CHANNEL_2,TIM_CHANNEL_3,TIM_CHANNEL_4};
//
///*==================[internal functions declaration]=========================*/
// * @brief:   adds encoder to the the list of working encoders
// * @param:   encoderNumber:   ID of the encoder, check encoderMap_t
// * @return:   True if encoder was successfully attached, False if not.
// */
//static bool_t encoderAttach( encoderMap_t encoderNumber );
//
///*
// * @brief:   removes encoder (attached to encoderNumber) from the list
// * @param:   encoderNumber:   ID of the pwm, check encoderMap_t
// * @return:    True if encoder was successfully detached, False if not.
// */
//static bool_t encoderDetach( encoderMap_t encoderNumber );
//
//
//
/*
 * @Brief: Initializes the encoder peripheral.
 * @param  uint8_t ecoderNumber
 * @param  uint8_t config
 * @return bool_t true (1) if config it is ok
 */
static bool_t enableEncoderFor(encoderMap_t encoderNumber, encoderInit_t config );
///*==================[internal data definition]===============================*/
//
///*==================[external data definition]===============================*/

/*when the user adds a encoder with encoderAttach the list updates with the id number of the element*/
static uint8_t AttachedEncoderList[ENCODER_TOTALNUMBER] = {
   /*Position | Encoder Number*/
   /*0*/  EMPTY_POSITION,
   /*1*/  EMPTY_POSITION,
   /*2*/  EMPTY_POSITION,
   /*3*/  EMPTY_POSITION,
};


///*==================[internal functions definition]==========================*/

static bool_t enableEncoderFor(encoderMap_t encoderNumber, encoderInit_t config ){
    bool_t ret_val = 1;
    TIM_HandleTypeDef *aux = stmEncTimers[encoderNumber].timer;
    TIM_Encoder_InitTypeDef sConfig = { 0 };
    GPIO_InitTypeDef GPIO_InitStruct = { 0 };
    TIM_MasterConfigTypeDef sMasterConfig = { 0 };
    uint32_t chSel;

    switch (encoderNumber) {
    case ENC_TIM1:
        aux->Instance = TIM1;
        __HAL_RCC_TIM1_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
        break;
    case ENC_TIM2:
        aux->Instance = TIM2;
        __HAL_RCC_TIM2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
    case ENC_TIM3:
        aux->Instance = TIM3;
        __HAL_RCC_TIM3_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();
        break;
    case ENC_TIM4:
        aux->Instance = TIM4;
        __HAL_RCC_TIM4_CLK_ENABLE();
        __HAL_RCC_GPIOB_CLK_ENABLE();
        break;
    }

    switch (config) {
    case ENCODER_COUNT_CHANNEL_1:
        sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
        chSel = TIM_CHANNEL_1;
        break;
    case ENCODER_COUNT_CHANNEL_2:
        sConfig.EncoderMode = TIM_ENCODERMODE_TI2;
        chSel = TIM_CHANNEL_2;
        break;
    case ENCODER_COUNT_CHANNEL_ALL:
        sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
        chSel = TIM_CHANNEL_ALL;
        break;
    }

    aux->Init.Prescaler = 0;
    aux->Init.CounterMode = TIM_COUNTERMODE_UP;
    aux->Init.Period = 0xFFFF;
    aux->Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    aux->Init.RepetitionCounter = 0;
    aux->Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 0;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 0;
    if (HAL_TIM_Encoder_Init(aux, &sConfig) != HAL_OK) {
        ret_val = 0;
        return ret_val;
    }
    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(aux, &sMasterConfig) != HAL_OK) {
        ret_val = 0;
        return ret_val;
    }

    GPIO_InitStruct.Pin = stmEncTimers[encoderNumber].encCh12Pin.gpio.pin;
    GPIO_InitStruct.Mode = stmEncTimers[encoderNumber].encCh12Pin.mode;
    HAL_GPIO_Init(stmEncTimers[encoderNumber].encCh12Pin.gpio.port,
            &GPIO_InitStruct);

    HAL_TIM_Encoder_Start(aux, chSel);

    return ret_val;
}
//
///*
// * @brief:   adds encoder to the the list of working encoders
// * @param:   encoderNumber:   ID of the encoder
// * @return:   True if encoder was successfully attached, False if not.
// */
static bool_t encoderAttach( encoderMap_t encoderNumber)
{

   bool_t success = FALSE;
   uint8_t position = 0;

   position = encoderIsAttached(encoderNumber);
   if(position==0) {
      position = encoderIsAttached(EMPTY_POSITION); /* Searches for the first empty position */
      if(position) { /* if position==0 => there is no room in the list for another pwm */
         AttachedEncoderList[position-1] = encoderNumber;
         success = TRUE;
      }
   }
   return success;
}
//
//
///*
// * @brief:   removes encoder (attached to encoderNumber) from the list
// * @param:   emcoderNumber:   ID of the pwm
// * @return:    True if encoder was successfully detached, False if not.
// */
static bool_t encoderDetach( encoderMap_t encoderNumber )
{

   bool_t success = FALSE;
   uint8_t position = 0;

   position = encoderIsAttached(encoderNumber);

   if(position) {
      AttachedEncoderList[position-1] = EMPTY_POSITION;
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
//bool_t pwmWrite( pwmMap_t pwmNumber, uint8_t value )
//{
//   TIM_HandleTypeDef* handle = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;
//   bool_t success = FALSE;
//   uint8_t position = 0;
//   uint16_t pwmPeriod = ((uint16_t)handle->Init.Period*value)/255;
//
//   position = pwmIsAttached(pwmNumber);
//
//   if(position) {
//      __HAL_TIM_SET_COMPARE(handle, channel[pwmNumber/NUMBER_OF_TIMERS], pwmPeriod);
//      success = TRUE;
//   }
//
//   return success;
//}
//
///*
// * @brief:   read the value of the pwm in the pin
// * @param:   pwmNumber:   ID of the pwm, from 0 to 15
// * @return:   value of the pwm in the pin (0 ~ 255).
// *   If an error ocurred, return = EMPTY_POSITION = 255
// */
//bool_t pwmRead( pwmMap_t pwmNumber, uint8_t* rv )
//{
//   TIM_HandleTypeDef* handle = stmTimers[pwmNumber%NUMBER_OF_TIMERS].timer;
//   uint8_t position = 0;
//   position = pwmIsAttached(pwmNumber);
//   uint16_t pwmPeriod = (uint16_t)handle->Init.Period;
//   uint32_t value=0;
//
//   if(position) {
//       value= (__HAL_TIM_GET_COMPARE(handle, channel[pwmNumber/NUMBER_OF_TIMERS]));
//       value*=255;
//       value/=pwmPeriod;
//       if(value)
//           value++;    // por redondeo siempre devuelve una unidad menos
//       *rv = (uint8_t)value;
//   } else {
//       return FALSE;
//   }
//
//   return TRUE;
//}
//
/*
 * @Brief: Initializes the pwm peripheral.
 * @param  encoderMap_t encoderNumber
 * @param  pwmInit_t config
 * @return bool_t true (1) if config it is ok
 */
bool_t encoderInit( encoderMap_t encoderNumber, encoderInit_t config)
{

    bool_t ret_val = 1;

    switch(config){
        case ENCODER_COUNT_CHANNEL_1:
        case ENCODER_COUNT_CHANNEL_2:
        case ENCODER_COUNT_CHANNEL_ALL:
            if(encoderAttach( encoderNumber )){
                ret_val=enableEncoderFor(encoderNumber,config);
            }
            else{
                ret_val=0;
            }
            break;
        case ENCODER_COUNT_DISABLE:
            ret_val = encoderDetach( encoderNumber );
            break;
        default:
            ret_val=0;
            break;
    }

    return ret_val;
}
//
///*
// * @brief:   Tells if the pwm is currently active, and its position
// * @param:   pwmNumber:   ID of the pwm, from 0 to 10
// * @return:   position (1 ~ PWM_TOTALNUMBER), 0 if the element was not found.
// */
uint8_t encoderIsAttached( encoderMap_t encoderNumber )
{
   uint8_t position = 0, positionInList = 0;
   while ( (position < ENCODER_TOTALNUMBER) &&
           (encoderNumber != AttachedEncoderList[position]) ) {
      position++;
   }

   if (position < ENCODER_TOTALNUMBER) {
      positionInList = position + 1;
   } else {
      positionInList = 0;
   }

   return positionInList;
}

///*==================[end of file]============================================*/
