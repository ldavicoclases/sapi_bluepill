/* Copyright 2016, Ian Olivieri
 * Copyright 2016, Eric Pernia.
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
 */

/* Date: 2016-02-10 */

#ifndef _SAPI_PWM_H_
#define _SAPI_PWM_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"

/*==================[c++]====================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros and definitions]=================================*/

#define pwmConfig pwmInit

#define PWM_T1FREC          10000 /* 10Khz */      /*MIN FREC 733Hz */
#define PWM_T2FREC          5000 /* 5000hz */
#define PWM_T3FREC          2500 /* 2500hz */
#define PWM_T4FREC          1000 /* 1000hz */
//#define PWM_PERIOD        1000 /* 1000uS = 1ms*/

#define PWM_T1PERIOD          ((48*1000000U)/PWM_T1FREC)
#define PWM_T2PERIOD          ((48*1000000U)/PWM_T2FREC)
#define PWM_T3PERIOD          ((48*1000000U)/PWM_T3FREC)
#define PWM_T4PERIOD          ((48*1000000U)/PWM_T4FREC)

/*==================[typedef]================================================*/

typedef enum{
   PWM_ENABLE, PWM_DISABLE,
   PWM_ENABLE_OUTPUT, PWM_DISABLE_OUTPUT
} pwmInit_t;

/*==================[external functions declaration]=========================*/

/*
 * @Brief: Initializes the pwm peripheral.
 * @param  uint8_t pwmNumber
 * @param  uint8_t config
 * @return bool_t true (1) if config it is ok
 */
bool_t pwmInit( pwmMap_t pwmNumber, pwmInit_t config);

/*
 * @brief:   Tells if the pwm is currently active, and its position
 * @param:   pwmNumber:   ID of the pwm, from 0 to 10
 * @return:   position (1 ~ PWM_TOTALNUMBER), 0 if the element was not found.
 */
uint8_t pwmIsAttached( pwmMap_t pwmNumber );

/*
 * @brief:   read the value of the pwm in the pin
 * @param:   pwmNumber:   ID of the pwm, from 0 to 10
 * @return:   value of the pwm in the pin (0 ~ 255).
 *   If an error ocurred, return = EMPTY_POSITION = 255
 */
bool_t pwmRead( pwmMap_t pwmNumber, uint8_t* rv );

/*
 * @brief:   change the value of the pwm at the selected pin
 * @param:   pwmNumber:   ID of the pwm, from 0 to 10
 * @param:   value:   8bit value, from 0 to 255
 * @return:   True if the value was successfully changed, False if not.
 */
bool_t pwmWrite( pwmMap_t pwmNumber, uint8_t percent );

bool_t EnablePwmfor(pwmMap_t pwmNumber);

/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_PWM_H_ */
