/* Copyright 2015, Eric Pernia.
 * Copyright 2016, Ian Olivieri.
 * Copyright 2016, Eric Pernia.
 * Copyright 2020, Nahuel Espinosa.
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
 */

/* Date: 2020-06-08 */

#ifndef _SAPI_PERIPHERALMAP_H_
#define _SAPI_PERIPHERALMAP_H_

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "stm32f1xx_hal.h"

/*==================[c++]====================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[typedef]================================================*/


/* ------- Begin STM32F106C8 BLUEPILL Peripheral Map ------ */

/* Defined for sapi_gpio.h */

typedef enum {

   VCC = -2, GND = -1,

   // Left header
   PB12, PB13, PB14, PB15, PA8 , PA9 , PA10, PA11, PA12, PA15, PB3 , PB4 , PB5 , PB6 , PB7 , PB8 , PB9,

   // Right header
   PB11, PB10, PB1 , PB0 , PA7 , PA6 , PA5 , PA4 , PA3 , PA2 , PA1 , PA0 , PC15, PC14, PC13

} gpioMap_t;

#define BTN		RST
#define LED		PC13

/* Defined for sapi_adc.h */
typedef enum {
   CH0 = 0,
   CH1 = 1,
   CH2 = 2,
   CH3 = 3,
} adcMap_t;

/* Defined for sapi_dac.h */
typedef enum {
   A0 = 0,
} dacMap_t;

/* Defined for sapi_uart.h */
typedef enum {
   UART_1,
   UART_2,
   UART_3,
   UART_USB,
   UART_MAXNUM,
} uartMap_t;

/*Defined for sapi_timer.h*/
typedef enum {
   TIMER0, TIMER1, TIMER2, TIMER3
} timerMap_t;
typedef enum {
   TIMERCOMPAREMATCH0, TIMERCOMPAREMATCH1, TIMERCOMPAREMATCH2, TIMERCOMPAREMATCH3
} timerCompareMatch_t;

/*Defined for sapi_sct.h*/
typedef enum {
   CTOUT0, CTOUT1, CTOUT2, CTOUT3, CTOUT4, CTOUT5, CTOUT6, CTOUT7, CTOUT8,
   CTOUT9, CTOUT10, CTOUT11, CTOUT12, CTOUT13
} sctMap_t;

/*Defined for sapi_pwm.h*/
typedef enum {
   TIM1_CH1, TIM2_CH1, TIM3_CH1, TIM4_CH1,
   TIM1_CH2, TIM2_CH2, TIM3_CH2, TIM4_CH2,
   TIM1_CH3, TIM2_CH3, TIM3_CH3, TIM4_CH3,
   TIM1_CH4, TIM2_CH4, TIM3_CH4, TIM4_CH4
} pwmMap_t;

/*Defined for sapi_servo.h*/
typedef enum {
   SERVO0, SERVO1, SERVO2, SERVO3, SERVO4, SERVO5, SERVO6, SERVO7, SERVO8
} servoMap_t;

typedef enum {
   I2C_1,
   I2C_2
} i2cMap_t;

typedef enum {
   SPI_1,
   SPI_2
} spiMap_t;

/*Defined for sapi_encoder.h*/
typedef enum {
   ENC_TIM1,
   ENC_TIM2,
   ENC_TIM3,
   ENC_TIM4
} encoderMap_t;

/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_PERIPHERALMAP_H_ */
