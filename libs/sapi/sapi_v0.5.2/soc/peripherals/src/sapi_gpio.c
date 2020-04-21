/* Copyright 2015-2016, Eric Pernia.
 * Copyright 2020, Nahuel Espinosa
 * All rights reserved.
 *
 * This file is part of CIAA Firmware.
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

/* Date: 2020-04-21 */

/*==================[inclusions]=============================================*/

#include "sapi_gpio.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

const pinInitGpioStm32f1xx_t gpioPinsInit[] = {

	/*{ {PORT, PIN}, MODE, PULL, SPEED }*/
   { {GPIOB, GPIO_PIN_12}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB12                 */
   { {GPIOB, GPIO_PIN_13}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB13                 */
   { {GPIOB, GPIO_PIN_14}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB14                 */
   { {GPIOB, GPIO_PIN_15}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB15                 */
   { {GPIOA, GPIO_PIN_8 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA8                  */
   { {GPIOA, GPIO_PIN_9 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA9                  */
   { {GPIOA, GPIO_PIN_10}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA10                 */
   { {GPIOA, GPIO_PIN_11}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA11                 */
   { {GPIOA, GPIO_PIN_12}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA12                 */
   { {GPIOA, GPIO_PIN_15}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA15                 */
   { {GPIOB, GPIO_PIN_3 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB3                  */
   { {GPIOB, GPIO_PIN_4 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB4                  */
   { {GPIOB, GPIO_PIN_5 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB5                  */
   { {GPIOB, GPIO_PIN_6 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB6                  */
   { {GPIOB, GPIO_PIN_7 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB7                  */
   { {GPIOB, GPIO_PIN_8 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB8                  */
   { {GPIOB, GPIO_PIN_9 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB9                  */

   { {GPIOB, GPIO_PIN_11}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB11                 */
   { {GPIOB, GPIO_PIN_10}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB10                 */
   { {GPIOB, GPIO_PIN_1 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB1                  */
   { {GPIOB, GPIO_PIN_0 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PB0                  */
   { {GPIOA, GPIO_PIN_7 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA7                  */
   { {GPIOA, GPIO_PIN_6 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA6                  */
   { {GPIOA, GPIO_PIN_5 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA5                  */
   { {GPIOA, GPIO_PIN_4 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA4                  */
   { {GPIOA, GPIO_PIN_3 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA3                  */
   { {GPIOA, GPIO_PIN_2 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA2                  */
   { {GPIOA, GPIO_PIN_1 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA1                  */
   { {GPIOA, GPIO_PIN_0 }, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PA0                  */
   { {GPIOC, GPIO_PIN_15}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PC15                 */
   { {GPIOC, GPIO_PIN_14}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PC14                 */
   { {GPIOC, GPIO_PIN_13}, GPIO_MODE_INPUT    , GPIO_NOPULL  , GPIO_SPEED_FREQ_LOW },     /*   PC13                 */
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

bool_t gpioInit( gpioMap_t pin, gpioInit_t config )
{
   if( pin == VCC ){
	  return FALSE;
   }
   if( pin == GND ){
	  return FALSE;
   }

   GPIO_InitTypeDef GPIO_InitStruct;

   bool_t ret_val = 1;

   GPIO_TypeDef * gpioPort = gpioPinsInit[pin].gpio.port;
   uint16_t       gpioPin  = gpioPinsInit[pin].gpio.pin;
   int8_t         mode     = gpioPinsInit[pin].mode;
   int8_t         pull     = gpioPinsInit[pin].pull;
   int8_t         speed    = gpioPinsInit[pin].speed;

   switch(config){
	  case GPIO_INPUT:
         GPIO_InitStruct.Pin   = gpioPin;
         GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
         GPIO_InitStruct.Pull  = GPIO_NOPULL;
         GPIO_InitStruct.Speed = speed;
         HAL_GPIO_Init( gpioPort, &GPIO_InitStruct);
         break;

      case GPIO_INPUT_PULLUP:
         GPIO_InitStruct.Pin   = gpioPin;
         GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
         GPIO_InitStruct.Pull  = GPIO_PULLUP;
         GPIO_InitStruct.Speed = speed;
         HAL_GPIO_Init( gpioPort, &GPIO_InitStruct);
         break;

      case GPIO_INPUT_PULLDOWN:
         GPIO_InitStruct.Pin   = gpioPin;
         GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
         GPIO_InitStruct.Pull  = GPIO_PULLDOWN;
         GPIO_InitStruct.Speed = speed;
         HAL_GPIO_Init( gpioPort, &GPIO_InitStruct);
         break;

      case GPIO_INPUT_PULLUP_PULLDOWN:
         GPIO_InitStruct.Pin   = gpioPin;
         GPIO_InitStruct.Mode  = GPIO_MODE_INPUT;
         GPIO_InitStruct.Pull  = GPIO_PULLDOWN | GPIO_PULLUP;
         GPIO_InitStruct.Speed = speed;
         HAL_GPIO_Init( gpioPort, &GPIO_InitStruct);
         break;

      case GPIO_OUTPUT:
         GPIO_InitStruct.Pin   = gpioPin;
         GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
         GPIO_InitStruct.Pull  = GPIO_NOPULL;
         GPIO_InitStruct.Speed = speed;
         HAL_GPIO_Init( gpioPort, &GPIO_InitStruct);
         break;

      case GPIO_ALT_FUNCTION:
         GPIO_InitStruct.Pin   = gpioPin;
         GPIO_InitStruct.Mode  = mode;
         GPIO_InitStruct.Pull  = pull;
         GPIO_InitStruct.Speed = speed;
         HAL_GPIO_Init( gpioPort, &GPIO_InitStruct);
         break;

      default:
         ret_val = 0;
         break;
   }

   return ret_val;
}

bool_t gpioWrite( gpioMap_t pin, bool_t value )
{
   if( pin == VCC ){
	  return FALSE;
   }
   if( pin == GND ){
	  return FALSE;
   }

   bool_t ret_val = 1;

   GPIO_TypeDef * gpioPort = gpioPinsInit[pin].gpio.port;
   uint16_t       gpioPin  = gpioPinsInit[pin].gpio.pin;

   HAL_GPIO_WritePin(gpioPort, gpioPin, value);

   return ret_val;
}

bool_t gpioToggle( gpioMap_t pin )
{
   if( pin == VCC ){
	  return FALSE;
   }
   if( pin == GND ){
	  return FALSE;
   }

   bool_t ret_val = 1;

   GPIO_TypeDef * gpioPort = gpioPinsInit[pin].gpio.port;
   uint16_t       gpioPin  = gpioPinsInit[pin].gpio.pin;

   HAL_GPIO_TogglePin(gpioPort, gpioPin);

   return ret_val;
}

bool_t gpioRead( gpioMap_t pin )
{
   if( pin == VCC ){
      return TRUE;
   }
   if( pin == GND ){
      return FALSE;
   }

   bool_t ret_val = 1;

   GPIO_TypeDef * gpioPort = gpioPinsInit[pin].gpio.port;
   uint16_t gpioPin        = gpioPinsInit[pin].gpio.pin;

   ret_val = (bool_t) HAL_GPIO_ReadPin(gpioPort, gpioPin);

   return ret_val;
}

/*==================[end of file]============================================*/
