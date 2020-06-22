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
 */

/* Date: 2020-06-21 */


#ifndef _SAPI_ENCODER_H_
#define _SAPI_ENCODER_H_


/*==================[   WARNING!!!   ]=======================================*/
/*      This peripheral share the hardware timers with PWM peripheral.
 *      You should not use an PWM timer and same Encoder Timer at once
 *      Is user's responsibility to check that.
 */

/*==================[inclusions]=============================================*/

#include "sapi_datatypes.h"
#include "sapi_peripheral_map.h"

/*==================[c++]====================================================*/
#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros and definitions]=================================*/

#define encoderConfig encoderInit


/*==================[typedef]================================================*/

/* This option changes the count method
*
*      _____________________________________________________________________________________________________
*      | Option             | Opposite Signal                |      TI1Signal       |       TI2Signal       |
*      |                    |(TI2 for CH1 or TI1 for CH2)    |______________________|_______________________|
*      |                    |                                |  RISING  |   FALLING |   RISING  |   FALLING |
*      |____________________|________________________________|__________|___________|___________|___________|
*      |                    |                                |          |           |           |           |
*      |                    |               HIGH             |    DOWN  |     UP    |  NO COUNT | NO COUNT  |
*      |    Counting on CH1 |________________________________|__________|___________|___________|___________|
*      |        Only        |                                |          |           |           |           |
*      |                    |               LOW              |    UP    |    DOWN   |  NO COUNT | NO_COUNT  |
*      |____________________|________________________________|__________|___________|___________|___________|
*      |                    |                                |          |           |           |           |
*      |                    |               HIGH             | NO_COUNT | NO_COUNT  |     UP    |   DOWN    |
*      |    Counting on CH2 |________________________________|__________|___________|___________|___________|
*      |        Only        |                                |          |           |           |           |
*      |                    |               LOW              | NO_COUNT | NO_COUNT  |    DOWN   |    UP     |
*      |____________________|________________________________|__________|___________|___________|___________|
*      |                    |                                |          |           |           |           |
*      |                    |               HIGH             |    DOWN  |     UP    |     UP    |   DOWN    |
*      |    Counting on CH1 |________________________________|__________|___________|___________|___________|
*      |      and CH2       |                                |          |           |           |           |
*      |                    |               LOW              |    UP    |    DOWN   |    DOWN   |    UP     |
*      |____________________|________________________________|__________|___________|___________|___________|
 *
 */

typedef enum{
    ENCODER_COUNT_CHANNEL_1,
    ENCODER_COUNT_CHANNEL_2,
    ENCODER_COUNT_CHANNEL_ALL,  //count on both channels
    ENCODER_COUNT_DISABLE
} encoderInit_t;

/*==================[external functions declaration]=========================*/

/*
 * @Brief: Initializes the encoder peripheral.
 * @param  uint8_t ecoderNumber
 * @param  uint8_t config
 * @return bool_t true (1) if config it is ok
 */
bool_t encoderInit( encoderMap_t encoderNumber, encoderInit_t config);

/*
 * @brief:   Tells if the encoder is currently active, and its position
 * @param:   encoderNumber:   ID of the encoder, see encoderMap_t
 * @return:   position (1 ~ ENCODER_TOTALNUMBER), 0 if the element was not found.
 */
uint8_t encoderIsAttached( encoderMap_t encoderNumber );


/*
 * @brief:   read the value of the encoder in the timer
 * @param:   encoderNumber:   ID of the encoder, see in encoderMap_t
 * @param:   rv:   Pointer to variable where the reading will be stored
 * @return:  bool_t true (1) if reading  is ok, false if encoder is not attached
 *
 */
bool_t encoderRead( encoderMap_t encoderNumber, uint16_t* rv );

/*
 * @brief:   change the value of the encoder at the correspondent timer
 * @param:   encoderNumber:   ID of the encoder, see in encoderMap_t
 * @param:   value:   16bit value, from 0 to 65535
 * @return:   True if the value was successfully changed, False if not.
 */
bool_t encoderWrite( encoderMap_t encoderNumber, uint16_t value );

/*==================[c++]====================================================*/
#ifdef __cplusplus
}
#endif

/*==================[end of file]============================================*/
#endif /* _SAPI_ENCODER_H_ */
