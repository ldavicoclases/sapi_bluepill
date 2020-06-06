/* Copyright 2016, Eric Pernia.
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

/*==================[inclusions]=============================================*/

#include "sapi_spi.h"
#include "sapi_gpio.h"

/*==================[macros and definitions]=================================*/

/*==================[typedef]================================================*/

typedef struct{
   SPI_HandleTypeDef*       spi;
   pinInitGpioStm32f1xx_t   mosiPin;
   pinInitGpioStm32f1xx_t   misoPin;
   pinInitGpioStm32f1xx_t   clkPin;
} spiStmInit_t;

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;

static const spiStmInit_t stmSpi[] = {
// { spiAddr, { {mosiPort, mosiPin}, mosiMode }, { {misoPort, misoPin}, misoMode }, { {clkPort, clkPin}, clkMode } },
   {
      &hspi1,
      {{GPIOA, GPIO_PIN_7 }, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
      {{GPIOA, GPIO_PIN_6 }, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      {{GPIOA, GPIO_PIN_5 }, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}
   },
   {
      &hspi2,
      {{GPIOB, GPIO_PIN_15}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
      {{GPIOB, GPIO_PIN_14}, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      {{GPIOB, GPIO_PIN_13}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH}
   }
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

bool_t spiInit( spiMap_t spi )
{
   bool_t retVal = TRUE;
   GPIO_InitTypeDef GPIO_InitStruct = {0};

   if( spi == SPI_1 ) {
      __HAL_RCC_SPI1_CLK_ENABLE();
      __HAL_RCC_GPIOA_CLK_ENABLE();
      /**SPI1 GPIO Configuration
      PA5     ------> SPI1_SCK
      PA6     ------> SPI1_MISO
      PA7     ------> SPI1_MOSI
      */

      stmSpi[spi].spi->Instance = SPI1;

   } else if( spi == SPI_2 ) {
      __HAL_RCC_SPI2_CLK_ENABLE();
      __HAL_RCC_GPIOB_CLK_ENABLE();
      /**SPI2 GPIO Configuration
      PB13     ------> SPI2_SCK
      PB14     ------> SPI2_MISO
      PB15     ------> SPI2_MOSI
      */

      stmSpi[spi].spi->Instance = SPI2;

   } else {
      retVal = FALSE;
      return retVal;
   }

   GPIO_InitStruct.Pin   = stmSpi[spi].mosiPin.gpio.pin;
   GPIO_InitStruct.Mode  = stmSpi[spi].mosiPin.mode;
   GPIO_InitStruct.Pull  = stmSpi[spi].mosiPin.pull;
   GPIO_InitStruct.Speed = stmSpi[spi].mosiPin.speed;
   HAL_GPIO_Init(stmSpi[spi].mosiPin.gpio.port, &GPIO_InitStruct);

   GPIO_InitStruct.Pin   = stmSpi[spi].misoPin.gpio.pin;
   GPIO_InitStruct.Mode  = stmSpi[spi].misoPin.mode;
   GPIO_InitStruct.Pull  = stmSpi[spi].misoPin.pull;
   GPIO_InitStruct.Speed = stmSpi[spi].misoPin.speed;
   HAL_GPIO_Init(stmSpi[spi].misoPin.gpio.port, &GPIO_InitStruct);

   GPIO_InitStruct.Pin   = stmSpi[spi].clkPin.gpio.pin;
   GPIO_InitStruct.Mode  = stmSpi[spi].clkPin.mode;
   GPIO_InitStruct.Pull  = stmSpi[spi].clkPin.pull;
   GPIO_InitStruct.Speed = stmSpi[spi].clkPin.speed;
   HAL_GPIO_Init(stmSpi[spi].clkPin.gpio.port, &GPIO_InitStruct);

   stmSpi[spi].spi->Init.Mode = SPI_MODE_MASTER;
   stmSpi[spi].spi->Init.Direction = SPI_DIRECTION_2LINES;
   stmSpi[spi].spi->Init.DataSize = SPI_DATASIZE_8BIT;
   stmSpi[spi].spi->Init.CLKPolarity = SPI_POLARITY_LOW;
   stmSpi[spi].spi->Init.CLKPhase = SPI_PHASE_1EDGE;
   stmSpi[spi].spi->Init.NSS = SPI_NSS_SOFT;
   stmSpi[spi].spi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
   stmSpi[spi].spi->Init.FirstBit = SPI_FIRSTBIT_MSB;
   stmSpi[spi].spi->Init.TIMode = SPI_TIMODE_DISABLE;
   stmSpi[spi].spi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
   stmSpi[spi].spi->Init.CRCPolynomial = 10;
   HAL_SPI_Init(stmSpi[spi].spi);

   return retVal;
}


bool_t spiRead( spiMap_t spi, uint8_t* buffer, uint32_t bufferSize )
{

   bool_t retVal = FALSE;

   if( spi == SPI_1 || spi == SPI_2 ) {
       if( HAL_SPI_Receive(stmSpi[spi].spi, buffer, bufferSize, 10) == HAL_OK ) {
           retVal = TRUE;
       }
   }

   return retVal;
}


bool_t spiWrite( spiMap_t spi, uint8_t* buffer, uint32_t bufferSize )
{

   bool_t retVal = FALSE;

   if( spi == SPI_1 || spi == SPI_2 ) {
       if( HAL_SPI_Transmit(stmSpi[spi].spi, buffer, bufferSize, 10) == HAL_OK ) {
           retVal = TRUE;
       }
   }

   return retVal;
}


bool_t spiWriteRead( spiMap_t spi, uint8_t* txBuffer, uint8_t* rxBuffer, uint32_t bufferSize )
{
    bool_t retVal = FALSE;

    if( spi == SPI_1 || spi == SPI_2 ) {
        if( HAL_SPI_TransmitReceive(stmSpi[spi].spi, txBuffer, rxBuffer, bufferSize, 10) == HAL_OK ) {
            retVal = TRUE;
        }
    }

    return retVal;
}


/*==================[ISR external functions definition]======================*/



/** @} doxygen end group definition */
/*==================[end of file]============================================*/
