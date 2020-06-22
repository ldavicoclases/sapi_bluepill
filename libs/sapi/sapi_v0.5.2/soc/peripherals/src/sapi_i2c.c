/* Copyright 2016, Eric Pernia
 * Copyright 2016, Alejandro Permingeat.
 * Copyright 2016, Eric Pernia
 * Copyright 2020, Guillermo Ferrari
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

/*
 * Date:
 * 2016-05-02 Eric Pernia - Only define API
 * 2016-06-23 Alejandro Permingeat - First functional version
 * 2016-08-07 Eric Pernia - Improve names
 * 2016-09-10 Eric Pernia - Add unlimited buffer transfer
 * 2016-11-20 Eric Pernia - Software I2C
 * 2020-06-15 Guillermo Ferrari - stm32f103c8t6 Adapted
 */

/*==================[inclusions]=============================================*/

#include "sapi_i2c.h"
#include "sapi_gpio.h"
#include "sapi_delay.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
typedef struct{
   I2C_HandleTypeDef*       i2c;
   pinInitGpioStm32f1xx_t   SDAPin;
   pinInitGpioStm32f1xx_t   SCLPin;
} I2CStmInit_t;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

static const I2CStmInit_t stmI2Cs[] = {

        &hi2c1,
        {{GPIOB, GPIO_PIN_7 }, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH},
        {{GPIOB, GPIO_PIN_6}, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH},

        &hi2c2,
        {{GPIOB, GPIO_PIN_11 }, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH},
        {{GPIOB, GPIO_PIN_10}, GPIO_MODE_AF_OD, GPIO_PULLUP, GPIO_SPEED_FREQ_HIGH},

};

/*==================[internal functions declaration]=========================*/

/*This four functions works for "fix" busy flag bug in HAL I2C Initialization*/

static HAL_StatusTypeDef I2C_ClearBusyFlagErrata_2_14_7(i2cMap_t i2cNumber);

static void saveI2CRegisters(I2C_HandleTypeDef* hi2c,uint32_t* regs);

static void restoreI2CRegisters(I2C_HandleTypeDef* hi2c,uint32_t* regs);

static void HAL_GPIO_WRITE_ODR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin);

static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnTXEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_WaitOnBTFFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
static HAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c);
static HAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart);
#if( I2C_SOFTWARE == 1 )

static bool_t i2cSoftwareInit( i2cMap_t i2cNumber, uint32_t clockRateHz );

static bool_t i2cSoftwareRead( i2cMap_t  i2cNumber,
                               uint8_t  i2cSlaveAddress,
                               uint8_t* dataToReadBuffer,
                               uint16_t dataToReadBufferSize,
                               bool_t   sendWriteStop,
                               uint8_t* receiveDataBuffer,
                               uint16_t receiveDataBufferSize,
                               bool_t   sendReadStop );

static bool_t i2cSoftwareWrite( i2cMap_t  i2cNumber,
                                uint8_t  i2cSlaveAddress,
                                uint8_t* transmitDataBuffer,
                                uint16_t transmitDataBufferSize,
                                bool_t   sendWriteStop );

static void i2cSoftwarePinInit( gpioMap_t pin, uint8_t mode );
static void i2cSoftwarePinWrite( gpioMap_t pin, bool_t value );
static bool_t i2cSoftwarePinRead( gpioMap_t pin );

#else

static bool_t i2cHardwareInit( i2cMap_t i2cNumber, uint32_t clockRateHz );

static bool_t i2cHardwareRead( i2cMap_t  i2cNumber,
                               uint8_t  i2cSlaveAddress,
                               uint8_t* dataToReadBuffer,
                               uint16_t dataToReadBufferSize,
                               bool_t   sendWriteStop,
                               uint8_t* receiveDataBuffer,
                               uint16_t receiveDataBufferSize,
                               bool_t   sendReadStop );

static bool_t i2cHardwareWrite( i2cMap_t  i2cNumber,
                                uint8_t  i2cSlaveAddress,
                                uint8_t* transmitDataBuffer,
                                uint16_t transmitDataBufferSize,
                                bool_t   sendWriteStop );

#endif

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

#if( I2C_SOFTWARE == 1 )

static bool_t i2cSoftwareInit( i2cMap_t i2cNumber, uint32_t clockRateHz )
{

   bool_t retVal = TRUE;

   i2cSoftwarePinInit( I2C_SOFTWARE_SDA_DIR, GPIO_INPUT_PULLUP );
   i2cSoftwarePinInit( I2C_SOFTWARE_SCL_DIR, GPIO_INPUT_PULLUP );

   return retVal;
}

static bool_t i2cSoftwareRead( i2cMap_t  i2cNumber,
                               uint8_t  i2cSlaveAddress,
                               uint8_t* dataToReadBuffer,
                               uint16_t dataToReadBufferSize,
                               bool_t   sendWriteStop,
                               uint8_t* receiveDataBuffer,
                               uint16_t receiveDataBufferSize,
                               bool_t   sendReadStop )
{

   bool_t retVal = TRUE;
   uint16_t i = 0;

   // Check Errors
   if( (dataToReadBuffer == NULL)  || (dataToReadBufferSize < 0) ||
       (receiveDataBuffer == NULL) || (receiveDataBufferSize <= 0) ) {
      return FALSE;
   }

   // First Write

   if( dataToReadBufferSize > 0 ) {
      retVal &= i2cSoftwareWrite( i2cNumber,
                                  i2cSlaveAddress,
                                  dataToReadBuffer,
                                  dataToReadBufferSize,
                                  sendWriteStop );
   }

   // Then Read

   // Start condition
   i2cSoftwareMasterWriteStart();
   // 7 bit address + Read = 1
   i2cSoftwareMasterWriteAddress( i2cSlaveAddress, I2C_SOFTWARE_READ );
   // Write all data buffer
   for( i=0; i<receiveDataBufferSize; i++ ) {
      receiveDataBuffer[i] = i2cSoftwareMasterReadByte( TRUE ); // TRUE send ACK, FALSE not
   }
   // Send Stop condition
   if( sendReadStop ) {
      i2cSoftwareMasterWriteStop();
   }
   return retVal;
}

static bool_t i2cSoftwareWrite( i2cMap_t  i2cNumber,
                                uint8_t  i2cSlaveAddress,
                                uint8_t* transmitDataBuffer,
                                uint16_t transmitDataBufferSize,
                                bool_t   sendWriteStop )
{

   bool_t retVal = TRUE;
   uint16_t i = 0;

   // Check Errors
   if( (transmitDataBuffer == NULL) || (transmitDataBufferSize <= 0) ) {
      return FALSE;
   }
   // Start condition
   i2cSoftwareMasterWriteStart();
   // 7 bit address + Write = 0
   i2cSoftwareMasterWriteAddress( i2cSlaveAddress, I2C_SOFTWARE_WRITE );
   // Write all data buffer
   for( i=0; i<transmitDataBufferSize; i++ ) {
      i2cSoftwareMasterWriteByte( transmitDataBuffer[i] );
   }
   // Send Stop condition
   if(sendWriteStop) {
      i2cSoftwareMasterWriteStop();
   }

   return retVal;
}


// Point of contact with sapi_gpio module

static void i2cSoftwarePinInit( uint8_t pin, uint8_t mode )
{

   if( pin == I2C_SOFTWARE_SDA_DIR ) {
      if( mode == GPIO_OUTPUT ) {
         //IO_DIR_PORT_PIN( OCM_DATA_PORT, OCM_DATA_PIN, IO_OUT );
         gpioInit( I2C_SOFTWARE_SDA_DIR, GPIO_OUTPUT );
      } else if( mode == GPIO_INPUT_PULLUP ) {
         // Seteo de pines como ENTRADA
         //IO_DIR_PORT_PIN( OCM_DATA_PORT, OCM_DATA_PIN, IO_IN );
         // Seteo de pines con pull-up
         //IO_PUD_PORT( OCM_DATA_PORT, IO_PUP );
         gpioInit( I2C_SOFTWARE_SDA_DIR, GPIO_INPUT );
      }
   } else if( pin == I2C_SOFTWARE_SCL_DIR ) {
      if( mode == GPIO_OUTPUT ) {
         //IO_DIR_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_OUT );
         gpioInit( I2C_SOFTWARE_SCL_DIR, GPIO_OUTPUT );
      } else if( mode == GPIO_INPUT_PULLUP ) {
         // Seteo de pines como ENTRADA
         //IO_DIR_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_IN );
         // Seteo de pines con pull-up
         //IO_PUD_PORT( OCM_CLK_PORT, IO_PUP );
         gpioInit( I2C_SOFTWARE_SCL_DIR, GPIO_INPUT );
      }
   }

}
static void i2cSoftwarePinWrite( uint8_t pin, bool_t value )
{

   if( pin == I2C_SOFTWARE_SDA_OUT ) {
      gpioWrite( I2C_SOFTWARE_SDA_OUT, value );
   } else if( pin == I2C_SOFTWARE_SCL_OUT ) {
      if(value) {
         //IO_DIR_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_IN );
         gpioInit( I2C_SOFTWARE_SCL_DIR, GPIO_INPUT );
         while( !gpioRead( I2C_SOFTWARE_SCL_IN ) );   // Espera hasta que el clock este en alto
         i2cSoftwareDelay(1); // 1 clock time delay
      } else {
         //IO_DIR_PORT_PIN( OCM_CLK_PORT, OCM_CLK_PIN, IO_OUT );
         //OCM_SCL = 0;                //Setea el clock a LOW
         gpioInit( I2C_SOFTWARE_SCL_DIR, GPIO_OUTPUT );
         gpioWrite( I2C_SOFTWARE_SCL_OUT, LOW );
      }
      // 1 clock time delay
      i2cSoftwareDelay(1);
   }
}


static bool_t i2cSoftwarePinRead( uint8_t pin )
{

   bool_t retVal = 0;

   retVal = gpioRead( (int8_t)pin );
   return retVal;
}
#else

static bool_t i2cHardwareInit( i2cMap_t i2cNumber, uint32_t clockRateHz )
{

    I2C_HandleTypeDef* aux= stmI2Cs[i2cNumber].i2c;
    GPIO_InitTypeDef GPIO_InitStruct = {0};
    uint32_t regs[4];

    switch(i2cNumber){
        case I2C_1:
            aux->Instance= I2C1;
            __HAL_RCC_I2C1_CLK_ENABLE();
            break;
        case I2C_2:
            aux->Instance=I2C2;
            __HAL_RCC_I2C2_CLK_ENABLE();
            break;
    }
    aux->Instance = I2C1;
    aux->Init.ClockSpeed = clockRateHz;
    aux->Init.DutyCycle = I2C_DUTYCYCLE_2;
    aux->Init.OwnAddress1 = 0;
    aux->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    aux->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    aux->Init.OwnAddress2 = 0;
    aux->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    aux->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(aux) != HAL_OK)
    {
      Error_Handler();
    }



    __HAL_RCC_GPIOB_CLK_ENABLE();

    GPIO_InitStruct.Pin = stmI2Cs[i2cNumber].SCLPin.gpio.pin;
    GPIO_InitStruct.Mode = stmI2Cs[i2cNumber].SCLPin.mode;
    GPIO_InitStruct.Pull = stmI2Cs[i2cNumber].SCLPin.pull;
    GPIO_InitStruct.Speed = stmI2Cs[i2cNumber].SCLPin.speed;
    HAL_GPIO_Init(stmI2Cs[i2cNumber].SCLPin.gpio.port, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = stmI2Cs[i2cNumber].SDAPin.gpio.pin;
    GPIO_InitStruct.Mode = stmI2Cs[i2cNumber].SDAPin.mode;
    GPIO_InitStruct.Pull = stmI2Cs[i2cNumber].SDAPin.pull;
    GPIO_InitStruct.Speed = stmI2Cs[i2cNumber].SDAPin.speed;
    HAL_GPIO_Init(stmI2Cs[i2cNumber].SDAPin.gpio.port, &GPIO_InitStruct);

    HAL_Delay(200); // necesario para que lo que sigue no se cuelgue


    saveI2CRegisters(aux,regs); //salva los registros

    if(I2C_ClearBusyFlagErrata_2_14_7(i2cNumber)!=HAL_OK){  //arregla un bug de la inicializacion del I2C pero deja los registros en cualquier valor
        return FALSE;
    }

    restoreI2CRegisters(aux,regs); //recupera los registros


//   // Configuracion de las lineas de SDA y SCL de la placa
//   Chip_SCU_I2C0PinConfig( I2C0_STANDARD_FAST_MODE ); // Equal for CIAA-NXP and EDU-CIAA-NXP on I2C0
//
//   // Inicializacion del periferico
//   Chip_I2C_Init( i2cNumber );
//   // Seleccion de velocidad del bus
//   Chip_I2C_SetClockRate( i2cNumber, clockRateHz );
//   // Configuracion para que los eventos se resuelvan por polliong
//   // (la otra opcion es por interrupcion)
//   Chip_I2C_SetMasterEventHandler( i2cNumber, Chip_I2C_EventHandlerPolling );

   return TRUE;
}

static bool_t i2cHardwareRead( i2cMap_t  i2cNumber,
                               uint8_t  i2cSlaveAddress,
                               uint8_t* dataToReadBuffer,
                               uint16_t dataToReadBufferSize,
                               bool_t   sendWriteStop,
                               uint8_t* receiveDataBuffer,
                               uint16_t receiveDataBufferSize,
                               bool_t   sendReadStop )
{

    I2C_HandleTypeDef* hi2c = stmI2Cs[i2cNumber].i2c;
    bool_t retVal = TRUE;
    uint32_t tickstart = HAL_GetTick(), Timeout = I2C_DEFAULT_TIMEOUT;


    // Check Errors
    if( (dataToReadBuffer == NULL)  || (dataToReadBufferSize < 0) ||
        (receiveDataBuffer == NULL) || (receiveDataBufferSize <= 0) ) {
       return FALSE;
    }

    if (hi2c->State == HAL_I2C_STATE_READY)
     {
       /* Wait until BUSY flag is reset */
       if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
       {
         return HAL_BUSY;
       }

       /* Process Locked */
       __HAL_LOCK(hi2c);

       /* Check if the I2C is already enabled */
       if ((hi2c->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
       {
         /* Enable I2C peripheral */
         __HAL_I2C_ENABLE(hi2c);
       }

       /* Disable Pos */
       CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

       hi2c->State     = HAL_I2C_STATE_BUSY_RX;
       hi2c->Mode      = HAL_I2C_MODE_MEM;
       hi2c->ErrorCode = HAL_I2C_ERROR_NONE;

       /* Prepare transfer parameters */
       hi2c->pBuffPtr    = receiveDataBuffer;
       hi2c->XferCount   = receiveDataBufferSize;
       hi2c->XferSize    = hi2c->XferCount;
       hi2c->XferOptions = I2C_NO_OPTION_FRAME;

       if( dataToReadBufferSize > 0 ) {
           /* Enable Acknowledge */
           SET_BIT(hi2c->Instance->CR1, I2C_CR1_ACK);

           /* Generate Start */
           SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);

           /* Wait until SB flag is set */
           if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, tickstart) != HAL_OK)
           {
             return HAL_ERROR;
           }

           /* Send slave address */
           hi2c->Instance->DR = I2C_7BIT_ADD_WRITE(i2cSlaveAddress);

           /* Wait until ADDR flag is set */
           if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout, tickstart) != HAL_OK)
           {
             return HAL_ERROR;
           }

           /* Clear ADDR flag */
           __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

           /* Wait until TXE flag is set */
           if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
           {
             if (hi2c->ErrorCode == HAL_I2C_ERROR_AF)
             {
               /* Generate Stop */
               SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
             }
             return HAL_ERROR;
           }
           for(uint16_t i=0;i<dataToReadBufferSize;i++){
           /* Send MSB of Memory Address */
               hi2c->Instance->DR = I2C_MEM_ADD_LSB(dataToReadBuffer[i]);

               /* Wait until TXE flag is set */
               if (I2C_WaitOnTXEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
               {
                 if (hi2c->ErrorCode == HAL_I2C_ERROR_AF)
                 {
                   /* Generate Stop */
                   SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
                 }
                 return HAL_ERROR;
               }
           }

           if(sendWriteStop){
           /* Generate Stop */
           SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
           }
       }
       /* Generate start */
       SET_BIT(hi2c->Instance->CR1, I2C_CR1_START);

       /* Wait until SB flag is set */
       if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_SB, RESET, Timeout, tickstart) != HAL_OK)
       {
         return HAL_ERROR;
       }

       /* Send slave address */
       hi2c->Instance->DR = I2C_7BIT_ADD_READ(i2cSlaveAddress);

       /* Wait until ADDR flag is set */
       if (I2C_WaitOnMasterAddressFlagUntilTimeout(hi2c, I2C_FLAG_ADDR, Timeout, tickstart) != HAL_OK)
       {
         return HAL_ERROR;
       }

       if (hi2c->XferSize == 0U)
       {
         /* Clear ADDR flag */
         __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

         if(sendReadStop){
         /* Generate Stop */
         SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
         }
       }
       else if (hi2c->XferSize == 1U)
       {
         /* Disable Acknowledge */
         CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_ACK);

         /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
            software sequence must complete before the current byte end of transfer */
         __disable_irq();

         /* Clear ADDR flag */
         __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

         if(sendReadStop){
         /* Generate Stop */
         SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
         }
         /* Re-enable IRQs */
         __enable_irq();
       }
       else if (hi2c->XferSize == 2U)
       {
         /* Enable Pos */
         SET_BIT(hi2c->Instance->CR1, I2C_CR1_POS);

         /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
            software sequence must complete before the current byte end of transfer */
         __disable_irq();

         /* Clear ADDR flag */
         __HAL_I2C_CLEAR_ADDRFLAG(hi2c);

         /* Disable Acknowledge */
         CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_ACK);

         /* Re-enable IRQs */
         __enable_irq();
       }
       else
       {
         /* Enable Acknowledge */
         SET_BIT(hi2c->Instance->CR1, I2C_CR1_ACK);
         /* Clear ADDR flag */
         __HAL_I2C_CLEAR_ADDRFLAG(hi2c);
       }

       while (hi2c->XferSize > 0U)
       {
         if (hi2c->XferSize <= 3U)
         {
           /* One byte */
           if (hi2c->XferSize == 1U)
           {
             /* Wait until RXNE flag is set */
             if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
             {
               return HAL_ERROR;
             }

             /* Read data from DR */
             *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DR;

             /* Increment Buffer pointer */
             hi2c->pBuffPtr++;

             /* Update counter */
             hi2c->XferSize--;
             hi2c->XferCount--;
           }
           /* Two bytes */
           else if (hi2c->XferSize == 2U)
           {
             /* Wait until BTF flag is set */
             if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != HAL_OK)
             {
               return HAL_ERROR;
             }

             /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
                software sequence must complete before the current byte end of transfer */
             __disable_irq();

             if(sendReadStop){
             /* Generate Stop */
             SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
             }

             /* Read data from DR */
             *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DR;

             /* Increment Buffer pointer */
             hi2c->pBuffPtr++;

             /* Update counter */
             hi2c->XferSize--;
             hi2c->XferCount--;

             /* Re-enable IRQs */
             __enable_irq();

             /* Read data from DR */
             *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DR;

             /* Increment Buffer pointer */
             hi2c->pBuffPtr++;

             /* Update counter */
             hi2c->XferSize--;
             hi2c->XferCount--;
           }
           /* 3 Last bytes */
           else
           {
             /* Wait until BTF flag is set */
             if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != HAL_OK)
             {
               return HAL_ERROR;
             }

             /* Disable Acknowledge */
             CLEAR_BIT(hi2c->Instance->CR1, I2C_CR1_ACK);

             /* Disable all active IRQs around ADDR clearing and STOP programming because the EV6_3
                software sequence must complete before the current byte end of transfer */
             __disable_irq();

             /* Read data from DR */
             *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DR;

             /* Increment Buffer pointer */
             hi2c->pBuffPtr++;

             /* Update counter */
             hi2c->XferSize--;
             hi2c->XferCount--;

             /* Wait until BTF flag is set */
             if (I2C_WaitOnFlagUntilTimeout(hi2c, I2C_FLAG_BTF, RESET, Timeout, tickstart) != HAL_OK)
             {
               return HAL_ERROR;
             }

             if(sendReadStop){
             /* Generate Stop */
             SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);
             }

             /* Read data from DR */
             *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DR;

             /* Increment Buffer pointer */
             hi2c->pBuffPtr++;

             /* Update counter */
             hi2c->XferSize--;
             hi2c->XferCount--;

             /* Re-enable IRQs */
             __enable_irq();

             /* Read data from DR */
             *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DR;

             /* Increment Buffer pointer */
             hi2c->pBuffPtr++;

             /* Update counter */
             hi2c->XferSize--;
             hi2c->XferCount--;
           }
         }
         else
         {
           /* Wait until RXNE flag is set */
           if (I2C_WaitOnRXNEFlagUntilTimeout(hi2c, Timeout, tickstart) != HAL_OK)
           {
             return HAL_ERROR;
           }

           /* Read data from DR */
           *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DR;

           /* Increment Buffer pointer */
           hi2c->pBuffPtr++;

           /* Update counter */
           hi2c->XferSize--;
           hi2c->XferCount--;

           if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == SET)
           {
             /* Read data from DR */
             *hi2c->pBuffPtr = (uint8_t)hi2c->Instance->DR;

             /* Increment Buffer pointer */
             hi2c->pBuffPtr++;

             /* Update counter */
             hi2c->XferSize--;
             hi2c->XferCount--;
           }
         }
       }

       hi2c->State = HAL_I2C_STATE_READY;
       hi2c->Mode = HAL_I2C_MODE_NONE;

       /* Process Unlocked */
       __HAL_UNLOCK(hi2c);

       return TRUE;
     }
     else
     {
       return FALSE;
     }
   //TODO: ver i2cData.options si se puede poner la condicion opcional de stop
//
//   I2CM_XFER_T i2cData;
//
//   i2cData.slaveAddr = i2cSlaveAddress;
//   i2cData.options   = 0;
//   i2cData.status    = 0;
//   i2cData.txBuff    = dataToReadBuffer;
//   i2cData.txSz      = dataToReadBufferSize;
//   i2cData.rxBuff    = receiveDataBuffer;
//   i2cData.rxSz      = receiveDataBufferSize;
//
//   if( Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData ) == 0 ) {
//      return FALSE;
//   }


}

static bool_t i2cHardwareWrite( i2cMap_t  i2cNumber,
                                uint8_t  i2cSlaveAddress,
                                uint8_t* transmitDataBuffer,
                                uint16_t transmitDataBufferSize,
                                bool_t   sendWriteStop )
{

    I2C_HandleTypeDef * aux = stmI2Cs[i2cNumber].i2c;


    if( (transmitDataBuffer == NULL) || (transmitDataBufferSize <= 0) ) {
       return FALSE;
    }

    /* Init tickstart for timeout management*/
    uint32_t tickstart = HAL_GetTick(), Timeout = I2C_DEFAULT_TIMEOUT;


    if (aux->State == HAL_I2C_STATE_READY)
    {
      /* Wait until BUSY flag is reset */
      if (I2C_WaitOnFlagUntilTimeout(aux, I2C_FLAG_BUSY, SET, I2C_TIMEOUT_BUSY_FLAG, tickstart) != HAL_OK)
      {
        return HAL_BUSY;
      }

      /* Process Locked */
      __HAL_LOCK(aux);

      /* Check if the I2C is already enabled */
      if ((aux->Instance->CR1 & I2C_CR1_PE) != I2C_CR1_PE)
      {
        /* Enable I2C peripheral */
        __HAL_I2C_ENABLE(aux);
      }

      /* Disable Pos */
      CLEAR_BIT(aux->Instance->CR1, I2C_CR1_POS);

      aux->State     = HAL_I2C_STATE_BUSY_TX;
      aux->Mode      = HAL_I2C_MODE_MEM;
      aux->ErrorCode = HAL_I2C_ERROR_NONE;

      /* Prepare transfer parameters */
      aux->pBuffPtr    = transmitDataBuffer;
      aux->XferCount   = transmitDataBufferSize;
      aux->XferSize    = aux->XferCount;
      aux->XferOptions = I2C_NO_OPTION_FRAME;

      /* Generate Start */
      SET_BIT(aux->Instance->CR1, I2C_CR1_START);

      /* Wait until SB flag is set */
      if (I2C_WaitOnFlagUntilTimeout(aux, I2C_FLAG_SB, RESET, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Send slave address */
      aux->Instance->DR = I2C_7BIT_ADD_WRITE(i2cSlaveAddress);

      /* Wait until ADDR flag is set */
      if (I2C_WaitOnMasterAddressFlagUntilTimeout(aux, I2C_FLAG_ADDR, Timeout, tickstart) != HAL_OK)
      {
        return HAL_ERROR;
      }

      /* Clear ADDR flag */
      __HAL_I2C_CLEAR_ADDRFLAG(aux);

      /* Wait until TXE flag is set */
      if (I2C_WaitOnTXEFlagUntilTimeout(aux, Timeout, tickstart) != HAL_OK)
      {
        if (aux->ErrorCode == HAL_I2C_ERROR_AF)
        {
          /* Generate Stop */
          SET_BIT(aux->Instance->CR1, I2C_CR1_STOP);
        }
        return HAL_ERROR;
      }

      while (aux->XferSize > 0U)
      {
        /* Wait until TXE flag is set */
        if (I2C_WaitOnTXEFlagUntilTimeout(aux, Timeout, tickstart) != HAL_OK)
        {
          if (aux->ErrorCode == HAL_I2C_ERROR_AF)
          {
            /* Generate Stop */
            SET_BIT(aux->Instance->CR1, I2C_CR1_STOP);
          }
          return HAL_ERROR;
        }

        /* Write data to DR */
        aux->Instance->DR = *aux->pBuffPtr;

        /* Increment Buffer pointer */
        aux->pBuffPtr++;

        /* Update counter */
        aux->XferSize--;
        aux->XferCount--;

        if ((__HAL_I2C_GET_FLAG(aux, I2C_FLAG_BTF) == SET) && (aux->XferSize != 0U))
        {
          /* Write data to DR */
          aux->Instance->DR = *aux->pBuffPtr;

          /* Increment Buffer pointer */
          aux->pBuffPtr++;

          /* Update counter */
          aux->XferSize--;
          aux->XferCount--;
        }
      }
      if (I2C_WaitOnBTFFlagUntilTimeout(aux, Timeout, tickstart) != HAL_OK)
          {
            if (aux->ErrorCode == HAL_I2C_ERROR_AF)
            {
              /* Generate Stop */
              SET_BIT(aux->Instance->CR1, I2C_CR1_STOP);
            }
            return HAL_ERROR;
          }


     if(sendWriteStop){


     /* Generate Stop */
     SET_BIT(aux->Instance->CR1, I2C_CR1_STOP);

     }

     aux->State = HAL_I2C_STATE_READY;
     aux->Mode = HAL_I2C_MODE_NONE;

     /* Process Unlocked */
     __HAL_UNLOCK(aux);

     return TRUE;
    }
    else
    {
      return FALSE;
    }




   //TODO: ver i2cData.options si se puede poner la condicion opcional de stop

//   I2CM_XFER_T i2cData;
//
//   if( i2cNumber != I2C0 ) {
//      return FALSE;
//   }
//
//   // Prepare the i2cData register
//   i2cData.slaveAddr = i2cSlaveAddress;
//   i2cData.options   = 0;
//   i2cData.status    = 0;
//   i2cData.txBuff    = transmitDataBuffer;
//   i2cData.txSz      = transmitDataBufferSize;
//   i2cData.rxBuff    = 0;
//   i2cData.rxSz      = 0;
//
//   /* Send the i2c data */
//   if( Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData ) == 0 ) {
//      return FALSE;
//   }

   /* *** TEST I2C Response ***

   Chip_I2CM_XferBlocking( LPC_I2C0, &i2cData );

   if( i2cData.status == I2CM_STATUS_OK){
      while(1){
         gpioWrite( LEDB, ON );
         delay(100);
         gpioWrite( LEDB, OFF );
         delay(100);
      }
   }

   *** END - TEST I2C Response *** */




}

#endif


static HAL_StatusTypeDef I2C_ClearBusyFlagErrata_2_14_7(i2cMap_t i2cNumber) {

    I2C_HandleTypeDef * aux= stmI2Cs[i2cNumber].i2c;
//    static uint8_t resetTried = 0;
//    if (resetTried == 1) {
//        return HAL_ERROR ;
//    }
    uint32_t SDA_PIN = stmI2Cs[i2cNumber].SDAPin.gpio.pin ;
    uint32_t SCL_PIN = stmI2Cs[i2cNumber].SCLPin.gpio.pin ;
    GPIO_TypeDef * I2C_GPIO_PORT = stmI2Cs[i2cNumber].SCLPin.gpio.port;
    GPIO_InitTypeDef GPIO_InitStruct;

    // 1
    __HAL_I2C_DISABLE(aux);

    // 2
    GPIO_InitStruct.Pin = SDA_PIN|SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(I2C_GPIO_PORT, SDA_PIN);
    HAL_GPIO_WRITE_ODR(I2C_GPIO_PORT, SCL_PIN);

    // 3
 //   GPIO_PinState pinState;
    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, SDA_PIN) == GPIO_PIN_RESET) {
       return HAL_ERROR;
    }
    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, SCL_PIN) == GPIO_PIN_RESET) {
        return HAL_ERROR;
    }

    // 4
    GPIO_InitStruct.Pin = SDA_PIN;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_TogglePin(I2C_GPIO_PORT, SDA_PIN);

    // 5
    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, SDA_PIN) == GPIO_PIN_SET) {
        return HAL_ERROR;
    }

    // 6
    GPIO_InitStruct.Pin = SCL_PIN;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_TogglePin(I2C_GPIO_PORT, SCL_PIN);

    // 7
    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, SCL_PIN) == GPIO_PIN_SET) {
        return HAL_ERROR;
    }

    // 8
    GPIO_InitStruct.Pin = SDA_PIN;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(I2C_GPIO_PORT, SDA_PIN);

    // 9
    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, SDA_PIN) == GPIO_PIN_RESET) {
        return HAL_ERROR;
    }

    // 10
    GPIO_InitStruct.Pin = SCL_PIN;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

    HAL_GPIO_WRITE_ODR(I2C_GPIO_PORT, SCL_PIN);

    // 11
    if (HAL_GPIO_ReadPin(I2C_GPIO_PORT, SCL_PIN) == GPIO_PIN_RESET) {
        return HAL_ERROR;
    }

    // 12
    GPIO_InitStruct.Pin = SDA_PIN|SCL_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
    HAL_GPIO_Init(I2C_GPIO_PORT, &GPIO_InitStruct);

   // 13
    aux->Instance->CR1 |= I2C_CR1_SWRST;

   // 14
   aux->Instance->CR1 ^= I2C_CR1_SWRST;

   // 15
   __HAL_I2C_ENABLE(aux);

//   resetTried = 1;
   return HAL_OK;
}

static void saveI2CRegisters(I2C_HandleTypeDef* hi2c,uint32_t* regs){
    regs[0]=READ_REG(hi2c->Instance->CR2);
    regs[1]=READ_REG(hi2c->Instance->OAR1);
    regs[2]=READ_REG(hi2c->Instance->CCR);
    regs[3]=READ_REG(hi2c->Instance->TRISE);
    return;

}

static void restoreI2CRegisters(I2C_HandleTypeDef* hi2c,uint32_t* regs){
    WRITE_REG(hi2c->Instance->CR2,regs[0]);
    WRITE_REG(hi2c->Instance->OAR1,regs[1]);
    WRITE_REG(hi2c->Instance->CCR,regs[2]);
    WRITE_REG(hi2c->Instance->TRISE,regs[3]);
    return;

}

static void HAL_GPIO_WRITE_ODR(GPIO_TypeDef* GPIOx, uint16_t GPIO_Pin)
{
  /* Check the parameters */
  assert_param(IS_GPIO_PIN(GPIO_Pin));

  GPIOx->ODR |= GPIO_Pin;
}

/**
  * @brief  This function handles I2C Communication Timeout.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  Flag specifies the I2C flag to check.
  * @param  Status The new Flag status (SET or RESET).
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, FlagStatus Status, uint32_t Timeout, uint32_t Tickstart)
{
  /* Wait until flag is set */
  while (__HAL_I2C_GET_FLAG(hi2c, Flag) == Status)
  {
    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState     = I2C_STATE_NONE;
        hi2c->State             = HAL_I2C_STATE_READY;
        hi2c->Mode              = HAL_I2C_MODE_NONE;
        hi2c->ErrorCode         |= HAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}
/**
  * @brief  This function handles I2C Communication Timeout for Master addressing phase.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *         the configuration information for I2C module
  * @param  Flag specifies the I2C flag to check.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnMasterAddressFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Flag, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_I2C_GET_FLAG(hi2c, Flag) == RESET)
  {
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
    {
      /* Generate Stop */
      SET_BIT(hi2c->Instance->CR1, I2C_CR1_STOP);

      /* Clear AF Flag */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

      hi2c->PreviousState       = I2C_STATE_NONE;
      hi2c->State               = HAL_I2C_STATE_READY;
      hi2c->Mode                = HAL_I2C_MODE_NONE;
      hi2c->ErrorCode           |= HAL_I2C_ERROR_AF;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = HAL_I2C_STATE_READY;
        hi2c->Mode                = HAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= HAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of TXE flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */

static HAL_StatusTypeDef I2C_WaitOnTXEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_TXE) == RESET)
  {
    /* Check if a NACK is detected */
    if (I2C_IsAcknowledgeFailed(hi2c) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = HAL_I2C_STATE_READY;
        hi2c->Mode                = HAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= HAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of BTF flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnBTFFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{
  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_BTF) == RESET)
  {
    /* Check if a NACK is detected */
    if (I2C_IsAcknowledgeFailed(hi2c) != HAL_OK)
    {
      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if (Timeout != HAL_MAX_DELAY)
    {
      if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
      {
        hi2c->PreviousState       = I2C_STATE_NONE;
        hi2c->State               = HAL_I2C_STATE_READY;
        hi2c->Mode                = HAL_I2C_MODE_NONE;
        hi2c->ErrorCode           |= HAL_I2C_ERROR_TIMEOUT;

        /* Process Unlocked */
        __HAL_UNLOCK(hi2c);

        return HAL_ERROR;
      }
    }
  }
  return HAL_OK;
}

/**
  * @brief  This function handles Acknowledge failed detection during an I2C Communication.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_IsAcknowledgeFailed(I2C_HandleTypeDef *hi2c)
{
  if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_AF) == SET)
  {
    /* Clear NACKF Flag */
    __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_AF);

    hi2c->PreviousState       = I2C_STATE_NONE;
    hi2c->State               = HAL_I2C_STATE_READY;
    hi2c->Mode                = HAL_I2C_MODE_NONE;
    hi2c->ErrorCode           |= HAL_I2C_ERROR_AF;

    /* Process Unlocked */
    __HAL_UNLOCK(hi2c);

    return HAL_ERROR;
  }
  return HAL_OK;
}

/**
  * @brief  This function handles I2C Communication Timeout for specific usage of RXNE flag.
  * @param  hi2c Pointer to a I2C_HandleTypeDef structure that contains
  *                the configuration information for the specified I2C.
  * @param  Timeout Timeout duration
  * @param  Tickstart Tick start value
  * @retval HAL status
  */
static HAL_StatusTypeDef I2C_WaitOnRXNEFlagUntilTimeout(I2C_HandleTypeDef *hi2c, uint32_t Timeout, uint32_t Tickstart)
{

  while (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_RXNE) == RESET)
  {
    /* Check if a STOPF is detected */
    if (__HAL_I2C_GET_FLAG(hi2c, I2C_FLAG_STOPF) == SET)
    {
      /* Clear STOP Flag */
      __HAL_I2C_CLEAR_FLAG(hi2c, I2C_FLAG_STOPF);

      hi2c->PreviousState       = I2C_STATE_NONE;
      hi2c->State               = HAL_I2C_STATE_READY;
      hi2c->Mode                = HAL_I2C_MODE_NONE;
      hi2c->ErrorCode           |= HAL_I2C_ERROR_NONE;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }

    /* Check for the Timeout */
    if (((HAL_GetTick() - Tickstart) > Timeout) || (Timeout == 0U))
    {
      hi2c->PreviousState       = I2C_STATE_NONE;
      hi2c->State               = HAL_I2C_STATE_READY;
      hi2c->Mode                = HAL_I2C_MODE_NONE;
      hi2c->ErrorCode           |= HAL_I2C_ERROR_TIMEOUT;

      /* Process Unlocked */
      __HAL_UNLOCK(hi2c);

      return HAL_ERROR;
    }
  }
  return HAL_OK;
}




/*==================[external functions definition]==========================*/

bool_t i2cInit( i2cMap_t i2cNumber, uint32_t clockRateHz )
{

   bool_t retVal = FALSE;

   if( i2cNumber != I2C_1 && i2cNumber != I2C_2 ) {
      return FALSE;
   }

#if( I2C_SOFTWARE == 1 )
   retVal = i2cSoftwareInit( i2cNumber, clockRateHz );
#else
   retVal = i2cHardwareInit( i2cNumber, clockRateHz );
#endif

   return retVal;
}

/**
  * @brief  This function Reads data through I2C.
  * @param  i2cNumber Number of I2C.
  *         i2cSlaveAddress Slave 7 bit address, must be shifted left (LSBit should be free)
  *         dataToReadBuffer Pointer to data buffer to be transfered before the Reading operation, if the Read is targeting a Slave's register,
  *           register's address must be the in first positions of this buffer (if the register's address is bigger than 8 bits
  *           MSByte must be first).
  *           If a direct reading, not a particular register, wants to be performed this buffer must be empty.
  *         dataToReadBufferSize Number of bytes to be transmitted. Can be zero if direct reading wants to be performed
  *         sendWriteStop Option for send stop condition
  *         receiveDataBuffer Pointer to data buffer to store received data
  *         receiveDataBufferSize Number of bytes to be Received
  *         sendReadStop Option for send stop condition
  *
  * @retval bool_t
  */
bool_t i2cRead( i2cMap_t  i2cNumber,
                uint8_t  i2cSlaveAddress,
                uint8_t* dataToReadBuffer,
                uint16_t dataToReadBufferSize,
                bool_t   sendWriteStop,
                uint8_t* receiveDataBuffer,
                uint16_t receiveDataBufferSize,
                bool_t   sendReadStop )
{

   bool_t retVal = FALSE;

   if( i2cNumber != I2C_1 && i2cNumber != I2C_2 ) {
      return FALSE;
   }

#if( I2C_SOFTWARE == 1 )
   retVal = i2cSoftwareRead( i2cNumber,
                             i2cSlaveAddress,
                             dataToReadBuffer,
                             dataToReadBufferSize,
                             sendWriteStop,
                             receiveDataBuffer,
                             receiveDataBufferSize,
                             sendReadStop );
#else
   retVal = i2cHardwareRead( i2cNumber,
                             i2cSlaveAddress,
                             dataToReadBuffer,
                             dataToReadBufferSize,
                             sendWriteStop,
                             receiveDataBuffer,
                             receiveDataBufferSize,
                             sendReadStop );
#endif

   return retVal;
}

/**
  * @brief  This function Transmit data through I2C.
  * @param  i2cNumber Number of I2C.
  *         i2cSlaveAddress Slave's 7 bit address, must be shifted left (LSBit should be free)
  *         transmitDataBuffer Pointer to data buffer to be transfered, if the write is targeting a Slave's register,
  *           register's address must be the first positions in this buffer and the data comes next.(if the register's address is bigger than 8 bits
  *           MSByte must be first)
  *         transmitDataBufferSize Number of bytes to be Transmitted
  *         sendWriteStop Option for send stop condition
  *
  * @retval bool_t
  */
bool_t i2cWrite( i2cMap_t  i2cNumber,
                 uint8_t  i2cSlaveAddress,
                 uint8_t* transmitDataBuffer,
                 uint16_t transmitDataBufferSize,
                 bool_t   sendWriteStop )
{

   bool_t retVal = FALSE;
   if( i2cNumber != I2C_1 && i2cNumber != I2C_2 ) {
      return FALSE;
   }

#if( I2C_SOFTWARE == 1 )
   retVal = i2cSoftwareWrite( i2cNumber,
                              i2cSlaveAddress,
                              transmitDataBuffer,
                              transmitDataBufferSize,
                              sendWriteStop );
#else
   retVal = i2cHardwareWrite( i2cNumber,
                              i2cSlaveAddress,
                              transmitDataBuffer,
                              transmitDataBufferSize,
                              sendWriteStop );
#endif

   return retVal;
}


#if( I2C_SOFTWARE == 1 )
// Software Master I2C

void i2cSoftwareDelay( tick_t duration )
{
   volatile tick_t i;

   duration = 13 * duration;
   for( i=duration; i>0; i-- );
}

// Ver!!!
// communication reset: DATA-line=1 and at least 9 SCK cycles followed by transstart
//       _____________________________________________________         ________
// DATA:                                                      |_______|
//          _    _    _    _    _    _    _    _    _        ___     ___
// SCK : __| |__| |__| |__| |__| |__| |__| |__| |__| |______|   |___|   |______


// Generates a transmission start bit sequence
//      ________
// SCL:         |_
//      _____
// SDA:      |____
//
void i2cSoftwareMasterWriteStart( void )
{

   // Clock (SCL) pin HIGH
   i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, HIGH );
   // Data (SDA) pin HIGH
   i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, HIGH );
   // 1 clock time delay
   i2cSoftwareDelay(10);

   // Data (SDA) pin LOW
   i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, LOW );
   // 1/2 clock time delay
   i2cSoftwareDelay(5);

   // Clock (SCL) pin LOW
   i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, LOW );
   // 3/10 clock time delay
   i2cSoftwareDelay(3);
}

// Generates a transmission stop bit sequence
//        ________
// SCL: _|
//           _____
// SDA: ____|

void i2cSoftwareMasterWriteStop( void )
{
   // Data (SDA) pin LOW
   i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, LOW );
   // Clock (SCL) pin LOW
   i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, LOW );
   // 1/5 clock time delay
   i2cSoftwareDelay(2);

   // Clock (SCL) pin HIGH
   i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, HIGH );
   // 1/2 clock time delay
   i2cSoftwareDelay(5);

   // Data (SDA) pin HIGH
   i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, HIGH );
   // 1 clock time delay
   i2cSoftwareDelay(10);
}

// Write data byte
//          ___     ___     ___     ___     ___     ___     ___     ___     ___
// SCL: ___| 1 |___| 2 |___| 3 |___| 4 |___| 5 |___| 6 |___| 7 |___| 8 |___| 9 |___
//         _______ _______ _______ _______ _______ _______ _______ _______
// SDA: __|   D7  |   D6  |   D5  |   D4  |   D3  |   D2  |   D1  |   D0  |__ACK?__
//
bool_t i2cSoftwareMasterWriteByte( uint8_t dataByte )
{

   uint8_t i;
   static bool_t ackOrNack;

   for( i=8; i>0; i-- ) {

      // Clock (SCL) pin LOW
      i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, LOW );
      // Data (SDA) pin with MSB bit value of dataByte
      i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, (dataByte & 0x80) );
      // 1/5 clock time delay
      i2cSoftwareDelay(2);

      // Clock (SCL) pin HIGH
      i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, HIGH );
      // 1/2 clock time delay
      i2cSoftwareDelay(5);

      // Clock (SCL) pin LOW
      i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, LOW );
      // 3/10 clock time delay
      i2cSoftwareDelay(3);

      // left shift dataByte
      dataByte <<= 1;
   }

   // Maintain SCL LOW for 1/10 clock time delay
   i2cSoftwareDelay(1);
   // Configure SDA pin as input
   i2cSoftwarePinInit( I2C_SOFTWARE_SDA_DIR, GPIO_INPUT_PULLUP );
   // Maintain SCL LOW for 1/10 clock time delay more
   i2cSoftwareDelay(1);

   // Clock (SCL) pin HIGH
   i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, HIGH );
   // 1/10 clock time delay
   i2cSoftwareDelay(1);
   // Read Data (SDA) pin for possible ACK bit
   ackOrNack = i2cSoftwarePinRead( I2C_SOFTWARE_SDA_IN );
   // 2/5 clock time delay
   i2cSoftwareDelay(4);

   // Clock (SCL) pin LOW
   i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, LOW );
   // 1/5 clock time delay
   i2cSoftwareDelay(2);
   // Configure SDA pin as output. This prevent that SCL master, the
   // microcontroller, and SCL Slave, device, both OUTPUT at the same time.
   // This Output-Output condition can damage devices.
   i2cSoftwarePinInit( I2C_SOFTWARE_SDA_DIR, GPIO_OUTPUT );
   // 1/10 clock time delay
   i2cSoftwareDelay(1);

   return ackOrNack;
}

// Read data byte
//          ___     ___     ___     ___     ___     ___     ___     ___     ___
// SCL: ___| 1 |___| 2 |___| 3 |___| 4 |___| 5 |___| 6 |___| 7 |___| 8 |___| 9 |___
//         _______ _______ _______ _______ _______ _______ _______ _______
// SDA: __|   D7  |   D6  |   D5  |   D4  |   D3  |   D2  |   D1  |   D0  |__ACK?__
//
uint8_t i2cSoftwareMasterReadByte( bool_t ack )
{

   uint8_t i, receivedData = 0;
   bool_t receivedBit = 0;

   // Configure SDA pin as input
   i2cSoftwarePinInit( I2C_SOFTWARE_SDA_DIR, GPIO_INPUT_PULLUP );

   for( i=8; i>0; i-- ) {

      // Clock (SCL) pin LOW
      i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, LOW );
      // 1/5 clock time delay
      i2cSoftwareDelay(2);

      //do{
      // Clock (SCL) pin HIGH
      i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, HIGH );
      //}while( SCL_IN == 0 );    // wait for any SCL clock stretching
      // 1/10 clock time delay
      i2cSoftwareDelay(1);
      // Read Data (SDA) pin
      receivedBit = i2cSoftwarePinRead( I2C_SOFTWARE_SDA_IN );
      // 2/5 clock time delay
      i2cSoftwareDelay(4);

      // Shift left receivedData
      receivedData <<= 1;

      if( receivedBit ) {
         receivedData |= 0x01;
      }

      // Clock (SCL) pin LOW
      i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, LOW );
      // 3/10 clock time delay
      i2cSoftwareDelay(3);
   }

   // Maintain SCL LOW for 1/10 clock time delay
   i2cSoftwareDelay(1);
   // Configure SDA pin as output
   i2cSoftwarePinInit( I2C_SOFTWARE_SDA_DIR, GPIO_OUTPUT );

   // send (N)ACK bit (ACK=LOW, NACK=HIGH)
   if( ack ) {
      // Data (SDA) pin LOW
      i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, LOW );
   } else {
      // Data (SDA) pin HIGH
      i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, HIGH );
   }
   // Maintain SCL LOW for 1/10 clock time delay more
   i2cSoftwareDelay(1);

   // Clock (SCL) pin HIGH
   i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, HIGH );
   // 1/2 clock time delay
   i2cSoftwareDelay(5);

   // Clock (SCL) pin LOW
   i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, LOW );
   // 1/10 clock time delay
   i2cSoftwareDelay(1);
   // Data (SDA) pin LOW
   i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, LOW );
   // 1/5 clock time delay
   i2cSoftwareDelay(2);

   return receivedData;
}
/* That's almost it for simple I2C communications, but there is one more
 * complication. When the master is reading from the slave, its the slave that
 * places the data on the SDA line, but its the master that controls the clock.
 * What if the slave is not ready to send the data! With devices such as
 * EEPROMs this is not a problem, but when the slave device is actually a
 * microprocessor with other things to do, it can be a problem. The
 * microprocessor on the slave device will need to go to an interrupt routine,
 * save its working registers, find out what address the master wants to read
 * from, get the data and place it in its transmission register. This can take
 * many uS to happen, meanwhile the master is blissfully sending out clock
 * pulses on the SCL line that the slave cannot respond to. The I2C protocol
 * provides a solution to this: the slave is allowed to hold the SCL line low!
 * This is called clock stretching. When the slave gets the read command from
 * the master it holds the clock line low. The microprocessor then gets the
 * requested data, places it in the transmission register and releases the
 * clock line allowing the pull-up resistor to finally pull it high. From the
 * masters point of view, it will issue the first clock pulse of the read by
 * making SCL high and then check to see if it really has gone high. If its
 * still low then its the slave that holding it low and the master should wait
 * until it goes high before continuing. Luckily the hardware I2C ports on
 * most microprocessors will handle this automatically.
 *
 * Sometimes however, the master I2C is just a collection of subroutines and
 * there are a few implementations out there that completely ignore clock
 * stretching. They work with things like EEPROM's but not with microprocessor
 * slaves that use clock stretching. The result is that erroneous data is read
 * from the slave. Beware!
 *
 * http://www.robot-electronics.co.uk/i2c-tutorial
 */


// Write 7 bit address + R or W bit
//              ___
// SDA: _______|
//          _______
// SCL: ___|
//
bool_t i2cSoftwareMasterWriteAddress( uint8_t i2cSlaveAddress,
                                      I2C_Software_rw_t readOrWrite )
{

   bool_t ackOrNack = FALSE;

   if( readOrWrite == I2C_SOFTWARE_WRITE ) {
      // 7 bit address + Write = 0
      i2cSlaveAddress <<= 1;
      ackOrNack = i2cSoftwareMasterWriteByte( i2cSlaveAddress );

   } else if( readOrWrite == I2C_SOFTWARE_READ ) {
      // 7 bit address + Read = 1
      i2cSlaveAddress <<= 1;
      i2cSlaveAddress |= 0x01;
      ackOrNack = i2cSoftwareMasterWriteByte( i2cSlaveAddress );
   }

   return ackOrNack;
}
#endif


#if( SOFTWARE_I2C_DEBUG == 1 )

/*
 * Conexión:
 *
 * Se debe conectar un led al pin elegido como SCL con una R de 470ohm.
 * Otro led al pin elegido como SDA con una R de 470ohm.
 * Una R de pull-up de 4K7 entre VDD=+3.3V y SDA.
 * Un pulsador entre GND y SDA.
 *
 * Funcionamiento:
 *
 * Cada 10 segundos toglea el modo del pin SDA entre Input y Output.
 * Mientras SDA es input escribe el valor del pulsador en el pin SCL.
 * Cada 500ms se toglea una variable llamada clockStatus.
 * Mientras el pin SDA es Output se escribe en el led conectado a SDA
 * el valor de la variable clockStatus.
 */

//#include "sapi_delay.h"

// Test vars
bool_t clockStatus = FALSE;
delay_t clockDelay;
bool_t direction = FALSE;
delay_t delayDir;

void i2cSoftwareMasterPinTestInit( void )
{
   delayInit( &clockDelay, 500 );
   delayInit( &delayDir, 10000 );
}

void i2cSoftwareMasterPinTest( void )
{

   if( delayRead( &delayDir ) ) {
      if( direction ) {
         direction = FALSE;
      } else {
         direction = TRUE;
      }
//          I2C_SOFTWARE_SDA_DIR = direction;
   }

   if( delayRead( &clockDelay ) ) {
      if( clockStatus ) {
         clockStatus = FALSE;
      } else {
         clockStatus = TRUE;
      }
      //I2C_SOFTWARE_SCL_OUT = clockStatus;
   }

   if( direction ) { // Input
      //I2C_SOFTWARE_SCL_OUT = I2C_SOFTWARE_SDA_IN;
      i2cSoftwarePinWrite( I2C_SOFTWARE_SCL_OUT, I2C_SOFTWARE_SDA_IN );
   } else {         // Output
      //I2C_SOFTWARE_SDA_OUT = clockStatus;
      i2cSoftwarePinWrite( I2C_SOFTWARE_SDA_OUT, clockStatus );
   }
}
#endif

/*==================[ISR external functions definition]======================*/

/*==================[end of file]============================================*/
