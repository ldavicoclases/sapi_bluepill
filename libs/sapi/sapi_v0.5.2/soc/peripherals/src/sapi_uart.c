/* Copyright 2014, Pablo Ridolfi (UTN-FRBA).
 * Copyright 2014, Juan Cecconi.
 * Copyright 2015-2017, Eric Pernia.
 * Copyright 2020, Nahuel Espinosa
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

/* Date: 2020-04-22 */

/*==================[inclusions]=============================================*/

#include "sapi_uart.h"

#include "string.h"
#include "sapi_circularBuffer.h"
#include "sapi_gpio.h"

/*==================[macros]=================================================*/

/*==================[typedef]================================================*/

typedef struct{
   UART_HandleTypeDef*      uart;
   pinInitGpioStm32f1xx_t   txPin;
   pinInitGpioStm32f1xx_t   rxPin;
   IRQn_Type                irq;
} uartStmInit_t;

/*==================[internal data declaration]==============================*/

#ifdef SAPI_USE_INTERRUPTS
static callBackFuncPtr_t rxIsrCallbackUART1 = 0;
static void* rxIsrCallbackUART1Params = NULL;
static callBackFuncPtr_t rxIsrCallbackUART2 = 0;
static void* rxIsrCallbackUART2Params = NULL;
static callBackFuncPtr_t rxIsrCallbackUART3 = 0;
static void* rxIsrCallbackUART3Params = NULL;

static callBackFuncPtr_t txIsrCallbackUART1 = 0;
static void* txIsrCallbackUART1Params = NULL;
static callBackFuncPtr_t txIsrCallbackUART2 = 0;
static void* txIsrCallbackUART2Params = NULL;
static callBackFuncPtr_t txIsrCallbackUART3 = 0;
static void* txIsrCallbackUART3Params = NULL;
#endif /* SAPI_USE_INTERRUPTS */

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

static const uartStmInit_t stmUarts[] = {
// { uartAddr, { {txPort, txpin}, txmode }, { {rxPort, rxpin}, rxmode }, uartIrqAddr  },
   {
      &huart1,
      {{GPIOA, GPIO_PIN_9 }, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
      {{GPIOA, GPIO_PIN_10}, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      USART1_IRQn
   },
   {
      &huart2,
      {{GPIOA, GPIO_PIN_2 }, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
      {{GPIOA, GPIO_PIN_3 }, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      USART2_IRQn
   },
   {
      &huart3,
      {{GPIOB, GPIO_PIN_10}, GPIO_MODE_AF_PP, GPIO_NOPULL, GPIO_SPEED_FREQ_HIGH},
      {{GPIOB, GPIO_PIN_11}, GPIO_MODE_INPUT, GPIO_NOPULL, GPIO_SPEED_FREQ_LOW},
      USART3_IRQn
   },
};


/*
   callBackFuncPtr_t txIsrCallback;
   callBackFuncPtr_t rxIsrCallback;
*/

/*==================[internal functions declaration]=========================*/

#ifdef SAPI_USE_INTERRUPTS
static void uartProcessIRQ( uartMap_t uart );

/*==================[internal functions definition]==========================*/

static void uartProcessIRQ( uartMap_t uart )
{
   // Rx Interrupt
   if(__HAL_UART_GET_FLAG(stmUarts[uart].uart, UART_FLAG_RXNE) == SET) { // uartRxReady
      // Execute callback
      if( ( uart == UART_1 ) && (rxIsrCallbackUART1 != 0) )
         (*rxIsrCallbackUART1)(0);

      if( ( uart == UART_2 ) && (rxIsrCallbackUART2 != 0) )
         (*rxIsrCallbackUART2)(0);

      if( ( uart == UART_3 ) && (rxIsrCallbackUART3 != 0) )
         (*rxIsrCallbackUART3)(0);
   }

   // Tx Interrupt
   if(__HAL_UART_GET_FLAG(stmUarts[uart].uart, UART_FLAG_TC) == SET) {

      // Execute callback
      if( ( uart == UART_1 ) && (txIsrCallbackUART1 != 0) )
         (*txIsrCallbackUART1)(0);

      if( ( uart == UART_2 ) && (txIsrCallbackUART2 != 0) )
         (*txIsrCallbackUART2)(0);

      if( ( uart == UART_3 ) && (txIsrCallbackUART3 != 0) )
         (*txIsrCallbackUART3)(0);
   }
}
#endif /* SAPI_USE_INTERRUPTS */

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

// Check for Receive a given pattern

waitForReceiveStringOrTimeoutState_t waitForReceiveStringOrTimeout(
   uartMap_t uart, waitForReceiveStringOrTimeout_t* instance )
{

   uint8_t receiveByte;
   //char receiveBuffer[100];

   switch( instance->state ) {

   case UART_RECEIVE_STRING_CONFIG:

      delayInit( &(instance->delay), instance->timeout );

      instance->stringIndex = 0;

      instance->state = UART_RECEIVE_STRING_RECEIVING;

      break;

   case UART_RECEIVE_STRING_RECEIVING:

      if( uartReadByte( uart, &receiveByte ) ) {

         //uartWriteByte( UART_DEBUG, receiveByte ); // TODO: DEBUG
         /*            if( (instance->stringIndex) <= 100 ){
                        receiveBuffer[instance->stringIndex] = receiveByte;
                     }
         */
         if( (instance->string)[(instance->stringIndex)] == receiveByte ) {

            (instance->stringIndex)++;

            if( (instance->stringIndex) == (instance->stringSize - 1) ) {
               instance->state = UART_RECEIVE_STRING_RECEIVED_OK;

//                  receiveBuffer[instance->stringIndex] = '\0';

               //uartWriteString( UART_DEBUG, receiveBuffer ); // TODO: DEBUG
               //uartWriteString( UART_DEBUG, "\r\n" );        // TODO: DEBUG
            }

         }

      }

      if( delayRead( &(instance->delay) ) ) {
         instance->state = UART_RECEIVE_STRING_TIMEOUT;
         //uartWriteString( UART_DEBUG, "\r\n" ); // TODO: DEBUG
      }

      break;

   case UART_RECEIVE_STRING_RECEIVED_OK:
      instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

   case UART_RECEIVE_STRING_TIMEOUT:
      instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

   default:
      instance->state = UART_RECEIVE_STRING_CONFIG;
      break;
   }

   return instance->state;
}

// Recibe bytes hasta que llegue el string patron que se le manda en el
// parametro string, stringSize es la cantidad de caracteres del string.
// Devuelve TRUE cuando recibio la cadena patron, si paso el tiempo timeout
// en milisegundos antes de recibir el patron devuelve FALSE.
// No almacena los datos recibidos!! Simplemente espera a recibir cierto patron.
bool_t waitForReceiveStringOrTimeoutBlocking(
   uartMap_t uart, char* string, uint16_t stringSize, tick_t timeout )
{

   bool_t retVal = TRUE; // True if OK

   waitForReceiveStringOrTimeout_t waitText;
   waitForReceiveStringOrTimeoutState_t waitTextState;

   waitTextState = UART_RECEIVE_STRING_CONFIG;

   waitText.state = UART_RECEIVE_STRING_CONFIG;
   waitText.string =  string;
   waitText.stringSize = stringSize;
   waitText.timeout = timeout;

   while( waitTextState != UART_RECEIVE_STRING_RECEIVED_OK &&
          waitTextState != UART_RECEIVE_STRING_TIMEOUT ) {
      waitTextState = waitForReceiveStringOrTimeout( uart, &waitText );
   }

   if( waitTextState == UART_RECEIVE_STRING_TIMEOUT ) {
      retVal = FALSE;
   }

   return retVal;
}


// Store bytes until receive a given pattern
waitForReceiveStringOrTimeoutState_t receiveBytesUntilReceiveStringOrTimeout(
   uartMap_t uart, waitForReceiveStringOrTimeout_t* instance,
   char* receiveBuffer, uint32_t* receiveBufferSize )
{

   uint8_t receiveByte;
   static uint32_t i = 0;
   //uint32_t j = 0;
   //uint32_t savedReceiveBufferSize = *receiveBufferSize;

   switch( instance->state ) {

   case UART_RECEIVE_STRING_CONFIG:

      delayInit( &(instance->delay), instance->timeout );

      instance->stringIndex = 0;
      i = 0;

      instance->state = UART_RECEIVE_STRING_RECEIVING;

      break;

   case UART_RECEIVE_STRING_RECEIVING:

      if( uartReadByte( uart, &receiveByte ) ) {

         //uartWriteByte( UART_DEBUG, receiveByte ); // TODO: DEBUG
         if( i < *receiveBufferSize ) {
            receiveBuffer[i] = receiveByte;
            i++;
         } else {
            instance->state = UART_RECEIVE_STRING_FULL_BUFFER;
            *receiveBufferSize = i;
            i = 0;
            return instance->state;
         }

         if( (instance->string)[(instance->stringIndex)] == receiveByte ) {

            (instance->stringIndex)++;

            if( (instance->stringIndex) == (instance->stringSize - 1) ) {
               instance->state = UART_RECEIVE_STRING_RECEIVED_OK;
               *receiveBufferSize = i;
               /*
               // TODO: For debug purposes
               for( j=0; j<i; j++ ){
                  uartWriteByte( UART_DEBUG, receiveBuffer[j] );
               }
               uartWriteString( UART_DEBUG, "\r\n" );
               */
               i = 0;
            }

         }

      }

      if( delayRead( &(instance->delay) ) ) {
         instance->state = UART_RECEIVE_STRING_TIMEOUT;
         //uartWriteString( UART_DEBUG, "\r\n" ); // TODO: DEBUG
         *receiveBufferSize = i;
         i = 0;
      }

      break;

   case UART_RECEIVE_STRING_RECEIVED_OK:
      instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

   case UART_RECEIVE_STRING_TIMEOUT:
      instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

   case UART_RECEIVE_STRING_FULL_BUFFER:
      instance->state = UART_RECEIVE_STRING_CONFIG;
      break;

   default:
      instance->state = UART_RECEIVE_STRING_CONFIG;
      break;
   }

   return instance->state;
}

// Guarda todos los bytes que va recibiendo hasta que llegue el string
// patron que se le manda en el parametro string, stringSize es la cantidad
// de caracteres del string.
// receiveBuffer es donde va almacenando los caracteres recibidos y
// receiveBufferSize es el tamaño de buffer receiveBuffer.
// Devuelve TRUE cuando recibio la cadena patron, si paso el tiempo timeout
// en milisegundos antes de recibir el patron devuelve FALSE.
bool_t receiveBytesUntilReceiveStringOrTimeoutBlocking(
   uartMap_t uart, char* string, uint16_t stringSize,
   char* receiveBuffer, uint32_t* receiveBufferSize,
   tick_t timeout )
{

   bool_t retVal = TRUE; // True if OK

   waitForReceiveStringOrTimeout_t waitText;
   waitForReceiveStringOrTimeoutState_t waitTextState;

   waitTextState = UART_RECEIVE_STRING_CONFIG;

   waitText.state = UART_RECEIVE_STRING_CONFIG;
   waitText.string =  string;
   waitText.stringSize = stringSize;
   waitText.timeout = timeout;

   while( waitTextState != UART_RECEIVE_STRING_RECEIVED_OK &&
          waitTextState != UART_RECEIVE_STRING_TIMEOUT ) {
      waitTextState = receiveBytesUntilReceiveStringOrTimeout(
                         uart, &waitText,
                         receiveBuffer, receiveBufferSize );
   }

   if( waitTextState == UART_RECEIVE_STRING_TIMEOUT ) {
      retVal = FALSE;
   }

   return retVal;
}

//-------------------------------------------------------------

#ifdef SAPI_USE_INTERRUPTS

// UART Global Interrupt Enable/Disable
void uartInterrupt( uartMap_t uart, bool_t enable )
{
   if( enable ) {
      // Interrupt Priority for UART channel
      NVIC_SetPriority( stmUarts[uart].irq, 5 ); // FreeRTOS Requiere prioridad >= 5 (numero mas alto, mas baja prioridad)
      // Enable Interrupt for UART channel
      NVIC_EnableIRQ( stmUarts[uart].irq );
   } else {
      // Disable Interrupt for UART channel
      NVIC_DisableIRQ( stmUarts[uart].irq );
   }
}

// UART Interrupt event Enable and set a callback
void uartCallbackSet( uartMap_t uart, uartEvents_t event, 
                      callBackFuncPtr_t callbackFunc, void* callbackParam )
{
   switch(event){

      case UART_RECEIVE:
          __HAL_UART_ENABLE_IT(stmUarts[uart].uart, UART_IT_RXNE);

         if( callbackFunc != 0 ) {
            // Set callback
            if( uart == UART_1 ){
               rxIsrCallbackUART1 = callbackFunc;
               rxIsrCallbackUART1Params = callbackParam;
            }
            if( uart == UART_2 ){
               rxIsrCallbackUART2 = callbackFunc;
               rxIsrCallbackUART2Params = callbackParam;
            }            
            if( uart == UART_3 ){
               rxIsrCallbackUART3 = callbackFunc;
               rxIsrCallbackUART3Params = callbackParam;
            }
         } else{
            return;
         }
      break;

      case UART_TRANSMITER_FREE:
         if( callbackFunc != 0 ) {
            __HAL_UART_ENABLE_IT(stmUarts[uart].uart, UART_IT_TC);
            
            // Set callback
            if( uart == UART_1 ){
               txIsrCallbackUART1 = callbackFunc;
               txIsrCallbackUART1Params = callbackParam;
            }
            if( uart == UART_1 ){
               txIsrCallbackUART2 = callbackFunc;
               txIsrCallbackUART2Params = callbackParam;
            }            
            if( uart == UART_3 ){
               txIsrCallbackUART3 = callbackFunc;
               txIsrCallbackUART3Params = callbackParam;
            }
         } else{
            return;
         }
      break;

      default:
         return;
   }

   // Enable UART Interrupt
   HAL_NVIC_SetPriority(stmUarts[uart].irq, 0, 0);
   HAL_NVIC_EnableIRQ(stmUarts[uart].irq);
}
                 
// UART Interrupt event Disable
void uartCallbackClr( uartMap_t uart, uartEvents_t event )
{
   switch(event){

      case UART_RECEIVE:
         __HAL_UART_DISABLE_IT(stmUarts[uart].uart, UART_IT_RXNE);
      break;
      
      case UART_TRANSMITER_FREE:
         __HAL_UART_DISABLE_IT(stmUarts[uart].uart, UART_IT_TC);
      break;
      
      default:
         return;
   }

   // Disable UART Interrupt
   NVIC_DisableIRQ( stmUarts[uart].irq );
}
 
// UART Set Pending Interrupt. Useful to force first character in tx transmission
void uartSetPendingInterrupt(uartMap_t uart) {
   NVIC_SetPendingIRQ(stmUarts[uart].irq);
}

// UART Clear Pending Interrupt.
void uartClearPendingInterrupt(uartMap_t uart) {
   NVIC_ClearPendingIRQ(stmUarts[uart].irq);
} 


#endif /* SAPI_USE_INTERRUPTS */

//-------------------------------------------------------------

// UART Initialization
void uartInit( uartMap_t uart, uint32_t baudRate )
{
   switch( uart ) {
      case UART_1:
          __HAL_RCC_USART1_CLK_ENABLE();
          /**USART1 GPIO Configuration
          PA9     ------> USART1_TX
          PA10     ------> USART1_RX
          */
          __HAL_RCC_GPIOA_CLK_ENABLE();

          stmUarts[uart].uart->Instance = USART1;
          break;
      case UART_2:
          __HAL_RCC_USART2_CLK_ENABLE();
          /**USART2 GPIO Configuration
          PA2     ------> USART2_TX
          PA3     ------> USART2_RX
          */
          __HAL_RCC_GPIOA_CLK_ENABLE();

          stmUarts[uart].uart->Instance = USART2;
          break;
      case UART_3:
          __HAL_RCC_USART3_CLK_ENABLE();
          /**USART3 GPIO Configuration
          PB10     ------> USART3_TX
          PB11     ------> USART3_RX
          */
          __HAL_RCC_GPIOB_CLK_ENABLE();

          stmUarts[uart].uart->Instance = USART3;
          break;
      case UART_USB:
          break;
      default:
          break;
   }

   GPIO_InitTypeDef GPIO_InitStruct = {0};

   GPIO_InitStruct.Pin   = stmUarts[uart].txPin.gpio.pin;
   GPIO_InitStruct.Mode  = stmUarts[uart].txPin.mode;
   GPIO_InitStruct.Pull  = stmUarts[uart].txPin.pull;
   GPIO_InitStruct.Speed = stmUarts[uart].txPin.speed;
   HAL_GPIO_Init(stmUarts[uart].txPin.gpio.port, &GPIO_InitStruct);

   GPIO_InitStruct.Pin   = stmUarts[uart].rxPin.gpio.pin;
   GPIO_InitStruct.Mode  = stmUarts[uart].rxPin.mode;
   GPIO_InitStruct.Pull  = stmUarts[uart].rxPin.pull;
   GPIO_InitStruct.Speed = stmUarts[uart].rxPin.speed;
   HAL_GPIO_Init(stmUarts[uart].rxPin.gpio.port, &GPIO_InitStruct);

   stmUarts[uart].uart->Init.BaudRate     = baudRate;
   stmUarts[uart].uart->Init.WordLength   = UART_WORDLENGTH_8B;
   stmUarts[uart].uart->Init.StopBits     = UART_STOPBITS_1;
   stmUarts[uart].uart->Init.Parity       = UART_PARITY_NONE;
   stmUarts[uart].uart->Init.Mode         = UART_MODE_TX_RX;
   stmUarts[uart].uart->Init.HwFlowCtl    = UART_HWCONTROL_NONE;
   stmUarts[uart].uart->Init.OverSampling = UART_OVERSAMPLING_16;

   HAL_UART_Init(stmUarts[uart].uart);
}

// Read 1 byte from RX FIFO, check first if exist aviable data
bool_t uartReadByte( uartMap_t uart, uint8_t* receivedByte )
{
    uint16_t *tmp;
    bool_t retVal = FALSE;
    UART_HandleTypeDef *huart;

    huart = stmUarts[uart].uart;

    /* Check that a Rx process is not already ongoing */
    if (huart->RxState == HAL_UART_STATE_READY && receivedByte != NULL) {

       /* Process Locked */
       __HAL_LOCK(huart);

       huart->ErrorCode = HAL_UART_ERROR_NONE;
       huart->RxState = HAL_UART_STATE_BUSY_RX;

       if(__HAL_UART_GET_FLAG(huart, UART_FLAG_RXNE) == SET) {
          if (huart->Init.WordLength == UART_WORDLENGTH_9B) {
             tmp = (uint16_t *) receivedByte;
             if (huart->Init.Parity == UART_PARITY_NONE) {
                *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x01FF);
             } else {
                *tmp = (uint16_t)(huart->Instance->DR & (uint16_t)0x00FF);
             }
          } else {
             if (huart->Init.Parity == UART_PARITY_NONE) {
                *receivedByte = (uint8_t)(huart->Instance->DR & (uint8_t)0x00FF);
             } else {
                *receivedByte = (uint8_t)(huart->Instance->DR & (uint8_t)0x007F);
             }
          }
          retVal = TRUE;
       }

       /* At end of Rx process, restore huart->RxState to Ready */
       huart->RxState = HAL_UART_STATE_READY;

       /* Process Unlocked */
       __HAL_UNLOCK(huart);
    }

    return retVal;
}

// Blocking Write 1 byte to TX FIFO
void uartWriteByte( uartMap_t uart, const uint8_t value )
{
   // Send byte
   HAL_UART_Transmit(stmUarts[uart].uart, &value, 1, 1);
}

// Blocking Send a string
void uartWriteString( uartMap_t uart, const char* str )
{
   while( *str != 0 ) {
      uartWriteByte( uart, (uint8_t)*str );
      str++;
   }
}

// Blocking, Send a Byte Array
void uartWriteByteArray( uartMap_t uart,
                         const uint8_t* byteArray, uint32_t byteArrayLen )
{
   uint32_t i = 0;
   for( i=0; i<byteArrayLen; i++ ) {
      uartWriteByte( uart, byteArray[i] );
   }
}

/*==================[ISR external functions definition]======================*/

#ifdef SAPI_USE_INTERRUPTS

__attribute__ ((section(".after_vectors")))

void UART1_IRQHandler(void)
{
   uartProcessIRQ( UART_1 );
}

void UART2_IRQHandler(void)
{
   uartProcessIRQ( UART_2 );
}

void UART3_IRQHandler(void)
{
   uartProcessIRQ( UART_3 );
}

#endif /* SAPI_USE_INTERRUPTS */

/*==================[end of file]============================================*/
