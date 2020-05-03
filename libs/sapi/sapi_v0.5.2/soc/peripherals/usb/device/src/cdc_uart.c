/* Copyright 2020, Nahuel Espinosa
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

/* Date: 2020-05-03 */

#include "stm32f1xx.h"
#include "stm32f1xx_hal.h"
#include "usbd_def.h"
#include "usbd_desc.h"
#include "usbd_core.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include "cdc_uart.h"
#include "sapi_circularBuffer.h"
#include "sapi_delay.h"

#define CDC_BUFFER_SIZE 100

/*****************************************************************************
 * Private types/enumerations/variables
 ****************************************************************************/

USBD_HandleTypeDef hUsbDeviceFS;

circularBufferNew( cdcBuffer, 1, CDC_BUFFER_SIZE );

/*****************************************************************************
 * Public types/enumerations/variables
 ****************************************************************************/
extern PCD_HandleTypeDef hpcd_USB_FS;

/*****************************************************************************
 * Private functions
 ****************************************************************************/

void CDC_RxCallback(uint8_t *Buf, uint32_t Len) {
   uint8_t i = 0;
   for( i = 0 ; i < Len ; i++ ) {
      circularBufferWrite(&cdcBuffer, &Buf[i]);
   }
}

/*****************************************************************************
 * Public functions
 ****************************************************************************/
void cdcUartInit( uint32_t baudRate )
{
    circularBufferConfig(cdcBuffer, 1, CDC_BUFFER_SIZE);

    USBD_Init(&hUsbDeviceFS, &FS_Desc, DEVICE_FS);
    USBD_RegisterClass(&hUsbDeviceFS, &USBD_CDC);
    USBD_CDC_RegisterInterface(&hUsbDeviceFS, &USBD_Interface_fops_FS);
    USBD_Start(&hUsbDeviceFS);
}

void cdcUartWriteByte( const uint8_t value )
{
    delay_t timeoutDelay;

    delayConfig(&timeoutDelay, 10);

    while( CDC_Transmit_FS(&value, 1) == USBD_BUSY && !delayRead(&timeoutDelay) );
}

bool_t cdcUartReadByte( uint8_t* receivedByte )
{
    bool_t retVal = TRUE;

    if( circularBufferRead( &cdcBuffer, receivedByte ) == CIRCULAR_BUFFER_EMPTY ) {
        retVal = FALSE;
    } else {
        retVal = TRUE;
    }

    return retVal;
}

//-------------------------------------------------------------
// Interrupts
//-------------------------------------------------------------

/**
  * @brief This function handles USB low priority or CAN RX0 interrupts.
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 0 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_FS);
  /* USER CODE BEGIN USB_LP_CAN1_RX0_IRQn 1 */

  /* USER CODE END USB_LP_CAN1_RX0_IRQn 1 */
}


#ifdef SAPI_USE_INTERRUPTS

// UART Global Interrupt Enable/Disable
void cdcUartInterrupt( bool_t enable )
{
   
}

// UART Interrupt event Enable and set a callback
void cdcUartCallbackSet( uartEvents_t event, callBackFuncPtr_t callbackFunc, 
                         void* callbackParam )
{
   
}
                 
// UART Interrupt event Disable
void cdcUartCallbackClr( uartEvents_t event )
{
   
}

#endif /* SAPI_USE_INTERRUPTS */
//-------------------------------------------------------------
