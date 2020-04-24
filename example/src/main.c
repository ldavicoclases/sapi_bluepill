/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "sapi.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
    uint8_t i;
    uint8_t recibido;
    delay_t tiempo_encendido;
    gpioMap_t secuencia[] = {PB0, PB1, PB3, PB4, PB5, PB6, PB7,
                             PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15};

    boardInit();
    delayInit( &tiempo_encendido, 500 );
    gpioInit(PA15, GPIO_OUTPUT);

    for( i=0 ; i<sizeof(secuencia) ; i++ ) {
        gpioInit(secuencia[i], GPIO_OUTPUT);
    }

    uartInit(UART_1, 9600);
    uartInit(UART_2, 9600);
    // uartInit(UART_3, 9600);  // No funciona en el simulador porque STM32F103C6 tiene sólo 2 UARTS

    uartWriteString(UART_1, "Esta es la consola 1\n");
    uartWriteString(UART_2, "Esta es la consola 2\n");

    while (1)
    {
        if( delayRead( &tiempo_encendido ) ) {
            gpioToggle(secuencia[i]);
            i++;
            i %= sizeof(secuencia);
        }

        if( uartReadByte(UART_1, &recibido) ) {
            gpioToggle(PA15);
        }

        delay(1);
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
