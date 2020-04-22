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
    gpioMap_t secuencia[] = {PA0, PA1, PA2, PA3, PA4, PA5, PA6, PA7,
                             PA8, PA9, PA10, PA11, PA12, PA15,
                             PB0, PB1, PB3, PB4, PB5, PB6, PB7,
                             PB8, PB9, PB10, PB11, PB12, PB13, PB14, PB15};

    boardInit();

    for( i=0 ; i<sizeof(secuencia) ; i++ ) {
    	gpioInit(secuencia[i], GPIO_OUTPUT);
    }

    while (1)
    {
    	for( i=0 ; i<sizeof(secuencia) ; i++ ) {
			gpioToggle(secuencia[i]);
			HAL_Delay(100);
		}
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
