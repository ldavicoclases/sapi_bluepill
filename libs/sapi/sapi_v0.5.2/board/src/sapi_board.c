/* Copyright 2015-2016, Eric Pernia.
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

/* Date: 2020-04-21 */

/*==================[inclusions]=============================================*/

#include "sapi_board.h"

#include "sapi_tick.h"
#include "sapi_gpio.h"
#include "sapi_cyclesCounter.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

void SystemClock_Config(void);

/*==================[external functions definition]==========================*/

/* Set up and initialize board hardware */
void boardInit(void)
{
   // Reset of all peripherals, Initializes the Flash interface and the Systick
   HAL_Init();

   // Configure the system clock
   SystemClock_Config();

   // Read clock settings and update SystemCoreClock variable
   SystemCoreClockUpdate();

   /**Configure the Systick interrupt time
   */
   HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

   /**Configure the Systick
   */
   HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

   /* SysTick_IRQn interrupt configuration */
   HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);

   // cyclesCounterInit( SystemCoreClock );

   // Inicializar el conteo de Ticks con resolucion de 1ms (si no se usa freeRTOS)
   #ifndef USE_FREERTOS
      tickInit( 1 );
   #endif

   __HAL_RCC_AFIO_CLK_ENABLE();
   __HAL_RCC_PWR_CLK_ENABLE();

   /* System interrupt init*/

   /** NOJTAG: JTAG-DP Disabled and SW-DP Enabled
   */
   __HAL_AFIO_REMAP_SWJ_NOJTAG();

   // GPIO Ports Clock Enable
   __HAL_RCC_GPIOA_CLK_ENABLE();
   __HAL_RCC_GPIOB_CLK_ENABLE();
   __HAL_RCC_GPIOC_CLK_ENABLE();
}

/* System Clock Configuration */
void SystemClock_Config(void)
{
   RCC_OscInitTypeDef RCC_OscInitStruct = {0};
   RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
   RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

   /** Initializes the CPU, AHB and APB busses clocks
   */
   RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE|RCC_OSCILLATORTYPE_LSE;
   RCC_OscInitStruct.HSEState       = RCC_HSE_ON;
   RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
   RCC_OscInitStruct.LSEState       = RCC_LSE_ON;
   RCC_OscInitStruct.HSIState       = RCC_HSI_ON;
   RCC_OscInitStruct.PLL.PLLState   = RCC_PLL_ON;
   RCC_OscInitStruct.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
   RCC_OscInitStruct.PLL.PLLMUL     = RCC_PLL_MUL6;
   HAL_RCC_OscConfig(&RCC_OscInitStruct);

   /** Initializes the CPU, AHB and APB busses clocks
   */
   RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                    |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
   RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
   RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
   RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
   RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

   HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);

   PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_ADC
                                       |RCC_PERIPHCLK_USB;
   PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSE;
   PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
   PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
   HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit);
}

/*==================[end of file]============================================*/
