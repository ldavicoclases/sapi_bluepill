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
    uint8_t recibido,value;
    uint8_t reg = 0x06;
    uint8_t data_in[] = {0x06,0xAA};
    uint8_t data_out = 0x80;
    uint8_t slave_addr=0x94;
    delay_t tiempo_encendido, tiempo_conversion;
    gpioMap_t secuencia[] = {PB1, PB3, PB4, PB5, PB7,
                             PB8, PB9,PB10, PB11, PB12, PB13, PB14, PB15};
    char time_string[] = "HH:MM:SS";
    rtc_t init_time, current_time;

    init_time.hour = 20;
    init_time.min = 6;
    init_time.sec= 0;
    init_time.mday = 26;
    init_time.month = 4;
    init_time.year = 2020;


    boardInit();

    gpioInit(PB0,GPIO_OUTPUT); //using a MAX44009 light sensor for testing I2C, an address pin should be driven low
    gpioWrite(PB0,RESET);

    i2cInit(I2C_1,100000);

    i2cWrite(I2C_1,slave_addr,data_in,sizeof(data_in),TRUE);
    i2cRead(I2C_1,slave_addr,&reg,sizeof(reg),TRUE,&data_out,sizeof(data_out),TRUE);
    rtcInit();

    rtcWrite(&init_time);

    delayInit(&tiempo_encendido, 500);
    delayInit(&tiempo_conversion, 50);

    gpioInit(PA15, GPIO_OUTPUT);
    gpioInit(PC13, GPIO_OUTPUT);

    for( i=0 ; i<sizeof(secuencia) ; i++ ) {
        gpioInit(secuencia[i], GPIO_OUTPUT);
    }

    uartInit(UART_1, 9600);
//  uartInit(UART_2, 9600);  // No se puede usar con las entradas ADC
 // uartInit(UART_3, 9600);  // No funciona en el simulador porque STM32F103C6 tiene sólo 2 UARTS

    cdcUartInit(9600); //no entra en la memoria del STM32F103C6

    adcInit(ADC_ENABLE);

    uartWriteString(UART_1, "Esta es la consola 1\r\n");
    uartWriteString(UART_2, "Esta es la consola 2\r\n");

    pwmInit(TIM4_CH1,PWM_ENABLE);
    pwmInit(TIM4_CH1,PWM_ENABLE_OUTPUT); //PB6

    pwmWrite(TIM4_CH1,191); //75% de ciclo de actividad
    if(pwmRead(TIM4_CH1, &value)){
    	if (value == 191){
    	    uartWriteString(UART_1, "TIM4 Channel 1: PWM activo a 75% de ciclo de actividad\r\n");
    	}
    }
    i = 0;

    while (1)
    {
       if( delayRead(&tiempo_encendido) ) {
            gpioToggle(PC13);
            gpioToggle(secuencia[i]);
            i++;
            i %= sizeof(secuencia);

            rtcRead(&current_time);

            uartWriteString(UART_1, hourMinSecToStringHHMMSS(current_time.hour, current_time.min, current_time.sec, time_string));
            uartWriteString(UART_1, "\r\n");
        }

        if( cdcUartReadByte(&recibido) ) {
            cdcUartWriteByte(recibido);     // uart echo
        }

        if( delayRead(&tiempo_conversion) ) {
            uartWriteString(UART_1, "ADC0: ");
            uartWriteByte(UART_1, adcRead(CH0) / 1024 + '0');
            uartWriteString(UART_1, "\r\n");
        }

        if( uartReadByte(UART_1, &recibido) ) {
            uartWriteByte(UART_1, recibido);
            gpioToggle(PA15);
        }

        delay(1);
    }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
