#include "example_adc.h"

#include "sapi.h"

int run_example_adc(void) {
	delay_t tiempo_conversion;

	boardInit();

	uartInit(UART_1, 9600);

	delayInit(&tiempo_conversion, 50);

	adcInit(ADC_ENABLE);

	while (1) {
		if (delayRead(&tiempo_conversion)) {
			uartWriteString(UART_1, "ADC0: ");
			uartWriteByte(UART_1, adcRead(CH0) / 1024 + '0');
			uartWriteString(UART_1, "\r\n");
		}
	}
}
