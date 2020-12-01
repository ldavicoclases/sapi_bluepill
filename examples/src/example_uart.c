#include "example_uart.h"

#include "sapi.h"

int run_example_uart(void) {
	uint8_t recibido;

	boardInit();

	uartInit(UART_1, 9600);

	while (1) {
		if (uartReadByte(UART_1, &recibido)) {
			uartWriteByte(UART_1, recibido);
		}

		delay(1);
	}
}
