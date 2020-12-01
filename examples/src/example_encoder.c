#include "example_encoder.h"

#include "sapi.h"

int run_example_encoder(void) {
	int i;
	uint16_t valueEncoder;

	uartInit(UART_1, 9600);
	boardInit();

	encoderInit(ENC_TIM2, ENCODER_COUNT_CHANNEL_ALL); //PA0 y PA1

	if (encoderRead(ENC_TIM2, &valueEncoder)) {
		uartWriteString(UART_1, "TIM2 Channel 1-2: Encoder:");
		for (i = 10000; i > 0; i /= 10) {
			uartWriteByte(UART_1, ((valueEncoder / i) % 10 + '0')); //integer to array
		}
		uartWriteString(UART_1, "\r\n");
	}

	while (1) {
		delay(1);
	}
}
