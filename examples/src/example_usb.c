#include "example_usb.h"

#include "sapi.h"

int run_example_usb(void) {
	uint8_t recibido;

	boardInit();

	cdcUartInit(115200);

	while (1) {
		if (cdcUartReadByte(&recibido)) {
			cdcUartWriteByte(recibido);
		}

		delay(1);
	}
}
