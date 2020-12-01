#include "example_gpio.h"

#include "sapi.h"

int run_example_gpio(void) {
	int i;
	delay_t tiempo_encendido;
	gpioMap_t secuencia[] = { PB0, PB1, PB3, PB4, PB5, PB8, PB9, PB10, PB11,
			PB12, PB13, PB14, PB15 };

	boardInit();

	delayInit(&tiempo_encendido, 500);

	for (i = 0; i < sizeof(secuencia); i++) {
		gpioInit(secuencia[i], GPIO_OUTPUT);
	}

	while (1) {
		if (delayRead(&tiempo_encendido)) {
			gpioToggle(PC13);
			gpioToggle(secuencia[i]);
			i++;
			i %= sizeof(secuencia);
		}
	}
}
