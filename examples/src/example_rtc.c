#include "example_rtc.h"

#include "sapi.h"

int run_example_rtc(void) {
	delay_t tiempo_encendido;
	rtc_t init_time, current_time;

	char time_string[] = "HH:MM:SS";

	boardInit();

	delayInit(&tiempo_encendido, 500);

	uartInit(UART_1, 9600);

	init_time.hour = 20;
	init_time.min = 6;
	init_time.sec = 0;
	init_time.mday = 26;
	init_time.month = 4;
	init_time.year = 2020;

	rtcInit();
	rtcWrite(&init_time);

	while (1) {
		if (delayRead(&tiempo_encendido)) {
			rtcRead(&current_time);

			uartWriteString(UART_1,
					hourMinSecToStringHHMMSS(current_time.hour,
							current_time.min, current_time.sec, time_string));
			uartWriteString(UART_1, "\r\n");
		}
	}

	return 0;
}
