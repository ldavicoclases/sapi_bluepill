#include "example_pwm.h"

#include "sapi.h"

int run_example_pwm(void) {
	uint8_t valuePwm;

	boardInit();

	uartInit(UART_1, 9600);

	pwmInit(TIM1_CH1, PWM_ENABLE);
	pwmInit(TIM1_CH1, PWM_ENABLE_OUTPUT); //PA8
	pwmInit(TIM1_CH3, PWM_ENABLE);
	pwmInit(TIM1_CH3, PWM_ENABLE_OUTPUT); //PA10
	pwmWrite(TIM1_CH1, 191);              //75% de ciclo de actividad

	if (pwmRead(TIM1_CH1, &valuePwm)) {
		if (valuePwm == 191) {
			uartWriteString(UART_1,
					"TIM1 Channel 1: PWM activo a 75% de ciclo de actividad\r\n");
		}
	}

	while (1) {
		delay(1);
	}

	return 0;
}
