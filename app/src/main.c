/*
 * Nombre del archivo:   main.c
 * Autor:
 *
 * Descripción:
 */

#include "sapi.h"
#include "examples.h"

/* ------------------------ Definiciones ------------------------------------ */

/* ------------------------ Prototipos de funciones ------------------------- */

/* ------------------------ Implementación de funciones --------------------- */

int main(void) {
	boardInit();

	// Para probar algún ejemplo se puede llamar la función que corresponda:
	// run_example_uart();
	// run_example_rtc();
	run_example_usb();

	while (1) {
		delay(1);
	}

    // NO DEBE LLEGAR NUNCA AQUÍ, debido a que este programa se ejecuta
    // directamente sobre un microcontrolador y no es llamado por un ningún
    // sistema operativo, como en el caso de un programa para PC.
}

/* ------------------------ Fin de archivo ---------------------------------- */
