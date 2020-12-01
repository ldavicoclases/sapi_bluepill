#include "example_i2c.h"

#include "sapi.h"

int run_example_i2c(void) {
	uint8_t reg[] = { 0x08, 0x00 };
	uint8_t data_to_write[] = { 0x08, 0x00, 0xAA, 0xAB, 0xAC, 0xAD };
	uint8_t data_to_read[] = { 0xFF, 0xFF, 0xFF, 0xFF };

	boardInit();

	/*
	 *using  24LC64 eeprom i2c memory, slave address 0xA0, writing 4 bytes
	 * to memory address 0x0800 and reading the same address after the write.
	 */
	i2cInit(I2C_1, 100000); // uses PB6 and PB7
	i2cWrite(I2C_1, 0xA0, data_to_write, sizeof(data_to_write), TRUE);
	delay(50); //delay to eeprom write
	i2cRead(I2C_1, 0xA0, reg, sizeof(reg), TRUE, data_to_read,
			sizeof(data_to_read), TRUE);

	while (1) {
		delay(1);
	}
}
