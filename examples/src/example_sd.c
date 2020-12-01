#include "sapi.h"

#include "fatfs.h"
#include "fatfs_sd.h"

#include "string.h"
#include "stdio.h"

FATFS fs;  // file system
FIL fil; // File
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

/**** capacity related *****/
FATFS *pfs;
DWORD fre_clust;
uint32_t total, free_space;

#define BUFFER_SIZE 128
char buffer[BUFFER_SIZE];  // to store strings..

int i=0;

int run_example_sd(void)
{
	boardInit();
	gpioInit(PC13, GPIO_OUTPUT);
	gpioInit(PB0, GPIO_OUTPUT);

	uartInit(UART_1, 9600);

	spiInit(SPI_1);

	FATFS_LinkDriver(&USER_Driver, USERPath);

	fresult = f_mount(&fs, "/", 1);
	if (fresult != FR_OK) uartWriteString(UART_1,"ERROR!!! in mounting SD CARD...\n\n");
	else uartWriteString(UART_1, "SD CARD mounted successfully...\n\n");

	/*************** Card capacity details ********************/

	/* Check free space */
	f_getfree("", &fre_clust, &pfs);

	total = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
	sprintf (buffer, "SD CARD Total Size: \t%lu\n", total);
	uartWriteString(UART_1, buffer);
	memset(buffer, '\0', BUFFER_SIZE);

	free_space = (uint32_t)(fre_clust * pfs->csize * 0.5);
	sprintf (buffer, "SD CARD Free Space: \t%lu\n\n", free_space);
	uartWriteString(UART_1, buffer);
	memset(buffer, '\0', BUFFER_SIZE);

	/************* The following operation is using PUTS and GETS *********************/

	/* Open file to write/ create a file if it doesn't exist */
	fresult = f_open(&fil, "file3.txt", FA_OPEN_ALWAYS | FA_READ | FA_WRITE);

	/* Writing text */
	f_puts("This data is from the FILE1.txt. And it was written using ...f_puts... ", &fil);

	/* Close file */
	fresult = f_close(&fil);

	if (fresult == FR_OK) uartWriteString(UART_1, "File1.txt created and the data is written \n");

	/* Open file to read */
	fresult = f_open(&fil, "file3.txt", FA_READ);

	/* Read string from the file */
	f_gets(buffer, f_size(&fil), &fil);

	uartWriteString(UART_1, "File1.txt is opened and it contains the data as shown below\n");
	uartWriteString(UART_1, buffer);
	uartWriteString(UART_1, "\n\n");

	/* Close file */
	f_close(&fil);

	memset(buffer, '\0', BUFFER_SIZE);

	/**************** The following operation is using f_write and f_read **************************/

	/* Create second file with read write access and open it */
	fresult = f_open(&fil, "file2.txt", FA_CREATE_ALWAYS | FA_WRITE);

	/* Writing text */
	strcpy (buffer, "This is File2.txt, written using ...f_write... and it says Hello from Controllerstech\n");

	fresult = f_write(&fil, buffer, strlen(buffer), &bw);

	uartWriteString(UART_1,"File2.txt created and data is written\n");

	/* Close file */
	f_close(&fil);

	// clearing buffer to show that result obtained is from the file
	memset(buffer, '\0', BUFFER_SIZE);

	/* Open second file to read */
	fresult = f_open(&fil, "file2.txt", FA_READ);
	if (fresult == FR_OK) uartWriteString(UART_1,"file2.txt is open and the data is shown below\n");

	/* Read data from the file
	 * Please see the function details for the arguments */
	f_read (&fil, buffer, f_size(&fil), &br);
	uartWriteString(UART_1,buffer);
	uartWriteString(UART_1,"\n\n");

	/* Close file */
	f_close(&fil);

	memset(buffer, '\0', BUFFER_SIZE);

	/*********************UPDATING an existing file ***************************/

	/* Open the file with write access */
	fresult = f_open(&fil, "file2.txt", FA_OPEN_EXISTING | FA_READ | FA_WRITE);

	/* Move to offset to the end of the file */
	fresult = f_lseek(&fil, f_size(&fil));

	if (fresult == FR_OK) uartWriteString(UART_1, "About to update the file2.txt\n");

	/* write the string to the file */
	fresult = f_puts("This is updated data and it should be in the end", &fil);

	f_close (&fil);

	memset(buffer, '\0', BUFFER_SIZE);

	/* Open to read the file */
	fresult = f_open (&fil, "file2.txt", FA_READ);

	/* Read string from the file */
	fresult = f_read (&fil, buffer, f_size(&fil), &br);
	if (fresult == FR_OK) uartWriteString(UART_1, "Below is the data from updated file2.txt\n");
	uartWriteString(UART_1, buffer);
	uartWriteString(UART_1, "\n\n");

	/* Close file */
	f_close(&fil);

	memset(buffer, '\0', BUFFER_SIZE);

	/*************************REMOVING FILES FROM THE DIRECTORY ****************************/

	//fresult = f_unlink("/file1.txt");
	//if (fresult == FR_OK) uartWriteString(UART_1,"file1.txt removed successfully...\n");

	//fresult = f_unlink("/file2.txt");
	//if (fresult == FR_OK) uartWriteString(UART_1,"file2.txt removed successfully...\n");

	/* Unmount SDCARD */
	//fresult = f_mount(NULL, "/", 1);
	//if (fresult == FR_OK) uartWriteString(UART_1,"SD CARD UNMOUNTED successfully...\n");

	while (1)
	{
		gpioWrite(PC13, ON);
	}
}
