/*
 * sdcard_audio.c
 *
 *  Created on: Aug 20, 2024
 *      Author: ched
 */
#include "sdcard_audio.h"


/**
 * 0-3 RIFF {0x52, 0x49, 0x46, 0x46}
 * 4-7 size of file (bytes) {data_sec size + 36}
 * 8-11 file type header "wave" {0x57, 0x41, 0x56, 0x45}
 * 12-15 "fmt" {0x66, 0x6d, 0x74, 0x20}
 * 16-19 length of format data {0x10, 0x00, 0x00, 0x00} (16)
 * 20-21 type of fomart "PCM" (1) {0x01, 0x00}
 * 22-23 num channels (1) {0x01, 0x00}
 * 24-27 sample rate (44099kHz) {0x80, 0x7d, 0x00, 0x00}
 * 28-31 byte rate; sample rate * Bpsample * channels (19200 it's actually 128000 tho) {0x00, 0xf4, 0x01, 0x00}
 * 32-33 Bpsample * channels (4) {0x04, 0x00}
 * 34-35 bits per sample (16) {0x10, 0x00}
 * 36-39 data {0x64, 0x61, 0x74, 0x61} data chunk header, marks beginning of data section
 * 40-43 size of data section {data section size}
 * THEN comes the data
 */

// 1 channel
static uint8_t fil_header [44]={
		0x52, 0x49, 0x46, 0x46,
		0xa4, 0xa9, 0x03, 0x00,
		0x57, 0x41, 0x56, 0x45,
		0x66, 0x6d, 0x74, 0x20,
		0x10, 0x00, 0x00, 0x00,
		0x01, 0x00,
		0x01, 0x00,
		0x43, 0xac, 0x00, 0x00,
		0x88, 0x58, 0x01, 0x00,
		0x02, 0x00,
		0x10, 0x00,
		0x64, 0x61, 0x74, 0x61,
		0x80, 0xa9, 0x03, 0x00};


FATFS fatfs;								// FatFS handler;
FIL fil;									// File handler
FRESULT f_result;							// Result of FatFS operations
static uint32_t fil_size;					// File size
static uint8_t initial_write_flag = 0;		// Flag indicating the first write to a file

extern UART_HandleTypeDef huart2;

/**
 * Print to console via UART
 */
void myprintf(const char *fmt, ...)
{
	static char buffer[256];
	va_list args;
	va_start(args, fmt);
	vsnprintf(buffer, sizeof(buffer), fmt, args);
	va_end(args);

	int len = strlen(buffer);
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, len, 100);
}

/**
 * Mount the SD Card.
 * Updates f_result global variable
 */
FRESULT sdcard_init()
{
	FRESULT fresult;
	fresult = f_mount(&fatfs, "", 1);
	if (fresult != FR_OK)
	{
		myprintf("sdcard_init mount error: %d\n", fresult);
		return fresult;
	}
	return fresult;
}

/**
 * Start recording of audio data. Creates & opens the file for writing.
 * Updates the frequency-related data wtihin the file header.
 *
 * @param freq - the frequency of recording
 * @param data - buffer with dummy bytes to pad the beginning of the recording
 * @return FRESULT
 */
FRESULT sdcard_prepare_wav_file(uint32_t freq)
{
	// Initialize file name
	static char file_name_prepare[] = "w_00.wav";
	static uint8_t file_digits_prepare = 0;
	uint16_t bytes_written;
	FRESULT fresult;

	uint32_t byte_rate = freq * 2 * 1; // byte rate = Bpsample * channels
	fil_header[24] = (uint8_t) (freq);
	fil_header[25] = (uint8_t) (freq >> 8);
	fil_header[26] = (uint8_t) (freq >> 16);
	fil_header[27] = (uint8_t) (freq >> 24);
	fil_header[28] = (uint8_t) (byte_rate);
	fil_header[29] = (uint8_t) (byte_rate >> 8);
	fil_header[30] = (uint8_t) (byte_rate >> 16);
	fil_header[31] = (uint8_t) (byte_rate >> 24);

	fil_size = 0;

	// Define wave file name
	file_name_prepare[2] = file_digits_prepare / 10 + 48; // ASCII = '0'
	file_name_prepare[3] = file_digits_prepare % 10 + 48;
	file_digits_prepare++;

	// Create the file to be written to
	fresult = f_open(&fil, file_name_prepare, FA_WRITE | FA_CREATE_ALWAYS);
	if (fresult != 0)
	{
		myprintf("sdcard_prepare_wav_file open error: %d\n", fresult);
		return fresult;
	}

	// Write file header to file
	fresult = f_write(&fil, (void *)fil_header, sizeof(fil_header),(UINT*)&bytes_written);
	if (fresult != FR_OK)
	{
		myprintf("sdcard_prepare_wav_file write error: %d\n", fresult);
	}

	return fresult;
}

/**
 * Updates wav file's meta data, ie the new size of the file and the size of the
 * data. Then closes the file and resets initial_write_flag so that sdcard_wav_write knows to
 * insert file header for the next time we record.
 *
 * @return FRESULT
 */
FRESULT sdcard_close_wav_file(void)
{
	uint16_t bytes_written;
	FRESULT fresult;

	fil_size -= 8; // Remove bytes 0-7 as they're not considered as part of file
	fil_header[4] = (uint8_t) (fil_size);
	fil_header[5] = (uint8_t) (fil_size >> 8);
	fil_header[6] = (uint8_t) (fil_size >> 16);
	fil_header[7] = (uint8_t) (fil_size >> 24);
	fil_size -= 36; // Remove rest of header bytes from count; only want the size of data
	fil_header[40] = (uint8_t) (fil_size);
	fil_header[41] = (uint8_t) (fil_size >> 8);
	fil_header[42] = (uint8_t) (fil_size >> 16);
	fil_header[43] = (uint8_t) (fil_size >> 24);


	fresult = f_lseek(&fil, 0); // Jump to beginning of file to update format to write header
	if (fresult != FR_OK)
	{
		myprintf("sdcard_close_wav_file pointer jump error: %d\n", fresult);
		return fresult;
	}

	f_write(&fil, (void *)fil_header, sizeof(fil_header),(UINT*)&bytes_written); // Write file header
	if (fresult != FR_OK)
	{
		myprintf("sdcard_close_wav_file write error: %d\n", fresult);
		return fresult;
	}

	fresult = f_close(&fil);
	if (fresult != FR_OK)
	{
		myprintf("sdcard_close_wav_file close error: %d\n", fresult);
		return fresult;
	}

	initial_write_flag = 0; // Reset initial write flag for next file

	return fresult;
}

/**
 * Writes data to open wav file.
 * Called when DMA double buffer reaches half or full capacity.
 * When called for the first time after a file is opened, writes the
 * file header metadata to the file.
 *
 * @param data - the data to write to the SD Card
 * @param data_size - the size of the data to write in bytes
 */
FRESULT sdcard_wav_write(uint8_t *data, uint16_t data_size)
{
	uint16_t bytes_written;
	FRESULT fresult;

	// Initialize the file header on the first write to wav file
	if (initial_write_flag == 0)
	{
		for (int i = 0; i < 44; i++)
		{
			*(data + i) = fil_header[i];
		}
		initial_write_flag = 1;
	}

	// Write passed data to data section of file
	fresult = f_write(&fil, (void *)data, data_size, (UINT *)&bytes_written);
	if (fresult != FR_OK)
	{
		myprintf("sdcard_wav_write write error: %d\n", fresult);
		return fresult;
	}
	else
	{
		fil_size += data_size; // update file's header data size
		return fresult;
	}
}


/**
 * Erases all .wav files from SD Card
 *
 * @return FRESULT - FR_OK if files successfully deleted, Error otherwise
 */
FRESULT sdcard_clear_files(void)
{
	static char file_name_delete[] = "w_00.wav";
	static uint8_t file_digits_delete = 0;
	FRESULT fresult = FR_OK;
	while (fresult == FR_OK)
	{
		file_name_delete[2] = file_digits_delete / 10 + 48; // ASCII = '0'
		file_name_delete[3] = file_digits_delete % 10 + 48;
		fresult = f_unlink(file_name_delete);
		file_digits_delete++;
	}
	// If any error besides no file found error, signify error
	if (fresult != FR_NO_FILE)
	{
		printf("sdcard_clear_files error: %d\n", fresult);
		return fresult;
	}
	else
	{
		return FR_OK;
	}
}
