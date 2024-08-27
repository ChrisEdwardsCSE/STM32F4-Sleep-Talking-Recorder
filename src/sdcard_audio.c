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
 * 22-23 num channels (2) {0x02, 0x00}
 * 24-27 sample rate (32kHz) {0x80, 0x7d, 0x00, 0x00}
 * 28-31 byte rate; sample rate * Bpsample * channels (19200 it's actually 128000 tho) {0x00, 0xf4, 0x01, 0x00}
 * 32-33 Bpsample * channels (4) {0x04, 0x00}
 * 34-35 bits per sample (16) {0x10, 0x00}
 * 36-39 data {0x64, 0x61, 0x74, 0x61} data chunk header, marks beginning of data section
 * 40-43 size of data section {data section size}
 * THEN comes the data
 */
static uint8_t fil_header [44]={0x52, 0x49, 0x46, 0x46,
		0xa4, 0xa9, 0x03, 0x00,
		0x57, 0x41, 0x56, 0x45,
		0x66, 0x6d, 0x74, 0x20,
		0x10, 0x00, 0x00, 0x00,
		0x01, 0x00,
		0x01, 0x00, // this WAS 0x02, 0x00
		0x43, 0xac, 0x00, 0x00, // (44099)
		0x00, 0xf4, 0x01, 0x00,
		0x02, 0x00, // this WAS 0x04 0x00 bc 2 * 2
		0x0c, 0x00, // this WAS 0x10 0x00 (16)
		0x64, 0x61, 0x74, 0x61,
		0x80, 0xa9, 0x03, 0x00};

static FATFS fatfs; // fatfs handler;
static FIL fil; // file handler
static FRESULT f_result;
static uint32_t fil_size;
static uint8_t first_time = 0;

extern UART_HandleTypeDef huart2;

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
 * Start recording of audio data. Creates & opens the file for writing.
 * Updates the frequency-related data wtihin the file header.
 *
 * @param freq - the frequency of recording
 */
void sdcard_prepare_wav_file(uint32_t freq)
{
	// Initialize file name
	static char file_name[] = "w_000.wav";
	static uint8_t file_count = 1;
	int file_num_digits = file_count;

	uint32_t byte_rate = freq * 2 * 1; // byte rate = Bpsample * channels
	fil_header[24] = (uint8_t) (freq);
	fil_header[25] = (uint8_t) (freq >> 8);
	fil_header[26] = (uint8_t) (freq >> 16);
	fil_header[27] = (uint8_t) (freq >> 24);
	fil_header[28] = (uint8_t) (byte_rate);
	fil_header[29] = (uint8_t) (byte_rate >> 8);
	fil_header[30] = (uint8_t) (byte_rate >> 16);
	fil_header[31] = (uint8_t) (byte_rate >> 24);

	// Define wave file name
	file_name[4] = file_num_digits % 10 + 48; // ASCII 48 = '0'
	file_num_digits /= 10;
	file_name[3] = file_num_digits % 10 + 48;
	file_num_digits /= 10;
	file_name[2] = file_num_digits % 10 + 48;
	file_num_digits /= 10;
	file_count++;

	// Create the file to be written to
	f_result = f_open(&fil, file_name, FA_WRITE | FA_CREATE_ALWAYS);
	if (f_result != 0)
	{
		myprintf("Error start_recording: %d\n", f_result);
		return;
	}
	else
	{
		myprintf("start_recording success\n");
	}
	fil_size = 0;
}

/**
 * Updates wav file's meta data, ie the new size of the file and the size of the
 * data. Then closes the file and resets first_time so that sdcard_wav_write knows to
 * insert file header for the next time we record.
 */
void sdcard_close_wav_file(void)
{
	uint16_t temp_num;
	fil_size -= 8; // remove byte 0-7 cuz they're not considered as part of file
	fil_header[4] = (uint8_t) (fil_size);
	fil_header[5] = (uint8_t) (fil_size >> 8);
	fil_header[6] = (uint8_t) (fil_size >> 16);
	fil_header[7] = (uint8_t) (fil_size >> 24);
	fil_size -= 36; // remove rest of header bytes from count; only want the data
	fil_header[40] = (uint8_t) (fil_size);
	fil_header[41] = (uint8_t) (fil_size >> 8);
	fil_header[42] = (uint8_t) (fil_size >> 16);
	fil_header[43] = (uint8_t) (fil_size >> 24);


	f_lseek(&fil, 0); // jump to beginning of file to update format to write header
	f_write(&fil, (void *)fil_header, sizeof(fil_header),(UINT*)&temp_num);
	if (f_result != 0)
	{
		myprintf("Error stop_recording: %d\n", f_result);
		return;
	}
	else
	{
		myprintf("stop_recording success\n");
	}
	f_close(&fil);
	first_time = 0;
	// finished
}

/**
 * Writes data to opened wav file. Called when DMA buffer with values reaches
 * half or full capacity. When called for the firs time after a file is opened,
 * writes the file header meta data to the file. Updates file_size global variable
 *
 * @param data - the data to write to the SD Card
 * @param data_size - the size of the data to write in bytes
 */
void sdcard_wav_write(uint8_t *data, uint16_t data_size)
{
	uint16_t temp_num;
	// Initialize the file header on the first write to wav file
	if (first_time == 0)
	{
		for (int i = 0; i < 44; i++)
		{
			*(data + i) = fil_header[i];
		}
		first_time = 1;
	}
	// Write passed data to data section of file
	f_result = f_write(&fil, (void *)data, data_size, (UINT *)&temp_num);
	if (f_result != 0)
	{
		myprintf("Error sdcard_wav_write: %d\n", f_result);
		return;
	}
	else
	{
		myprintf("sdcard_wav_write success\n");
	}
	fil_size += data_size; // update wav file's data size for its header
}

/**
 * Just mount the SD Card
 */
void sdcard_init()
{
	f_result = f_mount(&fatfs, "", 1);
	if (f_result != 0)
	{
		myprintf("Error sdcard_init, %d\n", f_result);
		return;
	}
	else
	{
		myprintf("sdcard_init success\n");
	}
}

void sdcard_read_stats()
{
	DWORD free_clusters, free_sectors, total_sectors;

	FATFS* getFreeFs;

	/**
	* get num of free clusters on card. set in
	* path = "" is default drive. puts the value in free_clusters
	*/
	f_result = f_getfree("", &free_clusters, &getFreeFs);
	if (f_result != FR_OK)
	{
		myprintf("f_getfree error (&i)\r\n", f_result);
		while(1);
	}
	// total sectors = (num of FAT entries - 2) * num clusters
	total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
	// num free sectors = num free clusters
	free_sectors = free_clusters * getFreeFs->csize;

	myprintf("sd card stats:\r\n%10lu KiB total drive space. \r\n%10lu KiB availabile.\r\n", total_sectors/2, free_sectors/2);

	f_close(&fil);
}

void sdcard_check_001wav(FILINFO *file_info)
{
	const char *file_name = "w_001.wav";
	f_result = f_stat(file_name, &file_info);
	if (f_result == FR_OK)
	{
		myprintf("found w_001.wav, and deleting it");
		f_result = f_unlink(file_name);
		if (f_result == FR_OK)
		{
			myprintf("delete w_001.wav");
		}
	}
	else if (f_result == FR_NO_FILE)
	{
		myprintf("could not find w_001.wav");
	}
}
