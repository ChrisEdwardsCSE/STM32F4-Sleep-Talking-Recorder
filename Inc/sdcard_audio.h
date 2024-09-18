/*
 * sdcard_audio.h
 *
 *  Created on: Aug 20, 2024
 *      Author: ched
 */

#ifndef INC_SDCARD_AUDIO_H_
#define INC_SDCARD_AUDIO_H_

#include "stm32f4xx_hal.h"

#include "fatfs.h"
#include "stdarg.h"
#include "string.h"
#include <stdio.h>

void myprintf(const char *fmt, ...);

FRESULT sdcard_init(void);

FRESULT sdcard_prepare_wav_file(uint32_t freq);

FRESULT sdcard_close_wav_file(void);

FRESULT sdcard_wav_write(uint8_t *data, uint16_t data_size);

/**
 * Erases all .wav files from SD Card
 * @return FRESULT - FR_OK if files successfully deleted, Error otherwise
 */
FRESULT sdcard_clear_files();

#endif /* INC_SDCARD_AUDIO_H_ */
