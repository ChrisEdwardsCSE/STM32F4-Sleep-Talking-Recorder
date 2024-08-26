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

void sdcard_init(void);

void start_recording(uint32_t freq);

void stop_recording(void);

void sdcard_wav_write(uint8_t *data, uint16_t data_size);

#endif /* INC_SDCARD_AUDIO_H_ */
