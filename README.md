Currently building a sleep talking recorder on an STM32F4 MCU

Uses a MEMS microphone to take in analog sound pressure values, sample at 44.1kHz with the ADC & DMA & Timer, then write to an SD Card as .wav files over SPI if it detects noise (the sleep talking). I'm going to try to make it low power when it's off or just in the "waiting for noise" state. Also hope to implement DAC writing to an amplifier + speaker so it can play back the sounds it detected.

https://youtu.be/M0g0gUcA7BM
