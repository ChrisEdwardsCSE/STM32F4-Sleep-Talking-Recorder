Currently building a sleep talking recorder on an STM32F4 MCU

It's gonna use a MEMS microphone to take in analog sound pressure values, sample with the ADC & DMA, then write to an SD Card as .wav files over SPI if it detects noise (the sleep talking). I'm going to try to make it low power when it's off or just in the listening state. Also hope to implement DAC writing to an amplifier + speaker so it can play back the sounds it detected.
