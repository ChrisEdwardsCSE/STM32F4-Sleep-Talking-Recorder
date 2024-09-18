# Sleep Talking Recorder Device on STM32F4 MCU
**[Demo Video](https://youtu.be/M0g0gUcA7BM)**

This project records all the sounds it detects during your sleep, saves them in .wav files, and plays them back to you in the morning. It uses an analog output MEMS microphone (ADC), an SD Card (FatFS, SPI), and an amplifier (I2S) and loudspeaker to playback the audio. The device was built on a STM32F4 microcontroller board using STM's HAL and drivers that I wrote for its peripherals.

The device operates as a state machine with its 4 states being: Off, Listening, Recording, and Playback. Users switch the device into Listening mode before they sleep and the device begiins taking microphone readings. If the device detects significant nosie, it transitions to the Recording state and writes the sounds it picks up to the SD Card. When the user wakes up, they can switch the device to the Playback state and the recordings that were taken will play.

The device is low power and energy efficient. The majority of its operating state is just listening for sound, so I made use of the microcontroller's supported low power states to conserve energy. I also made use of the DMA capabilities and interrupts to offload menial tasks from the CPU.



## Project Details
### Microphone
An analog output MEMS microphone is used to pick up the sound. The onboard ADC peripheral samples these readings at a sampling rate of 44.1kHz and sends the values to a double buffer via a DMA stream. The sampling rate is performed with a timer interrupt set to 44.1kHz.

### SD Card
The SD Card is configured with the FatFS filesystem and saves recordings as .wav files. The MCU communicates with the SD Card using a driver that I wrote for the onboard SPI peripheral. Any sounds detected above a specified threshold are saved to the SD Card, while silence is ignored.

### Amplifier & Loudspeaker
The amplifier receives digital data, performs digital-to-analog conversion, and outputs the signal to the loudspeaker. The MCU reads the audio files from the SD Card to the amplifier using the I2S peripheral via the DMA so that recordings can play in real time.


