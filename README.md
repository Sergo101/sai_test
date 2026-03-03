#stm32h7 sai interface

#Hardware
This project operate at STM32H750IBK6 dev board from aliexpress
STM32H7 Core Board STM32H750IBK6 Core Board STM32H7 Development Minimum System Board
https://ali.click/1xjg11i

    PC0     ------> SAI2_FS_B
    PA1     ------> SAI2_MCLK_B
    PA0     ------> SAI2_SD_B
    PA2     ------> SAI2_SCK_B

    PG11    ------> SPI1_SCK
    PA6     ------> SPI1_MISO
    PA7     ------> SPI1_MOSI
    PD3     ------> DAC_SPI_ChipSelect
Connect DAC PCM5122 to I2S & SPI interfaces to change output volume thru SPI or other I2S DAC w/o config interfaces

#Summary
App search wav files at SDCARD or USB stick and set them at playlist. 
For now app supports all bitrates but only for 16 and 32 bit samplerate

#Build & Debug
Easiest way to build and debug this project is:
1 install STM32_CubeCLT package
2 install vscode
3 install STM32extencion pack for vscode
4 config cmake preset & build
5 connect st-link v2, press debug
