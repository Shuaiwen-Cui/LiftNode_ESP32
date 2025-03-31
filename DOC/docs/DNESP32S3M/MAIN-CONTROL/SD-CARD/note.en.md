# TF CARD (SD CARD) Operation - SPI Mode

!!! warning
    For this chapter, I don't recommend you to refer to the code from the Alientek. After testing, I found that the program doesn't seem to work. So I referred to the content on the Internet and rewrote the content of this chapter.

## Introduction

!!! info "SD Card Overview"
    Many microcontroller systems require high-capacity storage devices to store data. Common options include USB drives, FLASH chips, and SD cards. Each has its advantages, but overall, SD cards are the most suitable for microcontroller systems. They offer large capacities (over 32GB), support SPI/SDIO interfaces, and come in various sizes (standard SD card and Micro SD card sizes), meeting the requirements of different applications.

    With just a few IO pins, you can extend storage capacity to 32GB or more, with a wide range of choices from tens of megabytes to tens of gigabytes. They are easy to replace, simple to program, and are the preferred choice for large-capacity external storage in microcontroller systems.

    The ZD Atom ESP32-S3 minimal system board uses a Micro SD card interface with a self-locking card slot. The SD SPI host driver is implemented based on the SPI Master Driver. With the SPI host driver, the SD card and other SPI devices can share the same SPI bus. The SPI host driver handles exclusive access from different tasks.

!!! info
    A complete SD card operation process involves the host (e.g., a microcontroller) initiating a "command." The SD card determines whether to send a response or data based on the command's content. For read/write operations, the host must also send a command to stop reading/writing data to end the operation. This means that after the host sends a command, the SD card may not respond or provide data, depending on the command's meaning.

!!! info "Card Modes"
    SD cards have two effective operation modes: "Card Identification Mode" and "Data Transfer Mode." Based on the operation method, there are SD mode and SPI mode. The ZD Atom ESP32-S3 minimal system board uses SPI mode.

## Key Steps

1. SD card initialization (SPI mode/SDIO mode)
2. SD card read/write operations

!!! warning
    The ESP32's SDIO driver mode and SPI driver mode are not compatible. You can only choose one driver mode.

## Use Case

This experiment introduces the following functionality: After a series of initializations, an SD card initialization condition is used within a `while` loop to check whether the SD card is successfully initialized. If initialization succeeds, SD card parameters will be output to the serial terminal or VSCode terminal, and the total capacity and remaining capacity will be displayed on the LCD. The LED blinks to indicate the program is running.

## Resources Used

1. **LED**
   - LED: IO0
2. **0.96-inch LCD**
3. **SD Card**
   - CS: IO2
   - SCK: IO12
   - MOSI: IO11
   - MISO: IO13

## Connection Diagram

This experiment connects the SD card to the ESP32-S3 using the SPI interface. The DNESP32S3M minimal system board has an onboard Micro SD card slot for connecting the SD card. The schematic for connecting the SD card to the ESP32-S3 is shown below.

![TF](TF.png)

## Dependencies

![dep](dep-07-spi-sd.png)