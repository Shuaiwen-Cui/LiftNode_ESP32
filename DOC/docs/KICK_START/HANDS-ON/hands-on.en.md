# HANDS ON

## Program Preparation

Find some programs for testing, here, I use the following repository: 

```shell
git clone --recursive https://gitee.com/vi-iot/esp32-board.git
```

## Hardware Preparation

Connect the ESP32 development board to the computer

## Select a Program

Enter the repository `esp32-board` we just downloaded, find a program, such as `helloworld`, enter the directory:

```shell
cd esp32-board
cd helloworld
```

## Set the Development Board Type

!!! tip "ESP32 Environment Issue"
    Type in `get_idf` to call the ESP32 environment if you are not currently in the ESP32 environment. Note that, the use of `get_idf` is only available after you properly set up the alias in the SHELL as indicated in the last tutorial.

If it is a general ESP32 development board, you can ignore this step, but if it is another development board, it is better to specify the development board type, such as esp32-s3:

```shell
idf.py set-target esp32s3
```

## Compile the Program

```shell
idf.py build
```

## Flash the Program

```shell
idf.py flash
```

Note that you can view the USB port where the device is located:

```shell
ls /dev/tty*
```

By comparing the linked/non-linked, find the port where the device is located, such as `/dev/ttyACM1`, and then flash the program:

```shell
idf.py -p /dev/ttyACM1 flash
```

## Monitor the Program Running

```shell
idf.py monitor
```

or you can use a specific port

```shell
idf.py -p /dev/ttyACM1 monitor
```

## Exit Monitoring

Press `Ctrl + ]` to exit monitoring