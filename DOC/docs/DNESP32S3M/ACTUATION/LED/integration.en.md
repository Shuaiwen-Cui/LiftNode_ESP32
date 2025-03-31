# INTEGRATION

## Introduction

!!! note
    In this section, we introduce how to integrate the LED driver into the project.

## Create a New Component

!!! warning
    Before you create a component in the `driver` directory, make sure you have incorporated the `driver` directory into the project by adding the path to the `driver` directory to the `EXTRA_COMPONENT_DIRS` variable in the project-level `CMakeLists.txt` file.

Open the project in VSCode, open the integrated terminal, and type in

```bash
get_idf 
```
to actiate the ESP-IDF environment. Then type in the following command to create a new component named `led`:

```bash
idf.py -C driver create-component led
```

The command above means to create a new component named `led` in the `driver` directory. After executing the command, you will see a new directory named `led` in the `driver` directory. And the command will automatically generate the `CMakeLists.txt` file for the new component, as well as the `led.h` and `led.c` files.

Alternatively, you can mannually create the `led` directory in the `driver` directory, and then create the `CMakeLists.txt`, `led.h`, and `led.c` files in the `led` directory.

## Replace the Code

> Component Level

Replace the code in this section into the `CMakelists.txt`, `led.h`, and `led.c` files under the component.

> Project Level

Replace the code in `main.c` with the corresponding code. No need to change the `CMakeLists.txt` file at the project level.

## Compile and Flash the Project

Open the project in VSCode, open the integrated terminal, and type in

```bash
idf.py build flash monitor
```

Or click the small flame icon in the lower-left corner of VSCode to compile, flash, and open the monitor. At this point, you should see the LED light up on the development board and see the corresponding output in the monitor.
