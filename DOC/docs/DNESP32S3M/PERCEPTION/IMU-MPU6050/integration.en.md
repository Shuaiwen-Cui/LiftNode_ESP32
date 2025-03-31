# INTEGRATION

## Introduction

!!! note
    In this section, we introduce how to integrate the mpu6050 driver into the project.

## Create a New Component

!!! warning
    Before you create a component in the `driver` directory, make sure you have incorporated the `driver` directory into the project by adding the path to the `driver` directory to the `EXTRA_COMPONENT_DIRS` variable in the project-level `CMakeLists.txt` file.

Open the project in VSCode, open the integrated terminal, and type in

```bash
get_idf 
```
to actiate the ESP-IDF environment. Then type in the following command to create a new component named `mpu6050`:

```bash
idf.py -C driver create-component mpu6050
```

The command above means to create a new component named `mpu6050` in the `driver` directory. After executing the command, you will see a new directory named `mpu6050` in the `driver` directory. And the command will automatically generate the `CMakeLists.txt` file for the new component, as well as the `mpu6050.h` and `mpu6050.c` files.

Alternatively, you can mannually create the `mpu6050` directory in the `driver` directory, and then create the `CMakeLists.txt`, `mpu6050.h`, and `mpu6050.c` files in the `mpu6050` directory.

## Replace the Code

> Component Level

Replace the code in this section into the `CMakelists.txt`, `mpu6050.h`, and `mpu6050.c` files under the component.

> Project Level

Replace the code in `main.c` with the corresponding code. No need to change the `CMakeLists.txt` file at the project level.

## Compile and Flash the Project

Open the project in VSCode, open the integrated terminal, and type in

```bash
idf.py build flash monitor
```

Then you can see the corresponding component output information.

