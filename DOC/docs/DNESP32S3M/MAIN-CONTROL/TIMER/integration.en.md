# INTEGRATION

## Introduction

!!! note
    In this section, we introduce how to integrate the timer driver into the project.

## Create a New Component

!!! warning
    Before you create a component in the `driver` directory, make sure you have incorporated the `driver` directory into the project by adding the path to the `driver` directory to the `EXTRA_COMPONENT_DIRS` variable in the project-level `CMakeLists.txt` file.

Open the project in VSCode, open the integrated terminal, and type in

```bash
get_idf 
```
to actiate the ESP-IDF environment. Then type in the following command to create a new component named tim`:

```bash
idf.py -C driver create-component tim
```

The command above means to create a new component named tim` in the `driver` directory. After executing the command, you will see a new directory named tim` in the `driver` directory. And the command will automatically generate the `CMakeLists.txt` file for the new component, as well as the `tim.h` and `tim.c` files.

Alternatively, you can mannually create the tim` directory in the `driver` directory, and then create the `CMakeLists.txt`, `tim.h`, and `tim.c` files in the tim` directory.

## Replace the Code

> Component Level

Replace the code in this section into the `CMakelists.txt`, `tim.h`, and `tim.c` files under the component.

> Project Level

Replace the code in `main.c` with the corresponding code. No need to change the `CMakeLists.txt` file at the project level.

## Compile and Flash the Project

Open the project in VSCode, open the integrated terminal, and type in

```bash
idf.py build flash monitor
```

Now, you will see the LED blinks at the interval of 1s, which is set by the timer period.
