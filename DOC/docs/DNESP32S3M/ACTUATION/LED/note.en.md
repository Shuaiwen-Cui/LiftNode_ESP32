# LED NOTES

## Introduction

!!! note
    In this section, we will talk about the onboard LED light on the DNESP32S3M development board. By modify the GPIO pin, the code can be used for other boards as well.

## The LED

![LED](LED.png){ width=800px }

## The Circuit Onboard

![LED_CIRCUIT](LED_CIRCUIT.png){ width=800px }

As can be seen, the GPIO to control the LED is IO1.

## The Effect

![LED-RED](LED-RED.png){ width=800px }

## Dependencies

![dep](dep-01-led.png)

## Key Functions

| Function Prototype | Explanation | Example |
| --- | --- | --- |
| `void led_init(void)` | Initialize the LED | `led_init();` |
| `void led(int x)` | Control the LED | `led(1);` |
| `void led_toggle(void)` | Toggle the LED | `led_toggle();` |