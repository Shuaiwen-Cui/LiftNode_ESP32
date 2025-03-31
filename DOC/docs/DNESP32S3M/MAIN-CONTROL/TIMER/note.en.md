# TIMER

## Introduction

This document introduces the use of the ESP32 timer.

!!! info "Timer"
    The timer is an integrated function of the microcontroller, which can be flexibly controlled by programming. The timing function of the microcontroller depends on the internal counter. Whenever the microcontroller experiences a machine cycle and generates a pulse, the counter will increase. The main function of the timer is to time. When the set time is reached, it will trigger an interrupt to notify the system that the timing is complete. In the interrupt service function, we can write specific programs to achieve the desired function.

!!! info "What can a timer do"
    - Execute scheduled tasks: Timers are often used to periodically execute specific tasks. For example, if a task needs to be executed every 500 milliseconds, the timer can accurately meet this requirement.

    - Time measurement: The timer can accurately measure time, whether it is the execution time of a code segment or the interval time of an event, it can be accurately measured by the timer.

    - Accurate delay: For delay scenarios that require microsecond accuracy, the timer can provide a reliable   solution to ensure the accuracy of the delay.

    - PWN signal generation: Through the precise control of the timer, we can generate PWM (pulse width modulation) signals, which is crucial for applications such as driving motors and adjusting LED brightness.

    - Event triggering and monitoring: The timer is not only used to trigger interrupts and implement event-driven logic, but also to implement watchdog functions, monitor system status, and perform reset operations when necessary to ensure stable operation of the system.

!!! info "Types of Timers"
    Timers can be implemented either in hardware or software, each with its unique characteristics suited for different scenarios:

    **Hardware Timers** rely on the built-in hardware mechanisms of microcontrollers, utilizing dedicated timing/counter circuits to achieve timing functionality. Their key advantages are high precision and reliability because hardware timers operate independently of software tasks and operating system scheduling, remaining unaffected by them. In scenarios demanding extremely high timing accuracy, such as generating PWM signals or performing precise time measurements, hardware timers are undoubtedly the best choice. Their design ensures that even if the main CPU is occupied with other tasks, the timer can still accurately trigger the designated operation at the preset time.

    **Software Timers**, on the other hand, simulate timing functionality through the operating system or software libraries. The performance of these timers is affected by the system's current load and task scheduling strategies, resulting in slightly lower precision compared to hardware timers. However, software timers excel in flexibility, allowing the creation of numerous timers, making them ideal for scenarios where strict timing control is not required. It is important to note that software timers may encounter timing accuracy issues, especially under heavy system loads or when there are many high-priority tasks. Nevertheless, for simple delays where high precision is not critical, software timers are typically sufficient to meet the requirements.

## ESP32-S3 Timer Overview

![ESPTIMER](ESPTIMER.png)

The system timer has two built-in counters UNIT0 and UNIT1 (as shown in the figure) and three comparators COMP0, COMP1, and COMP2 (as shown in the figure). The comparators are used to monitor whether the counter's count value reaches the alarm value.

(1) Counter

UNIT0 and UNIT1 are 52-bit counters built into the ESP32-S3 system timer. The counters use XTAL_CLK as the clock source (40MHz). After XTAL_CLK is divided, a clock signal with a frequency of fXTAL_CLK/3 is generated in one counting cycle, and then a clock signal with a frequency of fXTAL_CLK/2 is generated in another counting cycle. Therefore, the clock CNT_CLK used by the counter has an actual average frequency of fXTAL_CLK/2.5, which is 16MHz, as shown in Figure 14.1.2. For each CNT_CLK clock cycle, the count increases by 1/16µs, that is, 16 cycles increase by 1µs. Users can control the counter UNITn by configuring the following three bits in register SYSTIMER_CONF_REG. These three bits are:
①: SYSTIMER_TIMER_UNITn_WORK_EN
②: SYSTIMER_TIMER_UNITn_CORE0_STALL_EN
③: SYSTIMER_TIMER_UNITn_CORE1_STALL_EN
For the configuration of these three bits, please refer to the "esp32-s3_technical_reference_manual_en".

(2) Comparator

COMP0, COMP1, and COMP2 are 52-bit comparators built into the ESP32-S3 system timer. The comparators also use XTAL_CLK as the clock source (40MHz).

![ALARM](ALARM.png)

The above figure shows the process of the system timer generating an alarm. In the above process, a counter (Timer Countern) and a comparator (Timer Comparatorx) are used to generate an alarm interrupt based on the comparison result.

## Use Case

Let's design a program that configures a high-resolution timer to enable interrupts and toggle the LED status in the interrupt callback function. Since we are using on-board resources, no wiring is required.

## Dependencies

![dep](dep-03-timer.png)

## Key Functions

| Function Prototype | Explanation | Example |
| --- | --- | --- |
| void esptim_int_init(uint64_t tps); | Initialize a high-precision timer (ESP_TIMER) | exit_init(); |
| void esptim_callback(void *arg); | Timer callback function | exit_callback(); |
| esp_err_t esp_timer_create(const esp_timer_create_args_t* args, esp_timer_handle_t* out_handle) | Create a new timer | esp_timer_create(&tim_periodic_arg, &esp_tim_handle); |
| esp_err_t IRAM_ATTR esp_timer_start_periodic(esp_timer_handle_t timer, uint64_t period_us) | Start a periodic timer | esp_timer_start_periodic(esp_tim_handle, tps); |

## Summary

!!! note

    This chapter introduces the use of ESP32 timers. By designing a program, the configuration of high-resolution timers is realized, interrupts are enabled, and the state of LEDs is flipped in the interrupt callback function. Several key points:

    - How to add timer files to the project, source files, header files, path dependencies, etc.

    - How to construct the timer initialization function.

    - How to construct the timer callback function.

    - How to call the timer initialization function at the application layer.