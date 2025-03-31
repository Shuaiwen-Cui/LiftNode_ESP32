# RTC (Real Time Clock)

## Introduction

!!! note
    This chapter introduces the use of the ESP32-S3 real-time clock (RTC). The real-time clock can provide an accurate time for the system. Even if the system is reset or the main power is cut off, the RTC can still run (with battery)）. Therefore, the RTC is often used in various low-power scenarios.

!!! note "Real Time Clock"
    RTC (Real Time Clock) refers to a clock installed on an electronic device or an IC (integrated circuit) that implements its functions.

    When you call it a "clock" in digital circuits, you may think of a periodic signal, but in English, "clock" also means "clock". So why do we need a separate RTC?

    The reason is that the CPU's timer clock function only runs when it is "started" or "powered on" and stops when it is powered off. Of course, if the clock cannot continuously track time, you must manually set the time.

    Usually, the RTC is equipped with a separate power source, such as a button battery (backup battery). Even if the power of the DNESP32S3M minimum system board is turned off, it can still operate and display the time in real-time. Then, when the DNESP32S3M minimum system board is turned on again, the built-in timer clock of the computer reads the current time from the RTC and displays the time based on its own mechanism while being powered. By the way, since the button battery is relatively cheap and has a long service life, the RTC can operate at a very low cost. Based on this function, it can also be used as memory.

!!! note
    In the ESP32-S3, there is no RTC peripheral like the STM32 chip, but there is a system time. By using the system time, the function of a real-time clock can also be realized.
    The ESP32-S3 uses two hardware clock sources to establish and maintain system time. Depending on the application purpose and the accuracy requirements of the system time, you can use only one of the clock sources or both clock sources at the same time. These two hardware clock sources are the **RTC timer** and the **high-resolution timer**. By default, both timers are used. We will introduce them one by one below.

## Use Cases

1. Display the RTC time in real-time through the LCD / UART
2. The LED blinks to indicate that the program is running


## Dependencies

There is no dependency for this chapter.

## Key Functions

| Function Prototype | Explanation | Example |
| --- | --- | --- |
| void rtc_set_time(int year, int mon, int mday, int hour, int min, int sec);  | Set RTC time | rtc_set_time(2022, 12, 31, 23, 59, 59); |
| void rtc_get_time(void); /* Get time */ | Get RTC time, store it in the global variable calendar | rtc_get_time(); |



