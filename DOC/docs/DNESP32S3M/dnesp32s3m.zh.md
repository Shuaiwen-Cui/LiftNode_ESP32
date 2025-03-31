# 正点原子 DNESP32S3M 开发板

## 硬件架构

!!! NOTE "硬件架构"
    主控 + 感知 + 通信 + 执行 + 电源
 
>正点原子 DNESP32S3M 主控板

![DNESP32S3M](DNESP32S3M.png){width=800px}

![DNESP32S3M-BACK](DNESP32S3M-BACK.png){width=800px}

## 代码架构

>一般意义上的 ESP32 项目结构

```txt
- myProject/
             - CMakeLists.txt
             - sdkconfig
             - components/ - component1/ - CMakeLists.txt
                                         - Kconfig
                                         - src1.c
                           - component2/ - CMakeLists.txt
                                         - Kconfig
                                         - src1.c
                                         - include/ - component2.h
             - main/       - CMakeLists.txt
                           - src1.c
                           - src2.c
             - build/

```

>DNESP32S3M 模板项目结构

```txt
- DNESP32S3M-XXX/
             - .vscode/ (vscode 设置文件，可以忽略)
             - CMakeLists.txt (项目级 cmake 文件)
             - sdkconfig (项目级 sdkconfig 文件，通过 idf.py menuconfig 生成)
             - sdkconfig.old (sdkconfig 备份)
             - partitions-16MiB.csv (16MiB flash 调整的分区表文件)
             - dependencies.lock (依赖库版本锁定文件)
             - application/         - component1/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                    - component2/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                                  - include/ - component2.h
             - middleware/          - component1/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                    - component2/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                                  - include/ - component2.h
             - driver/              - component1/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                    - component2/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                                  - include/ - component2.h
             - main/                - CMakeLists.txt
                                    - src1.c
                                    - src2.c
             - build/

```

!!! NOTE "DNESP32S3M 模板项目结构"
    - `application` 目录下放置应用层代码
    - `middleware` 目录下放置中间件代码
    - `driver` 目录下放置驱动代码
    - `main` 目录下放置主函数代码

>DNESP32S3M 项目实际结构 (实时更新)

```txt
- DNESP32S3M-XXX/
             - .vscode/ (vscode 设置文件，可以忽略)
             - CMakeLists.txt (项目级 cmake 文件)
             - sdkconfig (项目级 sdkconfig 文件，通过 idf.py menuconfig 生成)
             - sdkconfig.old (sdkconfig 备份)
             - partitions-16MiB.csv (16MiB flash 调整的分区表文件)
             - dependencies.lock (依赖库版本锁定文件)
             - application/         - component1/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                    - component2/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                                  - include/ - component2.h
             - middleware/          - component1/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                    - component2/ - CMakeLists.txt
                                                  - Kconfig
                                                  - src1.c
                                                  - include/ - component2.h
             - driver/              - esp_rtc/    - CMakeLists.txt
                                                  - include/ - esp_rtc.h
                                                  - esp_rtc.c
                                    - esp32_mqtt/ - CMakeLists.txt
                                                  - include/ - mqtt.h
                                                  - mqttc.c
                                    - exit/       - CMakeLists.txt
                                                  - include/ - exit.h
                                                  - exit.c
                                    - i2c/        - CMakeLists.txt
                                                  - include/ - i2c.h
                                                  - i2c.c
                                    - lcd/        - CMakeLists.txt
                                                  - include/ - lcd.h
                                                  - lcd.c
                                    - led/        - CMakeLists.txt
                                                  - include/ - led.h
                                                  - led.c
                                    - mpu6050/    - CMakeLists.txt
                                                  - include/ - mpu6050.h
                                                  - mpu6050.c
                                    - rng/        - CMakeLists.txt
                                                  - include/ - rng.h
                                                  - rng.c
                                    - spi/        - CMakeLists.txt
                                                  - include/ - spi.h
                                                  - spi.c
                                    - spi_sdcard/ - CMakeLists.txt
                                                  - include/ - spi_sdcard.h
                                                  - spi_sdcard.c
                                    - tim/        - CMakeLists.txt
                                                  - include/ - tim.h
                                                  - tim.c
                                    - wifi/       - CMakeLists.txt
                                                  - include/ - wifi.h
                                                  - wifi.c
             - main/                - CMakeLists.txt
                                    - main.c

             - build/
```



## 本项目开发顺序

!!! tip
    对于每个组件，需要保证其前置条件已经满足，才能进行后续的开发。

0. 项目构建
1. 执行>LED
2. 主控>外部中断
3. 主控>定时器
4. 主控>实时时钟
5. 主控>随机数发生器
6. 主控>SPI & 执行 LCD
7. 主控>SPI & SD 卡
8. 通信>WIFI
9. 通信>IOT
10. 主控>I2C
11. 感知>MPU6050