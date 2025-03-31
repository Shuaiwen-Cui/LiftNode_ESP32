# 上手实践

## 程序准备

这里我们找一个合适的路径，下载一些案例程序：

```shell
git clone --recursive https://gitee.com/vi-iot/esp32-board.git
```

## 硬件准备

将ESP32开发板与电脑连接

## 选定案例程序

进入我们刚刚下载的仓库esp32-board，找到一个案例程序，比如`helloworld`，进入该目录：

```shell
cd esp32-board
cd helloworld
```

## 设定开发板类型

!!! tip "ESP32环境问题"
    如果你当前不在ESP32环境中，可以输入`get_idf`来调用ESP32环境。注意，只有正确设置了SHELL中的别名后，才能使用`get_idf`, 请参考上一篇教程。

如果是一般的esp32开发板，这一步其实可以不用管，但是如果是其他开发板，最好指定一下开发板类型，比如esp32-s3：

```shell
idf.py set-target esp32s3
```

## 编译程序

```shell
idf.py build
```

## 烧录程序

```shell
idf.py flash
```
注意，可以查看设备所在usb端口：

```shell
ls /dev/tty*
```

通过链接/非链接对比，找到设备所在端口，比如`/dev/ttyACM1`，然后烧录程序：

```shell
idf.py -p /dev/ttyACM1 flash
```

## 监测程序运行

```shell
idf.py monitor
```

也可以指定端口：

```shell
idf.py -p /dev/ttyACM1 monitor
```

## 退出监测

按下`Ctrl` + `]`即可退出监测。