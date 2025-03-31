# 集成

## 介绍

!!! note
    在本节中，我们介绍如何将定时器驱动集成到项目中。

## 创建新组件

!!! warning
    在 `driver` 目录中创建组件之前，请确保您已将 `driver` 目录合并到项目中，方法是将 `driver` 目录的路径添加到项目级 `CMakeLists.txt` 文件中的 `EXTRA_COMPONENT_DIRS` 变量中。

在 VSCode 中打开项目，打开集成终端，输入

```bash
get_idf 
```

激活 ESP-IDF 环境。然后输入以下命令创建一个名为 `tim` 的新组件：

```bash
idf.py -C driver create-component tim
```

上面的命令意味着在 `driver` 目录中创建一个名为 `tim` 的新组件。执行命令后，您将在 `driver` 目录中看到一个名为 `tim` 的新目录。命令将自动生成新组件的 `CMakeLists.txt` 文件，以及 `tim.h` 和 `tim.c` 文件。

或者，您可以在 `driver` 目录中手动创建 `tim` 目录，然后在 `tim` 目录中创建 `CMakeLists.txt`、`tim.h` 和 `tim.c` 文件。

## 替换代码

> 组件层面

将本节中代码分别替换到组件下的`CMakelists.txt`、`tim.h`和`tim.c`文件中。

> 项目层面

将`main.c`中的代码替换为相应的代码。项目层面的`CMakeLists.txt`文件无需更改。

## 编译烧录

在 VSCode 中打开项目，打开集成终端，输入

```bash
idf.py build flash monitor
```
此时，您将看到 LED 每隔 1s 闪烁一次，这是由定时器周期设置的。
