# ESP32 开发环境设置

!!! tip
    在继续之前，强烈建议您阅读官方指南以设置开发环境。官方指南可以在[这里](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html)找到。

!!! note
    如需在 ESP32 上使用 ESP-IDF，请安装以下软件：

    - 设置工具链，用于编译 ESP32 代码；

    - 编译构建工具 —— CMake和Ninja编译构建工具，用于编译ESP32应用程序；

    - 获取ESP-IDF软件开发框架。该框架已经基本包含ESP32使用的API（软件库和源代码）和运行工具链的脚本；

![what you need](what-you-need.png)

!!! note
    有两种 ESP32 ESP-IDF 编程方式：

    - 命令行界面（CLI）：ESP-IDF 编程的基础；

    - 集成开发环境（IDE）：适合初学者。
  
## WINDOWS

### CLI

<div class="grid cards" markdown>

-   :fontawesome-brands-youtube:{ .lg .middle } __How to Install the ESP-IDF Toolchain on Windows (CLI)__

    ---

    [:octicons-arrow-right-24: <a href="https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/get-started/windows-setup.html" target="_blank"> 传送门 </a>](#)

</div>

### IDE

<div class="grid cards" markdown>

-   :fontawesome-brands-bilibili:{ .lg .middle } __【乐鑫教程】｜使用一键安装工具快速搭建 ESP-IDF 开发环境 (Windows) 🎯🏆✅__

    ---

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1to4y177ko/?spm_id_from=333.999.0.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> 传送门 </a>](#)

</div>

## LINUX

!!! tip
    ESP-IDF 在 Linux 上比在 Windows 上快得多。建议使用 Linux 进行 ESP-IDF 开发。

### CLI 🎯

!!! note
    一共有五个步骤：

    - 第一步：安装准备

    - 第二步：获取ESP-IDF

    - 第三步：设置工具

    - 第四步：设置环境变量

    - 第五步：开始使用

#### 第一步 - 安装准备

- UBUNTU & DEBIAN

```shell
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

- CENTOS

```shell
sudo yum -y update && sudo yum install git wget flex bison gperf python3 python3-setuptools cmake ninja-build ccache dfu-util libusbx
```

#### 第二步 - 获取 ESP-IDF

在围绕 ESP32 构建应用程序之前，请先获取乐鑫提供的软件库文件 ESP-IDF 仓库。

获取 ESP-IDF 的本地副本：打开终端，切换到要保存 ESP-IDF 的工作目录，使用 git clone 命令克隆远程仓库。针对不同操作系统的详细步骤，请见下文。

打开终端，运行以下命令：

```shell
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
```
ESP-IDF 将下载至 `~/esp/esp-idf`。

请前往[ESP-IDF 版本简介](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/versions.html)，查看 ESP-IDF 不同版本的具体适用场景。

#### 第三步 - 设置工具

除了 ESP-IDF 本身，还需要为支持 ESP32 的项目安装 ESP-IDF 使用的各种工具，比如编译器、调试器、Python 包等。

```shell
cd ~/esp/esp-idf
./install.sh all
```

#### 第四步 - 设置环境变量

此时，刚刚安装的工具尚未添加至 PATH 环境变量，无法通过“命令窗口”使用这些工具。因此，必须设置一些环境变量。这可以通过 ESP-IDF 提供的另一个脚本进行设置。

请在需要运行 ESP-IDF 的终端窗口运行以下命令：

```shell
. $HOME/esp/esp-idf/export.sh
```

如果需要经常运行 ESP-IDF，可以为执行 export.sh 创建一个别名，具体步骤如下：

(1) 打开 shell 配置文件

```shell
nano ~/.bashrc
```

(2) 复制并粘贴以下命令到 shell 配置文件中

```shell
alias get_idf='. $HOME/esp/esp-idf/export.sh'
```

(3) 通过重启终端窗口或运行 `source [path to profile]`，如 `source ~/.bashrc` 来刷新配置文件。

现在可以在任何终端窗口中运行 `get_idf` 来设置或刷新 ESP-IDF 环境。

不建议直接将 `export.sh` 添加到 shell 的配置文件。这样做会导致在每个终端会话中都激活 IDF 虚拟环境（包括无需使用 ESP-IDF 的会话）。这违背了使用虚拟环境的目的，还可能影响其他软件的使用。

#### 第五步 - 开始使用

请参考其他教程，开始使用 ESP-IDF。





### IDE

<div class="grid cards" markdown>

-   :fontawesome-brands-bilibili:{ .lg .middle } __【乐鑫教程】｜使用 VS Code 快速搭建 ESP-IDF 开发环境 (Windows、Linux、MacOS) 🎯🏆✅__

    ---

    Recommended.

    [:octicons-arrow-right-24: <a href="https://www.bilibili.com/video/BV1V24y1T75n/?spm_id_from=333.999.0.0&vd_source=5a427660f0337fedc22d4803661d493f" target="_blank"> 传送门 </a>](#)

</div>

## 卸载ESP-IDF

(1) 删除 ESP-IDF 文件夹

首先，找到安装 ESP-IDF 的目录（通常在 `~/esp/esp-idf`）。你可以使用以下命令删除该目录：

```shell
rm -rf ~/esp/esp-idf
```

(2) 删除工具链

ESP-IDF 的 install.sh 脚本会下载并安装一些工具链和依赖，通常位于 `~/.espressif` 目录。使用以下命令删除该目录：

```shell
rm -rf ~/.espressif
```

(3) 删除环境变量
如果你在 shell 配置文件（例如 `~/.bashrc` 或 `~/.zshrc`）中添加了用于快速加载 ESP-IDF 环境变量的别名（如 `get_idf`），则需要编辑这些配置文件并删除该别名。可以使用如下命令打开并编辑文件

```shell
nano ~/.bashrc
```

(4) 刷新配置文件

最后，运行以下命令以使更改生效：

```shell
source ~/.bashrc
```
## VSCode ESP-IDF插件
!!! note
    这是当前最流行的ESP-IDF开发方式之一。VSCode是一个轻量级的代码编辑器，支持多种编程语言，包括C/C++。ESP-IDF插件为VSCode提供了对ESP-IDF的完整支持，包括代码自动完成、调试、构建和烧录等功能。

1. 安装VSCode

    首先，你需要安装VSCode。你可以从[这里](https://code.visualstudio.com/)下载适用于你的操作系统的安装程序。

2. 安装ESP-IDF插件

    安装ESP-IDF插件的步骤如下：

    - 打开VSCode。
    - 点击左侧的扩展图标。
    - 在搜索框中输入“ESP-IDF”。
    - 点击“安装”按钮。

3. 配置ESP-IDF路径 和 ESP-IDF tools路径

    安装ESP-IDF插件后，你需要配置ESP-IDF路径和ESP-IDF tools路径。这些路径告诉插件在哪里找到ESP-IDF和相关工具。

    - 打开VSCode。
    - 点击左下角的“设置”图标。
    - 在搜索框中输入“ESP-IDF”。
    - 在“ESP-IDF路径”和“ESP-IDF tools路径”字段中输入ESP-IDF和ESP-IDF tools的路径。（如果电脑上没有，设置好路径，插件会自动下载）
    - 保存设置。

4. 创建新项目