# ESP32 DEVELOPMENT ENVIRONMENT SETUP

!!! tip
    You are strongly recommended to read the official guide for setting up the development environment before proceeding. The official guide can be found [here](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/index.html). 

!!! note
    To start using ESP-IDF on ESP32, install the following software:
    
    - Toolchain to compile code for ESP32
    
    - Build tools - CMake and Ninja to build a full Application for ESP32
    
    - ESP-IDF that essentially contains API (software libraries and source code) for ESP32 and scripts to  operate the Toolchain

![what you need](what-you-need.png)

!!! note
    There are two ways for ESP32 ESP-IDF programming:

    - Command Line Interface (CLI): fundamental for ESP-IDF programming

    - IDE: easier for beginners

## WINDOWS

### CLI

<div class="grid cards" markdown>

-   :fontawesome-brands-youtube:{ .lg .middle } __How to Install the ESP-IDF Toolchain on Windows (CLI)__

    ---

    [:octicons-arrow-right-24: <a href="https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/get-started/windows-setup.html" target="_blank"> Portal </a>](#)

</div>

### IDE

<div class="grid cards" markdown>

-   :fontawesome-brands-youtube:{ .lg .middle } __How to Install the ESP-IDF Toolchain on Windows (VSCode)__

    ---

    [:octicons-arrow-right-24: <a href="https://www.youtube.com/watch?v=byVPAfodTyY&list=PLOzvoM7_Knrc9MKih2gNIMwZhGBfIKQb7" target="_blank"> Portal </a>](#)

</div>

## LINUX

!!! tip
    ESP-IDF is much faster on Linux than on Windows. It is recommended to use Linux for ESP-IDF development.

### CLI 🎯

!!! note
    There are a total of five steps:

    - Step 1: Installation Preparation

    - Step 2: Get ESP-IDF

    - Step 3: Set up the Toolchain

    - Step 4: Set up the Build System

    - Step 5: Start Using

#### Step 1 - Installation Preparation
- UBUNTU & DEBIAN

```shell
sudo apt-get install git wget flex bison gperf python3 python3-pip python3-venv cmake ninja-build ccache libffi-dev libssl-dev dfu-util libusb-1.0-0
```

- CENTOS

```shell
sudo yum -y update && sudo yum install git wget flex bison gperf python3 python3-setuptools cmake ninja-build ccache dfu-util libusbx
```

#### Step 2 - Get ESP-IDF

Before building applications around the ESP32, obtain the ESP-IDF repository, the software library files provided by Espressif.

To get a local copy of ESP-IDF, open the terminal, navigate to the working directory where you want to save ESP-IDF, and clone the remote repository using git clone. See detailed steps for different operating systems below.

Open the terminal and run the following commands:

```shell
mkdir -p ~/esp
cd ~/esp
git clone --recursive https://github.com/espressif/esp-idf.git
```
ESP-IDF will be downloaded to `~/esp/esp-idf`.

Visit the [ESP-IDF Version Overview](https://docs.espressif.com/projects/esp-idf/zh_CN/latest/esp32s3/versions.html) to check specific applicable scenarios for different ESP-IDF versions.

#### Step 3 - Set Up the Toolchain

Besides ESP-IDF itself, various tools used by ESP-IDF, such as compilers, debuggers, and Python packages, must also be installed to support ESP32 projects.
```shell
cd ~/esp/esp-idf
./install.sh all
```
#### Step 4 - Set Up the Environment Variables

At this stage, the tools you just installed are not yet added to the PATH environment variable, so they cannot be used from the command line. Therefore, some environment variables must be set. This can be done using another script provided by ESP-IDF.

Run the following command in the terminal where you need to use ESP-IDF:

```shell
. $HOME/esp/esp-idf/export.sh
```

If you frequently use ESP-IDF, you can create an alias for running export.sh as follows:

(1) Open the shell configuration file

```shell
nano ~/.bashrc
```

(2) Copy and paste the following command into your shell configuration file.


```shell
alias get_idf='. $HOME/esp/esp-idf/export.sh'
```

(3) Refresh the configuration file by restarting the terminal or by running `source [path to profile]`, such as `source ~/.bashrc`.

You can now run `get_idf` in any terminal window to set up or refresh the ESP-IDF environment.

It is not recommended to add `export.sh` directly to the shell configuration file, as this will activate the IDF virtual environment in every terminal session, including those where ESP-IDF is not needed. This goes against the purpose of using a virtual environment and may interfere with other software.

### IDE

<div class="grid cards" markdown>

-   :fontawesome-brands-youtube:{ .lg .middle } __How to Install the ESP-IDF Toolchain on Ubuntu 🎯🏆✅__

    ---

    [:octicons-arrow-right-24: <a href="https://www.youtube.com/watch?v=Hj3okDU-CIY&list=PLOzvoM7_Knrc9MKih2gNIMwZhGBfIKQb7&index=4" target="_blank"> Portal </a>](#)

</div>

## Uninstalling ESP-IDF

(1) Delete the `esp` directory.

First, navigate to the directory where you cloned the ESP-IDF repository and delete the `esp` directory, typically at `~/esp/esp-idf`.

```shell
rm -rf ~/esp/esp-idf
```

(2) Remove the toolchain.

If you installed the toolchain using the `install.sh` script, typically in `~/.espressif`, you can remove it by running the following command:

```shell
rm -rf ~/.espressif
```

(3) Remove the environment variables.

If you added an alias to quickly load the ESP-IDF environment variables in your shell configuration file (e.g., `~/.bashrc` or `~/.zshrc`), you need to edit these configuration files and remove the alias. You can open and edit the file using the following command:

```shell
nano ~/.bashrc
```

(4) Refresh the configuration file.

After removing the alias, refresh the configuration file by running the following command:

```shell
source ~/.bashrc
```

## VSCode ESP-IDF Plugin
!!! note
    This is currently one of the most popular ways to develop with ESP-IDF. VSCode is a lightweight code editor that supports multiple programming languages, including C/C++. The ESP-IDF plugin provides full support for ESP-IDF in VSCode, including features like code autocompletion, debugging, building, and flashing.

1. Install VSCode

    First, you need to install VSCode. You can download the installer for your operating system [here](https://code.visualstudio.com/).

2. Install the ESP-IDF Plugin

    To install the ESP-IDF plugin, follow these steps:

    - Open VSCode.
    - Click the Extensions icon on the left sidebar.
    - Type “ESP-IDF” in the search bar.
    - Click the “Install” button.

3. Configure ESP-IDF Path and ESP-IDF Tools Path

    After installing the ESP-IDF plugin, you need to configure the ESP-IDF path and ESP-IDF tools path. These paths tell the plugin where to find ESP-IDF and its related tools.

    - Open VSCode.
    - Click the “Settings” icon in the lower left corner.
    - Type “ESP-IDF” in the search bar.
    - Enter the paths for ESP-IDF and ESP-IDF tools in the “ESP-IDF Path” and “ESP-IDF Tools Path” fields. (If they are not already installed, the plugin will automatically download them once the paths are set.)
    - Save the settings.

4. Create a New Project
