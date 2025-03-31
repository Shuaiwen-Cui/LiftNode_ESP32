# FILE SYSTEM (SPIFFS)

## Introduction

We have introduced how to drive the SD card, but the operations on the SD card are all direct or write binary data, which is not user-friendly. So we need a file system to manage files, so that users can operate files by file name, rather than by address.

The file system we use is SPIFFS, which is a file system designed for SPI flash devices. It is a lightweight file system that is designed for microcontrollers with a small amount of RAM and ROM. It is designed to work with SPI NOR flash devices, which are the most common type of flash memory in microcontrollers.

SPIFFS has the following features:

- **Small Footprint:** Designed for limited RAM environments, avoiding the need for heap allocation.
- **Erase Constraints:** Only large blocks of data can be erased at once.
- **Erase Behavior:** Erasing resets all bits in a block to `1`.
- **Write Behavior:** Write operations flip bits from `1` to `0`.
- **Erase to Reset:** Bits set to `0` can only be reset to `1` through an erase operation.
- **Wear Leveling:** Implements wear leveling to ensure balanced usage of flash memory.

- **Optimized for Low RAM Usage:** Designed to use a static RAM buffer size, independent of the number of files.
- **POSIX-like Interface:** Supports operations such as open, close, read, write, seek, and stat, resembling a portable operating system interface.
- **Wide Compatibility:** Runs on any NOR flash memory, not just SPI flash, and can theoretically operate on embedded flash in microprocessors.
- **Multiple Configurations:** Allows multiple SPIFFS configurations to run on the same target, even on the same SPI flash device.
- **Static Wear Leveling:** Maintains the lifespan of flash memory through static wear leveling.
- **Filesystem Integrity Checks:** Includes a built-in file system consistency checker.
- **Highly Configurable:** Offers extensive customization options.

!!! note "VFS"
    VFS (Virtual File System) is a component of ESP-IDF that provides a unified interface for different file systems. This allows various file systems to be operated through a single API. SPIFFS is one such implementation under VFS.

## Use Case

1. Create a `hello.txt` file in the specified area of the NOR flash, then perform read and write operations on this file.
2. Blink an LED to indicate the program is running.

## Resources Involved

1. **LED**
   - Pin: IO0
2. **0.96-inch LCD**
3. **SPIFFS**

## SPIFFS API

### Register and Mount SPIFFS

This function registers SPIFFS to the VFS (Virtual File System) and mounts it with a given path prefix. The function prototype is as follows:

```c
esp_err_t esp_vfs_spiffs_register(const esp_vfs_spiffs_conf_t * conf);
```

Function Parameter Description

| Parameter | Description |
| :--- | :--- |
| `conf` | Pointer to the configuration structure of type `esp_vfs_spiffs_conf_t` |

Return Values

| Return Value | Description |
| :--- | :--- |
| `ESP_OK` | Returns `0`, configuration successful |
| `ESP_ERR_NO_MEM` | If objects cannot be allocated |
| `ESP_ERR_INVALID_STATE` | If already mounted or the partition is encrypted |
| `ESP_ERR_NOT_FOUND` | If the SPIFFS partition cannot be found |
| `ESP_FAIL` | If mounting or formatting fails |

Configuration Structure Definition

The function uses a structure variable of type `esp_vfs_spiffs_conf_t` as its parameter. The structure is defined as follows:

| Member Variable            | Description                                                                 |
| :--------------------- | :------------------------------------------------------------------ |
| `base_path`            | File path prefix associated with the file system.                                |
| `partition_label`      | Optional, the label of the SPIFFS partition to use. If set to `NULL`, the default partition is used. |
| `max_files`            | Maximum number of files that can be opened simultaneously.                       |
| `format_if_mount_failed` | If `true`, formats the file system if mounting fails.                          |

### Retrieve SPIFFS Information

This function is used to retrieve information about SPIFFS. Its function prototype is as follows:

```c
esp_err_t esp_spiffs_info(const char* partition_label,
                          size_t *total_bytes,
                          size_t *used_bytes);
```

The function parameters are described as follows:

| Parameter         | Description                                           |
| :---              | :---                                                 |
| `partition_label` | Label of the SPIFFS partition                        |
| `total_bytes`     | Pointer to a variable that stores the total byte count |
| `used_bytes`      | Pointer to a variable that stores the used byte count |

Return Values:

- `ESP_OK`: Success
- Other: Failure

### Unregister and Unmount SPIFFS

This function unregisters SPIFFS from the VFS and unmounts it. The function prototype is as follows:

```c
esp_err_t esp_vfs_spiffs_unregister(const char* partition_label);
```

The function parameters are described as follows:

| Parameter         | Description                                    |
| :---              | :---                                           |
| `partition_label` | Pointer to the partition table, partition name |

Return Values:

- `ESP_OK`: Success
- Others: Failure

<!-- ## Dependencies

![dep](dep-07-spi-sd.png) -->