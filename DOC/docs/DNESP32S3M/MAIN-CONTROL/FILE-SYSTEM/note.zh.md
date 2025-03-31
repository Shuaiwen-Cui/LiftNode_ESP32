# 文件系统 (SPIFFS)

## 介绍

我们已经介绍过如何驱动SD卡，但是对SD卡的操作都是直接或者写入二进制数据，这样的操作对于用户来说是不友好的。所以我们需要一个文件系统来管理文件，这样用户可以通过文件名来操作文件，而不是通过地址。

SPIFFS 是一个用于嵌入式目标上的 SPI NOR flash 设备的文件系统，并且有如下特点：

- 小目标，没有堆的少量 RAM
- 只有大范围的数据块才能被擦除
- 擦除会将块中的所有位重置为 1
- 写操作将 1 置 0
- 0 只能被擦除成 1
- 磨损均衡
- 以上几点是 SPIFFS 的特点，下面则说明了 SPIFFS 具体能做些什么：
- 专门为低 ram 使用而设计
- 使用静态大小的 ram 缓冲区，与文件的数量无关
- 类可移植操作系统接口:打开、关闭、读、写、查找、统计等
- 它可以在任何 NOR 闪存上运行，不仅是 SPI 闪存，理论上也可以在微处理器的嵌入式
闪存上运行
- 多个 spiffs 配置可以在相同的目标上运行—甚至可以在相同的 SPI 闪存设备上运行
- 实现静态磨损调平（也就是 flash 的寿命维护）
- 内置文件系统一致性检查
- 高度可配置的

!!! note "VFS"
    VFS是ESP-IDF的一个组件，它是一个虚拟文件系统，它可以将不同的文件系统统一起来，这样我们就可以通过一个接口来操作不同的文件系统。SPIFFS就是VFS的一个实现。

## 用例

1.在 nor flash 指定区域新建 hello.txt 文件，然后对这文件进行读写操作
2.LED 闪烁，指示程序正在运行

## 涉及资源

1.LED 灯
LED -IO0
2.0.96 寸 LCD
3.SPIFFS

## SPIFFS API

### 注册装载SPIFFS

该函数使用给定的路径前缀将SPIFFS注册并装载到VFS，其函数原型如下：

```c
esp_err_t esp_vfs_spiffs_register(const esp_vfs_spiffs_conf_t * conf);
```

该函数形参描述如下：

| 参数 | 描述 |
| :--- | :--- |
| conf | 指向 esp_vfs_spiffs_conf_t 配置结构的指针 |

返回值：

| 返回值 | 描述 |
| :--- | :--- |
| ESP_OK | 返回：0，配置成功 |
| ESP_ERR_NO_MEM | 如果无法分配对象 |
| ESP_ERR_INVALID_STATE | 如果已安装或分区已加密 |
| ESP_ERR_NOT_FOUND | 如果找不到 SPIFFS 的分区 |
| ESP_FAIL | 如果装载或格式化失败 |

该函数使用 esp_vfs_spiffs_conf_t 类型的结构体变量传入，该结构体的定义如下所示：

| 成员变量                | 说明                                                                 |
| :--------------------- | :------------------------------------------------------------------ |
| `base_path`            | 与文件系统关联的文件路径前缀。                                       |
| `partition_label`      | 可选，要使用的SPIFFS分区的标签。如果设置为`NULL`，则使用默认值。      |
| `max_files`            | 可以同时打开的最大文件数。                                           |
| `format_if_mount_failed` | 如果为`true`，则在挂载失败时将格式化文件系统。                     |

### 获取SPIFFS信息

该函数用于获取 SPIFFS 的信息，其函数原型如下所示：

```c
esp_err_t esp_spiffs_info(const char* partition_label,
                          size_t *total_bytes,
                          size_t *used_bytes);
```

该函数形参描述如下：

| 参数 | 描述 |
| :--- | :--- |
| partition_label | SPIFFS 分区的标签 |
| total_bytes | 指向存储总字节数的变量的指针 |
| used_bytes | 指向存储已使用字节数的变量的指钩 |

返回值：

- ESP_OK：成功
- 其他：失败

### 注销和卸载 SPIFFS

该函数从 VFS 注销和卸载 SPIFFS，其函数原型如下所示：

```c
esp_err_t esp_vfs_spiffs_unregister(const char* partition_label);
```

该函数形参描述如下：

| 参数 | 描述 |
| :--- | :--- |
| partition_label | 指向分区表的指针，分区表名称 |

返回值：

- ESP_OK：成功
- 其他：失败


<!-- ## 依赖关系

![dep](dep-07-spi-sd.png) -->