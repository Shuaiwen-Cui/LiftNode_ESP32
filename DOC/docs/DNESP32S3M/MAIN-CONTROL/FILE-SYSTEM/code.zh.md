# 文件系统（SPIFFS）代码

## 组件架构

```plaintext
- driver
    - spiffs
        - include
            - spiffs.h
        - spiffs.c
        - CMakeLists.txt
```

## driver/rng/CMakeLists.txt

```cmake
set(src_dirs
    .
)

set(include_dirs
    include
)

set(requires
    driver
    fatfs
    vfs
    spi
)

idf_component_register(SRC_DIRS ${src_dirs} INCLUDE_DIRS ${include_dirs} REQUIRES ${requires})
```
