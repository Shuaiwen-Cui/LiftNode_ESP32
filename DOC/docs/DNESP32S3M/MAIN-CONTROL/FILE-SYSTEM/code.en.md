# FILE SYSTEM (SPIFFS) CODE

## Component Architecture

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

## spiffs.h
    
```c

```

## spiffs.c

```c

```

## spiffs.c

```c

```

