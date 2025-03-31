# 项目构建

## 项目层级 CMakeLists.txt 文件

```cmake
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

set(EXTRA_COMPONENT_DIRS "./driver" "./middleware" "./application")

project(DNESP32S3-CORE)

```

!!! note
    为了与前面介绍的项目代码架构保持一致，我们建议您在项目根目录下创建 `driver`、`middleware` 和 `application` 目录，并将相应的代码放在相应的目录中。要将这些目录合并到项目中，您需要将这些目录的路径添加到项目级 `CMakeLists.txt` 文件中的 `EXTRA_COMPONENT_DIRS` 变量中。

!!! danger
    必须注意 `set(EXTRA_COMPONENT_DIRS "./driver" "./middleware" "./application")` 必须放在 `project(DNESP32S3-CORE)` 之前。否则，项目将无法在这些目录中找到组件。参考 [LINK](https://blog.51cto.com/u_16099361/12129084)。

## 主目录 CMakeLists.txt

```cmake
# Define source directories
set(src_dirs
    .
)

# Define include directories
set(include_dirs
    .
)

# Define required components
set(requires
)

# Register the component
idf_component_register(
    SRC_DIRS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
)

# Add compilation options
# component_compile_options(-ffast-math -O3 -Wno-error=format -Wno-format)
```