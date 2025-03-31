# 发布

## 代码仓库

<div class="grid cards" markdown>

-   :simple-github:{ .lg .middle } __NexNode__

    ---

    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/NexNode.git" target="_blank"> Portal </a>](#)

</div>

## 发布说明

| 分支 | 状态 | 初始化模板  | 主控制功能  | 外设功能  | 数学 + DSP + AI |
| :---: | :---: | :---: | :---: | :---: | :---: |
| DNESP32S3M-INIT | ✅ | ✔️ | | | |
| DNESP32S3M-CORE | ✅ | ✔️ | ✔️ | | |
| DNESP32S3M-IOT | ✅ | ✔️ | ✔️ | ✔️ | |
| DNESP32S3M-IOT-AI | ✅ | ✔️ | ✔️ | ✔️ | ✔️ |

> DNESP32S3M-INIT

初始化模板，无实际功能，可用于快速创建新的分支。

> DNESP32S3M-CORE

主控制功能，包括基本的控制功能，如串口、GPIO、定时器、中断等。不包含IOT、AI等高级功能。

> DNESP32S3M-IOT

DNESP32S3M-CORE + WIFI + IOT + Sensing

> DNESP32S3M-IOT-AI

DNESP32S3M-IOT + AI

!!! tip
    推荐使用DNESP32S3M-IOT和DNESP32S3M-IOT-AI分支进行进一步开发，后者功能覆盖前者。前者适用于不涉及复杂信号处理和AI的应用场景, 后者适用于需要复杂信号处理和AI的应用场景。