# RELEASE

## CODE REPOSITORY

<div class="grid cards" markdown>

-   :simple-github:{ .lg .middle } __NexNode__

    ---

    [:octicons-arrow-right-24: <a href="https://github.com/Shuaiwen-Cui/NexNode.git" target="_blank"> Portal </a>](#)

</div>

## RELEASE NOTE

| BRANCH | STATUS | INIT TEMPLATE  | MAIN CONTROL FUNCTIONS  | PERIPHERAL FUNCTIONS  | MATH + DSP + AI  |
| :---: | :---: | :---: | :---: | :---: | :---: |
| DNESP32S3M-INIT | ✅ | ✔️ | | | |
| DNESP32S3M-CORE | ✅ | ✔️ | ✔️ | | |
| DNESP32S3M-IOT | ✅ | ✔️ | ✔️ | ✔️ | |
| DNESP32S3M-IOT-AI | ✅ | ✔️ | ✔️ | ✔️ | ✔️ |

> DNESP32S3M-INIT

Initialization template for rapid creation of new branches. Contains minimal configuration.

> DNESP32S3M-CORE

Core functionality module, providing essential controls such as UART, GPIO, timers, and interrupts. Does not include advanced features like IoT or AI.

> DNESP32S3M-IOT

IoT functionality extension module built upon DNESP32S3M-CORE, adding IoT-related capabilities. Does not include AI features.

> DNESP32S3M-IOT-AI

DNESP32S3M-IOT + AI

!!! tip
    It is recommended to use the DNESP32S3M-IOT and DNESP32S3M-IOT-AI branches for further development, as the latter covers the former. The former is suitable for applications that do not involve complex signal processing and AI, while the latter is suitable for applications that require complex signal processing and AI.
