# 笔记

!!! note
    WiFi几乎是所有ESP32开发板的内建功能。它可以在"AP（接入点）"模式、"STA（站点）"模式下工作，或者同时在这两种模式下工作。在本项目中，我们主要使用ESP32作为站点连接路由器并访问互联网。

!!! tip
    正常的WiFi连接方式可以轻松在网上找到。在本项目中，我们将节点设置为AP模式，并连接到企业WiFi网络，如校园WiFi。具体来说，我们使用WPA2 Enterprise模式和PEAP-MSCHAPV2认证。

有关ESP32 WiFi功能的更多信息，请查看以下网站（我另一个ESP32学习项目）。

<div class="grid cards" markdown>

-   :material-file:{ .lg .middle } __ESP_STUDY__

    ---


    [:octicons-arrow-right-24: <a href="https://shuaiwen-cui.github.io/ESP_STUDY/zh/COMMUNICATION/WIFI/BASICS/wifi/" target="_blank"> 入口 </a>](#)

</div>


!!! tip
    如果您正在寻找家庭中的普通WiFi代码，您可以查看上面的链接。