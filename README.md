# 项目介绍

这是一个基于 UCloud 物联网的一个门禁系统 demo，主要面向对象还是有类似需求的学生，由于未经过验证，此项目不适合使用于实际产品中

本项目实现的功能简单的说就是，设备端可以通过各种传感器确认身份开门，远程端可以统计开门信息，也可以远程开门

本项目多少存在一些问题，但是我不想弄了，因为我不准备搞物联网这块，本项目宣告结束

# 目录介绍

| 文件夹   | 描述             |
| -------- | ---------------- |
| software | 存放软件相关源码 |
| hardware | 存放硬件相关资料 |

# 项目目标

提供一些与门禁控制相关的外设的软件包，这个项目主要是这些软件包的简单应用

计划提供的软件包

+ [x] [as608 指纹模块](https://github.com/greedyhao/as608)
+ [x] [rc522 nfc模块](https://github.com/greedyhao/rc522_rtt)

# 使用的软件包

+ [ucloud-iot](https://github.com/ucloud/ucloud-iot-rtthread-package)
+ [as608](https://github.com/greedyhao/as608)
+ [rc522](https://github.com/greedyhao/rc522_rtt)
+ [at_device](https://github.com/RT-Thread-packages/at_device)
+ [cJSON]()
+ [dht11](https://github.com/murphyzhao/dht11_rtt)
+ [mbedtls](https://github.com/RT-Thread-packages/mbedtls)

# 演示视频

[演示视频](https://www.bilibili.com/video/av78722650)

# 相关资料

+ [源码地址](https://github.com/greedyhao/entrance_guard)
+ [相关教程](https://zhuanlan.zhihu.com/c_1179710392883884032)
