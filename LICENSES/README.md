# 第三方许可说明

本仓库包含三个来源的代码，许可各不相同。

## 1. `User\`（应用层）与本工程配置

`main.c`、`zf_isr.c`、`ch32v30x_it.c`、`system_ch32v30x.c`、`ch32v30x_conf.h`、
以及 `.cproject` / `.project` / `Ld\Link.ld` 等工程文件，是本仓库作者编写或改造的部分。

## 2. `Core\` `Peripheral\` `Startup\` `Debug\`

来自 **WCH（南京沁恒微电子）CH32V30x EVT SDK**，具体许可和芯片使用限制以各源文件头部为准；不要将所有 WCH 文件笼统视为同一许可。

请逐文件保留原始版权声明与许可条件；以下仅为部分文件的 SPDX 声明示例：

```c
/********************************** (C) COPYRIGHT *******************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* SPDX-License-Identifier: Apache-2.0
*******************************************************************************/
```

请保留这些声明。

## 3. `zf_common\` `zf_driver\` `zf_device\` `zf_components\`

来自 **成都逐飞智能设备有限公司（SeekFree）的 CH32V307 开源库**，版本 **V3.7.2**，
许可为 **GPL-3.0**（完整文本见 [`SeekFree-GPL-3.0.txt`](SeekFree-GPL-3.0.txt)）。

- 上游仓库：<https://gitee.com/seekfree/CHV307_Library>
- 本次保留原始代码和声明；TFT180 SPI 请求速率改为18MHz，完整集成改动见根目录 THIRD_PARTY.md；每个源文件头部都保留了逐飞的完整版权与许可声明
- **若你分发包含这部分代码的固件或源码，必须遵守 GPL-3.0 条款**，
  并保留逐飞科技的版权声明

> `zf_device\libzf_device_config.a` 是逐飞提供的**预编译静态库**（摄像头 SCCB 寄存器表、
> IMU660RA / DL1B 配置数据等），随上述 GPL-3.0 库一同分发。
