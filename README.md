# second-test · CH32V307 智能车 V1.50

四轮齿轮联动、左右两路差速电控工程。由模板 `mode-for-ch32v307vct@7673725` 集成 WCH/逐飞底层并加入应用代码。

当前补丁版本V1.50.4：普通菜单改用五向上/下选择、中键点按确认，KEY1专门返回上一级（主菜单无上级时不操作）。紧凑4×4键盘加入SAVE：确认当前输入并写Flash；OK仅应用RAM，BACK或KEY1取消返回。键盘四向和中键、120×72布局、粗黑体和实物方向修正继续保留。

V1.50基于主干V1.3，加入五向导航菜单与KEY1返回、数字键盘与Flash保存、run 30开环调试和VOFA速度闭环调试，未合入V1.4 IMU内环。主机测试与WCH完整构建通过，Flash84768B、RAM31576B。V1.50.4已完成目标板编程、校验和复位，实际五向键操作、Flash掉电和巡线仍待验证。**默认实测参数为空，禁止巡线发车。** 当前产物为 `build/target/second-test-v1.50.hex`，历史release不是当前固件。

## 视觉与电控分层

视觉实现放在 `User/vision`，电控算法放在 `User/control`，通过 `User/shared/vision_control.h` 的 `VisionOutput` 连接。调参分别改 `User/config/vision_tuning.h` 和 `control_tuning.h`。详见[V1.2分层、替换和调参说明](docs/V1.2视觉电控分层与调参.md)。

## 文档阅读顺序（当前V1.50）

菜单使用先读[V1.50屏幕菜单与调试](docs/V1.50屏幕菜单与调试.md)。

1. [电控算法原理](docs/智能车电控算法开发文档.md)：先理解车怎么转，跟完一个右转例子。
2. [项目结构、函数调度、主流程与参数](docs/项目结构与主流程参数说明.md)：术语白话解释、文件位置和调参。
3. [代码书写、构建与上板流程](docs/代码书写流程.md)：分阶段操作，每步有完成标准和排查方向。
4. [完整代码开发文档](docs/智能车完整代码开发文档.md)：沿调用链读源码，末尾查真实接口。

需要替换算法时看[分层与调参说明](docs/V1.2视觉电控分层与调参.md)；视觉细节见[视觉链路实现](docs/视觉链路实现说明.md)。接线见[两份epro核对](docs/hardware/引脚与驱动核对.md)，固件与历史见[历史V1.2构建记录](release/v1.2)和[版本记录](docs/版本记录.md)。

视觉输出米制横向偏差、前向距离和右正曲率；需要实测 ground_h0..h7，默认全零保持 invalid。CAL_VISION_V2_VALIDATED 仍默认0，安全校验未改动。

## 构建

Python 3.9+、电脑 GCC、MounRiver 配套 WCH GCC 8.2。仓库根目录执行：

```powershell
python tools/test_host.py --cc C:/mingw64/bin/gcc.exe
python tools/build.py --toolchain "E:/RISC-V/MounRiver_Studio3/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC/bin"
```

工具链路径按本机安装位置修改，也可设置 `WCH_TOOLCHAIN`。产物在 `build/target`，测试记录在 `build/host`。必须保持 `rv32imafcxw / ilp32f`，以匹配逐飞预编译配置库。

MounRiver 导入根目录 `.project`，或打开 `.mrs/CH32V307VCT.mrs-workspace`。本次验证的是 CLI 构建，未做 GUI 编译与烧录验证。启动文件为 D8C，应用从 `User/main.c`、`User/integration/app_boot.c` 进入。

## 接线与首次运行

| 侧别 | EN PWM | PH 方向 | 编码器 |
|---|---|---|---|
| 左 | PE9，TIM1 MAP3 CH1 | PE11 | PA0/PA1，TIM2 |
| 右 | PE14，TIM1 MAP3 CH4 | PE13 | PD12/PD13，TIM4 MAP1 |

对应前提是主板 H3 与驱动板 CN5 同脚号直连。右路第3脚是 PH，第4脚是 EN。两板共地，编码器供电与输出电平核对3.3V系统。PH 的机械前进极性仍须实测。

上电主菜单为 `start / para / control`。普通菜单用五向上/下选择、中键点按确认，KEY1点按返回上一级；数字键盘用四向移动、中键确认，增加SAVE直接保存。start自动准备并等待新鲜视觉结果后巡线，同时显示当前灰度画面；para分视觉和电控，数字键盘ok应用、save to flash保存，reset后继续加载。control提供run 30、image、vofa、VOFA目标速度和stop。stop停止电机和图像采集，屏幕继续工作。

重点增益为视觉 `lateral_kp / lateral_kd` 和电控 `kp_left/right / ki_left/right`。speed Kp/Ki(L+R)同时修改两侧，另外保留单侧入口。Flash记录schema=4，兼容V1.3 schema=2；未标定的零值可保存，但仍不能发车。

## VOFA UART3协议与用法

UART3 PB10(TX)接USB转TTL的RX并共地，使用3.3V逻辑。电脑选择对应COM、115200、8N1、JustFloat。依据[VOFA官方JustFloat协议](https://www.vofa.plus/docs/learning/dataengines/justfloat/)，每帧为四个float32小端数，后接 `00 00 80 7F`，共20字节，每20ms发送一次。四通道依次为**左要求速度、左编码器实测速度、右要求速度、右编码器实测速度**，单位m/s；要求速度是加速规划后送给PI的目标。

先设置control / VOFA target m/s为正值，再确认vofa，会自动启动两轮速度闭环，屏幕显示目标与实测速度。几何、编码器和方向标定必须先完成。run 30独立保留为固定30%开环调试，不经过PI，且不受比赛duty_limit默认25%的裁剪。退出实时页或执行stop即停止电机和发送。发送时不混入printf日志，本版不接受串口文本命令。详细包示例、保存流程和上板步骤见[V1.50说明](docs/V1.50屏幕菜单与调试.md)。

正式路线在 `User/config/track_profile.h` 配置，不要把格式示例当现场地图。IMU 和电流闭环未启用；实际驱动板没有接回电流/故障反馈。

来源和第三方修改见 [THIRD_PARTY.md](THIRD_PARTY.md) 与 [LICENSES](LICENSES/README.md)。

V1.3的米制路径算法继续使用，坐标、标定与符号见[V1.3说明](docs/V1.3米制路径控制.md)。
