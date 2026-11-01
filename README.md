# second-test · CH32V307 智能车 V1.2

四轮齿轮联动、左右两路差速电控工程。由模板 `mode-for-ch32v307vct@7673725` 集成 WCH/逐飞底层并加入应用代码。

已用 WCH GCC 8.2 完成实际目标编译链接，生成 ELF、HEX、BIN；主机回归测试覆盖生产算法源码。**尚未上板验证，默认实测参数为空，保持静态调试并禁止发车。** 完整比赛路线还需现场配置和验证。

## 视觉与电控分层

视觉实现放在 `User/vision`，电控算法放在 `User/control`，通过 `User/shared/vision_control.h` 的 `VisionOutput` 连接。调参分别改 `User/config/vision_tuning.h` 和 `control_tuning.h`。详见[V1.2分层、替换和调参说明](docs/V1.2视觉电控分层与调参.md)。

## 文档阅读顺序（当前V1.2）

1. [项目结构、函数调度、主流程与全部参数](docs/项目结构与主流程参数说明.md)。
2. [代码书写、构建与上板流程](docs/代码书写流程.md)。
3. [视觉链路实现](docs/视觉链路实现说明.md)：Otsu、连通域、外轮廓、方向角和像素距离。
4. [视觉电控分层、替换与调参](docs/V1.2视觉电控分层与调参.md)。
5. [电控算法原理](docs/智能车电控算法开发文档.md)与[真实代码接口](docs/智能车完整代码开发文档.md)。
6. [两份epro引脚核对](docs/hardware/引脚与驱动核对.md)、[当前固件和验证](release/v1.2)、[版本记录](docs/版本记录.md)。

视觉按25mm白线、20cm平行间距、188×120灰度输入设计，输出方向角和像素距离；当前距离未转成米或直接用于减速。默认CAL_VISION_V2_VALIDATED=0，需验证方向增益、帧时序和场地效果后启用。内部孔洞轮廓与折点策略现有边界在视觉说明中列出。

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

上电 UART3 PB10/PB11 输出115200日志；屏幕显示静态状态，五向右键看二值图。先填 `User/config/vehicle_calibration.h` 的实测数据，再调 PI 与视觉参数。中键准备、KEY1发车、KEY2停车；全部标定条件未通过时不能准备。运行中没有远程控制或串口改参。

正式路线在 `User/config/track_profile.h` 配置，不要把格式示例当现场地图。IMU 和电流闭环未启用；实际驱动板没有接回电流/故障反馈。

来源和第三方修改见 [THIRD_PARTY.md](THIRD_PARTY.md) 与 [LICENSES](LICENSES/README.md)。
