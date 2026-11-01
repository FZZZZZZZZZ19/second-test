# second-test · CH32V307 智能车 V1.1

四轮齿轮联动、左右两路差速电控工程。由模板 `mode-for-ch32v307vct@7673725` 集成 WCH/逐飞底层并加入应用代码。

已用 WCH GCC 8.2 完成实际目标编译链接，生成 ELF、HEX、BIN；主机回归测试覆盖生产算法源码。**尚未上板验证，默认实测参数为空，保持静态调试并禁止发车。** 完整比赛路线还需现场配置和验证。

## 新版视觉

已按2026-11-01新增视觉大纲加入 Otsu、连通道路、外轮廓追踪、截断区间/折点与方向角/像素距离接口。详见 [V1.1新版视觉实现说明](docs/V1.1新版视觉实现说明.md)。默认 `CAL_VISION_V2_VALIDATED=0`，必须重新验证方向增益和最坏帧处理时间后启用；内部孔洞轮廓枚举及完整折点几何策略的现有边界在说明中列出。

## 先读这里

- [V1.0 代码书写流程](docs/V1.0代码书写流程.md)：从模板到当前源码的实现顺序、构建和上板步骤。
- [V1.0 项目结构、函数调度、主流程与参数](docs/V1.0项目结构与主流程参数说明.md)：逐文件对应函数，解释轮子怎么动、各参数怎么填。
- [两份 epro 的引脚与驱动核对](docs/hardware/引脚与驱动核对.md)：原始网络证据、接口脚号、驱动限制。
- [默认配置固件与验证记录](release/v1.1)：本次构建产物；标定后请重新编译。

## 构建

Python 3.9+、电脑 GCC、MounRiver 配套 WCH GCC 8.2。仓库根目录执行：

```powershell
python tools/test_host.py --cc C:/mingw64/bin/gcc.exe
python tools/build.py --toolchain "E:/RISC-V/MounRiver_Studio3/resources/app/resources/win32/components/WCH/Toolchain/RISC-V Embedded GCC/bin"
```

工具链路径按本机安装位置修改，也可设置 `WCH_TOOLCHAIN`。产物在 `build/target`，测试记录在 `build/host`。必须保持 `rv32imafcxw / ilp32f`，以匹配逐飞预编译配置库。

MounRiver 导入根目录 `.project`，或打开 `.mrs/CH32V307VCT.mrs-workspace`。本次验证的是 CLI 构建，未做 GUI 编译与烧录验证。启动文件为 D8C，应用从 `User/main.c`、`User/app_boot.c` 进入。

## 接线与首次运行

| 侧别 | EN PWM | PH 方向 | 编码器 |
|---|---|---|---|
| 左 | PE9，TIM1 MAP3 CH1 | PE11 | PA0/PA1，TIM2 |
| 右 | PE14，TIM1 MAP3 CH4 | PE13 | PD12/PD13，TIM4 MAP1 |

对应前提是主板 H3 与驱动板 CN5 同脚号直连。右路第3脚是 PH，第4脚是 EN。两板共地，编码器供电与输出电平核对3.3V系统。PH 的机械前进极性仍须实测。

上电 UART3 PB10/PB11 输出115200日志；屏幕显示静态状态，五向右键看二值图。先填 `User/vehicle_calibration.h` 的实测数据，再调 PI 与视觉参数。中键准备、KEY1发车、KEY2停车；全部标定条件未通过时不能准备。运行中没有远程控制或串口改参。

正式路线在 `User/track_profile.h` 配置，不要把格式示例当现场地图。IMU 和电流闭环未启用；实际驱动板没有接回电流/故障反馈。

## 开发文档

- [电控算法开发文档 V2.0](docs/智能车电控算法开发文档.md)：参考车模结构、四轮差速运动学、路况处理和调试流程。
- [完整代码开发文档 V1.0](docs/智能车完整代码开发文档.md)：从模板开始的模块接口、数据结构、调度、状态机、实现任务和验收计划。

这两份是此前的算法和接口设计依据；实际 V1.0 函数、文件、功能边界以新增的两份实现说明为准。引脚现已按 epro 核对，尺寸、机械极性和现场元素仍须实测。参考车照片取自用户提供的考核细则第2页，见 `docs/assets`。

来源和第三方修改见 [THIRD_PARTY.md](THIRD_PARTY.md) 与 [LICENSES](LICENSES/README.md)。
