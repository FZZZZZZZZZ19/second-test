# second-test · CH32V307 智能车 V1.50

V1.50.20根据台架观察修正右侧方向。前进PH电平为左1、右1，encoder_sign为左-1、右-1。旧参数格式2/4/5/6/7/8加载时更新编码器符号，保存为格式9后继续按保存值加载。停车执行para → save to flash保存一次。

V1.50.19按实测反馈调整保存阈值，两侧速度绝对值均小于0.5m/s时允许保存，仍要求停车模式和零PWM。control菜单新增run 5，左右固定5%占空比，stop或返回停止。run 30固定30%，vofa运行速度闭环。菜单顺序为run 30、image、vofa、VOFA target、stop、run 5、back。

V1.50.18修复VOFA target保存。保存检查STATIC模式、零PWM及反馈速度绝对值小于0.5m/s，允许停车时原始计数抖动，已移除额外500ms等待。VOFA只启动速度闭环，要求参数合法、编码器反馈有效和目标大于0。独立run 30提供30%开环调试。比赛start的完整标定检查继续生效。

当前版本 **V1.50.20**。根据2026-11-19实测，修正右电机方向后，两侧前进的编码器原始计数均为负，方向设置为左-1、右-1，使前进测速统一为正。旧Flash记录升级到schema9时应用这组方向，停车保存一次后持久保留。摄像头与队友视觉接口使用 `image[120][188]`，即120行、188列的灰度图。列号 `u=0..187`，行号 `v=0..119`，视觉直接用整帧坐标和原始八参计算地面位置。屏幕宽128像素，预览显示原图第30～157列。 [固件](release/v1.50.20/second-test-v1.50.20.hex) · [标定与参数升级说明](docs/2026-11-19相机标定记录.md)

四轮齿轮联动、左右两路差速电控工程。由模板 `mode-for-ch32v307vct@7673725` 集成 WCH/逐飞底层并加入应用代码。

V1.50.9已完成内容：MT6701CT-ACD按1024PPR、现有TI2二倍频及用户确认的车轮直连1:1，左右每圈均设2048计数。VOFA开环调试保留I0/I2设定目标速度，I1/I3测速有效时发送实测m/s，无效时发送NaN。para → vision增加camera calib，独立编辑ground_h0～ground_h7。键盘原OK键改为负号-，切换输入正负，SAVE统一确认并写Flash；CLR清空、BACK/KEY1取消返回。相机系数用科学计数法显示小数值，原有硬件页、返回位置记忆和旧保存参数继续保留。

V1.50基于主干V1.3，加入五向导航菜单与KEY1返回、数字键盘与Flash保存、run 30开环调试和VOFA速度闭环调试，未合入V1.4 IMU内环。下述为菜单早期版本记录（大小不代表当前固件）。V1.50.8已完成目标板编程、校验和复位，实际五向键操作、Flash掉电和巡线仍待验证。**实测标定尚不完整，禁止巡线发车。** 当前产物为 `build/target/second-test-v1.50.hex`，历史release不是当前固件。

2026-11-17尺寸已更新并写入目标板Flash：轮距166.6mm、左右轮直径各33.7mm、前后轴距35.5mm，启动默认值分别为0.1666、0.0337、0.0355m。轮距指左右轮中心间距。车长/车宽仅作档案，可保持0；该次尚缺编码器每圈计数及方向确认；计数已在下述V1.50.9更新，方向仍待确认，未设置标定完成掩码。烧录校验及复位后RAM读回通过，备用参数槽sequence=7，原增益和目标0.200保留。此尺寸版Flash86796B、RAM31596B；上段86772B对应此前菜单修复版。详见[硬件参数说明](docs/V1.50屏幕菜单与调试.md)。

## 视觉与电控分层

视觉实现放在 `User/vision`，电控算法放在 `User/control`，通过 `User/shared/vision_control.h` 的 `VisionOutput` 连接。调参分别改 `User/config/vision_tuning.h` 和 `control_tuning.h`。详见[V1.2分层、替换和调参说明](docs/V1.2视觉电控分层与调参.md)。

## 文档阅读顺序（当前V1.50）

菜单使用先读[V1.50屏幕菜单与调试](docs/V1.50屏幕菜单与调试.md)。

1. [电控算法原理](docs/智能车电控算法开发文档.md)：先理解车怎么转，跟完一个右转例子。
2. [项目结构、函数调度、主流程与参数](docs/项目结构与主流程参数说明.md)：术语白话解释、文件位置和调参。
3. [代码书写、构建与上板流程](docs/代码书写流程.md)：分阶段操作，每步有完成标准和排查方向。
4. [完整代码开发文档](docs/智能车完整代码开发文档.md)：沿调用链读源码，末尾查真实接口。

需要替换算法时看[分层与调参说明](docs/V1.2视觉电控分层与调参.md)；视觉细节见[视觉链路实现](docs/视觉链路实现说明.md)。接线见[两份epro核对](docs/hardware/引脚与驱动核对.md)，固件与历史见[历史V1.2构建记录](release/v1.2)和[版本记录](docs/版本记录.md)。

视觉输出米制横向偏差、前向距离和右正曲率；需要实测 ground_h0..h7，已填入用户提供的八参数；其他必要标定缺失时仍保持 invalid。CAL_VISION_V2_VALIDATED 仍默认0，安全校验未改动。

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

上电主菜单为 `start / para / control`。普通菜单用五向上/下选择、中键点按确认，KEY1点按返回上一级；数字键盘用四向移动、中键确认，增加SAVE直接保存。start自动准备并等待新鲜视觉结果后巡线，同时显示当前灰度画面；para分视觉、电控和hardware硬件参数，数字键盘SAVE保存，para也可save to flash保存，reset后继续加载。control提供run 30、image、vofa、VOFA目标速度和stop。stop停止电机和图像采集，屏幕继续工作。

重点增益为视觉 `lateral_kp / lateral_kd` 和电控 `kp_left/right / ki_left/right`。speed Kp/Ki(L+R)同时修改两侧，另外保留单侧入口。Flash写schema=9，兼容2/4/5/6/7/8。未标定的零值可保存，但仍不能发车。

## VOFA UART3协议与用法

UART3 PB10(TX)接USB转TTL的RX并共地，使用3.3V逻辑。电脑选择对应COM、115200、8N1、JustFloat。依据[VOFA官方JustFloat协议](https://www.vofa.plus/docs/learning/dataengines/justfloat/)，每帧为四个float32小端数，后接 `00 00 80 7F`，共20字节，每20ms发送一次。四通道依次为**左要求速度、左编码器实测速度、右要求速度、右编码器实测速度**，单位m/s；要求速度是加速规划后送给PI的目标。

V1.50.18修复VOFA target保存。保存检查STATIC模式、零PWM及反馈速度绝对值小于0.5m/s，允许停车时原始计数抖动，已移除额外500ms等待。VOFA只启动速度闭环，要求参数合法、编码器反馈有效和目标大于0。独立run 30提供30%开环调试。比赛start的完整标定检查继续生效。

正式路线在 `User/config/track_profile.h` 配置，不要把格式示例当现场地图。IMU 和电流闭环未启用；实际驱动板没有接回电流/故障反馈。

来源和第三方修改见 [THIRD_PARTY.md](THIRD_PARTY.md) 与 [LICENSES](LICENSES/README.md)。

V1.3的米制路径算法继续使用，坐标、标定与符号见[V1.3说明](docs/V1.3米制路径控制.md)。

相机标定路径为para → vision → camera calib，八个系数按原数值输入；小系数科学计数显示，参数记录为schema9；旧Flash迁移后停车执行一次para → save to flash，后续菜单修改按新记录保持。详见[菜单说明](docs/V1.50屏幕菜单与调试.md)。

V1.50.9依据[厂家手册第5页型号表](https://www.magntek.com.cn/upload/pdf/202407/MT6701_Rev.1.8.pdf)确认ACD出厂AB=1024PPR，现有`encoder_quad_init`使用TI2单通道双边沿，即2048计数/编码器轴一圈；车轮轴直连1:1，所以counts_left/right=2048。14位绝对角度的16384与TI12四倍频的4096均不适用于当前计数模式。未设置标定完成掩码：保留30%开环，可先观察左右测速正负，确认前进均为正后再启用速度闭环。目标构建Flash86884B、RAM31596B，七组主机测试通过。
