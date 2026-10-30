# 来源与本次改动

代码起点：[mode-for-ch32v307vct](https://github.com/FZZZZZZZZZ19/mode-for-ch32v307vct)，固定提交 `7673725bab177202212c43a409d778aab45c5fab`。

- WCH 内核、外设、调试和启动文件保留各自源文件许可声明。
- 逐飞 CH32V307 库保留 GPL-3.0 版权、许可和 `libzf_device_config.a`；完整文本在 `LICENSES/SeekFree-GPL-3.0.txt`。
- 新增应用代码标注 GPL-3.0-or-later；分发组合工程和固件时同时遵守第三方许可。

模板集成改动：替换 `User/main.c`，在 `User/zf_isr.c` 加入 DVP/TIM6 应用钩子，NMI/HardFault 先将两路 PWM 比较值置零，新增 User 应用模块，Flash 链接长度改248KiB以预留参数页，`.cproject` 排除测试和构建目录。`zf_device_tft180.h` 的 SPI 请求速率从72MHz改18MHz。未删除或改写第三方源文件版权头。

电路网络来自用户提供的两份 epro，原始工程未公开上传；仅提交必要的电气提取结果与哈希。参考车照片沿用原文档中由用户提供的细则截图，见 `docs/assets`。

源码换行统一为 LF，保留原文件字符编码；仓库属性固定文本换行，避免检出平台改变构建输入哈希。
