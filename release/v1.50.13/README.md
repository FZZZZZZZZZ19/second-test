# V1.50.13 原例程连续采集接入

烧录second-test-v1.50.13.hex。使用与已验证原工程相同的FLASH256KiB+RAM64KiB芯片分配，链接RAM区为0x20000040起63KiB；Flash应用区仍为248KiB，末端两页参数保留。请勿全片擦除。

已删除自定义逐帧ALL_CLR/RCV_CLR复位、丢首帧和每帧暂停DMA。进入图像页调用原dvp_camera_init，持续采集并使用原mt9v03x_finish_flag；停止只发生在stop/back/维护保存。相机.c、相机分发、DVP和软I2C模块按可用工程的rv32imafc/O0/ilp32f编译，控制和视觉仍保留原优化。软I2C的volatile延时修复仍保留。

完成帧在DVP中断中复制到独立应用帧，主循环处理期间不改写此副本，不增加裁切显示缓冲。RAM54192B（含预留栈）、Flash86228B，编译及已有主机回归通过。实际帧复制中断耗时、图像更新、再次进入及整车运行尚未在实物验证；当前无法连接调试，不能声称相机已修复成功。

build-report.json保留构建器原文件名second-test-v1.50.*，本目录只是改成明确版本名，字节和哈希不变。
