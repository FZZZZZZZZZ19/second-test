# V1.1 新视觉构建

2026-11-01，WCH GCC8.2，rv32imafcxw/ilp32f，完整构建通过。

Flash 63316字节；RAM静态数据及2KiB预留栈合计30924字节，32KiB配置尚余1844字节，运行峰值待测。
主机测试：默认配置5组138420断言，测试路线配置5组138424断言。原第三方库警告见build-report.json。

包含Otsu、连通道路、外轮廓、截断/折点和方向/像素距离接口。参见docs/V1.1新版视觉实现说明.md。默认CAL_VISION_V2_VALIDATED=0，未完成重新标定禁止发车；没有执行烧录或实车赛道测试。

固件及源文件SHA-256见build-report.json，主机结果见test-report.json。
