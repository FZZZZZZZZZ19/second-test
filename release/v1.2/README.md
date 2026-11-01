# V1.2 视觉电控分层构建

2026-11-01，WCH GCC8.2，rv32imafcxw/ilp32f，目标编译链接通过。
Flash63700字节，RAM静态数据及2KiB预留栈合计30908字节；运行栈峰值待上板测量。

完整回归：默认配置138420断言，测试路线配置138424断言。视觉独立链接测试和电控独立链接测试均通过。源码/固件校验见build-report.json，测试记录见test-report.json。

视觉在User/vision，电控在User/control，协议在shared/vision_control.h，参数默认值在config/vision_tuning.h和control_tuning.h。使用细节见docs/V1.2视觉电控分层与调参.md。

未烧录或验证实物，默认禁止发车的标定检查继续生效。
