# V1.50.19 run 5与保存阈值

固件second-test-v1.50.19.hex。control菜单新增run 5，左右固定5%占空比。菜单顺序为run 30、image、vofa、VOFA target m/s、stop、run 5、back。返回或stop停止电机。VOFA继续使用速度闭环。

保存要求STATIC模式、两侧PWM为零。测速有效时，两侧速度必须有限且绝对值小于0.5m/s。达到0.5m/s即拦截。Flash格式继续为schema8。

八组主机测试通过，覆盖0.49允许/0.5拒绝、run 5菜单请求、真实中断5%输出、保持与停止。WCH目标构建通过，Flash86268字节、RAM54192字节。尚未上板验证。
