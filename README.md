## GNSSINS-MATLAB

基于i2Nav开源代码二次开发所成，武汉大学测绘学院《组合导航》课程设计。

感谢GNSS中心王立强博士的辛勤付出。

### 简介
基于MATLAB的GNSS/INS组合导航算法，包括惯导机械编排算法，扩展卡尔曼滤波实现的组合导航，IMU误差反馈和补偿等。

主要函数及说明：

| 文件名 | 主要功能描述 |
| ---- | ---- |
| gnssins.m | 程序入口，处理主循环，调用其他程序 |
| ProcessConfig.m |  程序参数配置 |
| Initialize.m |  程序初始化 |
| InsMech.m |  捷联惯导机械编排算法 |
| InsPropagate.m |  误差状态预测 |
| GNSSUpdate.m |  GNSS位置/速度观测更新 |
| ODONHCUpdate.m |  ODO/NHC观测更新 |
| ZUPTUpdate.m |  ZUPT/ZARU更新 |
| detectZUPT.m |  ZUPT探测 |
| ErrorFeedback.m |  误差反馈 |
| plot-function/calc_error.m |  计算导航误差 |
| plot-function/plot_result.m |  绘制导航结果 |
| plot-function/plot_std.m |  绘制导航状态STD |
| plot-function/plot_imuerror.m |  绘制估计的IMU误差 |
| function | 常用基础函数 |
| dataset | 测试数据 |

### 注意事项
程序运行前，需要将function文件夹添加到工作区，通过右键单击function文件夹，选择“添加到路径”->"选定的文件夹和子文件夹"

新的数据运行时，需要修改ProcessCfg.m文件中的文件 路径和初始信息配置

ODO/NHC 每1s更新一次，在0.5s的时候更新，速度用ODO原始数据前后各10个历元平均得到
