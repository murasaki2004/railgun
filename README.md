# railgun

使用两个伺服电机(个人做的实物使用的舵机)调整角度的可瞄准轨道炮，预定射程为半径5m的半圆
MCU 采用STM32F103系列
正在预想一个非串口的靠谱简单操作方法，目前方案为OLED屏幕+摇杆

## 使用方法
    main.c
本程序使用正点原子的标准例程作为底层，编译下载完成后使用串口与开发板链接
改成16位显示，会有准备的提示(01)并且会将舵机改回0°，之后将需要输入的数据转成16进制通过串口发送即可
    cubeMX HAL
依照MX的信息接完线后，oled屏幕上会显示目前xy的值，使用摇杆可以进行修改，按下即可执行
（还没有做发射控制

## CudeMX配置的HAL版本

见压缩包，里面有MX和MDK可以直接打开的文件