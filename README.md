# railgun
    使用两个伺服/步进电机调整角度的可瞄准轨道炮，预定射程半径5m，覆盖前方半圆
    实验板采用正点原子的STM32f103zet6
    实际制作时使用stm32f103c8t6最小系统

# 关于mian.c中引用的头文件
    math、stm32f10x应该是MDK和ST提供的开发库
    
    usart、delay、sys是正点原子在其开发板附属资料中工程模板里的文件
    可以在正点原子的官方论坛中下载
