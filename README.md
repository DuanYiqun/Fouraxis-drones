# Fouraxis-drones  

This is a simple demo of four-axes drones. It uses MPU6050 sensor to detect the position and use MSP430 as it's control core.
For compile this project you need to install softwares and harddrives from TI on your computer.    
这是一个以前电设时期写的四旋翼飞行器的demo代码。
编译此工程最好去TI 下载MSP430 对应的[软件套件和驱动程序](https://e2echina.ti.com/question_answer/microcontrollers/msp430/f/55/t/17817) 根据不同版本的MSP430 可能需要调整一些寄存器配置。 不过这个工程写的还比较模块化，该起来比较方便。       
本来想写个详细的原理剖析，无奈于最近真的太忙了，把以前写的老东西上传上来，缺的说明慢慢再补。        

在这里MSP430_IIC 和 i2c都是异步通信协议，作为和芯片通信的主要手段。 inv_mpu 是驱动MPU6050的程序，并且返回姿态数据给主控制器。bluth 是蓝牙通讯部分，负责四旋翼和控制台的通信。在姿态控制方面主要采用了双环pid 控制。 如果以后有时间准备在pid控制方面加入一些reinforcement learning的程序。正好最近在学，看看效果如何。 
