## DEBUG
### 定时器编码器模式设置
别忘了设置滤波，不然会把噪声也计入
```c
sConfig.IC1Filter = 15;
```

别忘了检查线是不是好的



### DRV8833控制信号
不同于TB6612, DRV8833直接使用两路PWM即可完成控制,但是需要注意PWM信号的频率。一开始使用41kHZ的信号电机没反应，改到5kHz之后便可以了。



### DMA

[【STM32】CubeMX+HAL库之ADC+DMA_hal_adc_start_dma-CSDN博客](https://blog.csdn.net/weixin_45065888/article/details/117872009)

在STM32中，DMA（Direct Memory Access，直接内存访问）控制器用于管理一个或多个外设的存储器访问请求1。每个DMA控制器都有多个数据流，每个数据流又有多个通道1。每个通道都有一个仲裁器，用于处理DMA请求间的优先级1。

数据流（Stream）：数据流是DMA控制器的一部分，用于处理来自特定外设的DMA请求1。每个数据流都可以处理多个通道的请求1。

通道（Channel）：通道是数据流的一部分，用于处理来自特定外设的DMA请求1。每个数据流可以有多个通道，每个通道都映射到不同的外设2。每个数据流只能配置为映射到一个通道，无法配置为映射到多个通道2。



### USART 重定向

[STM32设置串口重定向输出 - 知乎 (zhihu.com)](https://zhuanlan.zhihu.com/p/499311941#:~:text=一、在CUBEMX上配置好串口并使能全局中断并且勾选生成单独的文件二、打开生成的keil工程并配置代码1.点击魔术棒改配置勾选“Use MicroLib”2.打开usart.c添加重定向函数重定向函数： int fputc,(int ch%2C FILE *f) {…)



### CubeMX GPIO重映射

[STM32 CUBEMX 设置GPIO重映射_stm32_cubemx实现管脚重定义-CSDN博客](https://blog.csdn.net/qq_27215587/article/details/104387965)