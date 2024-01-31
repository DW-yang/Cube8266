# Cube8266

## 简介
Cube8266是Cube小车项目的受控端程序
基于 NodeMCU V3 开发板
使用 VSCode + PlatformIO 开发

## 主要特点
1. 使用ESP-NOW技术进行通信，可控范围大，控制延迟低
2. 支持控制端断连自动刹车

## 使用方法

#### 材料
1. NodeMCU V3 开发板 x1
2. L298N两路直流电机驱动模块 x1
3. 18650锂电池充放电升压控制板 x1
4. 锂电池 并联x2
5. 轮胎+直流减速电机 x2
6. 万向轮 x1
7. 一个盒子(*作为车身*) x1

#### 接线
1. 锂电池连接充放电模块，将充放电模块的输出电压调至5V
2. 电机驱动模块连接放电模块，并连接电机，电机接线为左负右正，无需反接，驱动程序已经适配
3. 开发板 VIN连接放电模块正极，GND连接负极，D2连接电机驱动模块INT1，D5连接电机驱动模块INT2，D6连接电机驱动模块INT3，D7连接电机驱动模块INT4

#### 烧录程序
1. Clone本仓库
2. 在源码中配置控制端MAC地址和电机驱动板引脚
3. 编译并烧录程序