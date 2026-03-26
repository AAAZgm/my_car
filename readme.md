# LHL HAL 智能小车底盘控制系统

基于 **STM32CubeMX + HAL 库** 开发的智能小车底盘控制项目，支持多种底盘构型，实现闭环电机控制、里程计估算与上位机/蓝牙通信。

---

## 项目总览

| 子项目 | 底盘类型 | 电机数 | 运动学模型 |
|---|---|---|---|
| `lhl_car_2diff` | 两轮差速 | 2 | 差速运动学 (v, w) |
| `lhl_car_3ominiwheel` | 三轮全向 | 3 | 三轮全向运动学 (vx, vy, w) |
| `lhl_car_4diff_test` | 四轮差速（双驱并联） | 4 | 差速运动学 (v, w) |
| `lhl_car_4ominiwheel_test` | 四轮麦克纳姆轮 | 4 | 麦轮运动学 (vx, vy, w) |

---

## 硬件平台

- **MCU**: STM32F407VET6 (Cortex-M4, 168MHz, LQFP100)
- **开发工具**: STM32CubeMX + Keil MDK-ARM V5
- **调试接口**: SWD (PA13/PA14)

### 公共外设

| 外设 | 用途 | 引脚 |
|---|---|---|
| USART1 | 上位机通信 (115200bps, DMA发送) | PA9(TX), PA10(RX) |
| USART2 | 蓝牙 HC-08 通信 (中断接收) | PD5(TX), PD6(RX) |
| ADC1 | 电池电压检测 (DMA循环采集) | PB0 (CH8) |
| GPIO-LED | 状态指示 (低电平点亮) | PE10 |
| GPIO-KEY | 按键输入 | PE1, PE0(上拉), PD3(下拉) |
| QMI8658 | 6轴IMU (I2C, Mahony姿态融合) | 预留，未启用 |

---

## 各子项目硬件资源分配

### lhl_car_2diff — 两轮差速

| 定时器 | 模式 | 用途 |
|---|---|---|
| TIM1 CH3/CH4 | PWM | M1(左)电机驱动 |
| TIM9 CH1/CH2 | PWM | M2(右)电机驱动 |
| TIM5 CH1/CH2 | 编码器 | M1(左)编码器 |
| TIM4 CH1/CH2 | 编码器 | M2(右)编码器 |
| TIM2 | 定时中断 (50ms) | PID控制 (20Hz) + ADC触发 |

- 轮半径 65mm，轮距 250mm
- 每圈脉冲 1320 (11对极 x 4倍频 x 30减速比)
- PID: Kp=0.5, Ki=0.1, Kd=0

### lhl_car_3ominiwheel — 三轮全向

| 定时器 | 模式 | 用途 |
|---|---|---|
| TIM1 CH1-CH4 | PWM | M1(前) + M2(左)电机驱动 |
| TIM9 CH1/CH2 | PWM | M3(右)电机驱动 |
| TIM5 CH1/CH2 | 编码器 | M1(前)编码器 |
| TIM4 CH1/CH2 | 编码器 | M3(右)编码器 |
| TIM2 CH1/CH2 | 编码器 | M2(左)编码器 |
| TIM6 | 定时中断 (20ms) | PID控制 (50Hz) |
| TIM8 | 定时中断 | 里程计+消息发送 + ADC触发 |

- 轮半径 85mm，机器人半径 150mm (Y型120°布局)
- PID: Kp=0.5, Ki=0.1, Kd=0

### lhl_car_4diff_test — 四轮差速（双驱并联）

| 定时器 | 模式 | 用途 |
|---|---|---|
| TIM1 CH1-CH4 | PWM | M1(后左) + M2(前左)电机驱动 |
| TIM9 CH1/CH2 | PWM | M3(后右)电机驱动 |
| TIM10 CH1 / TIM11 CH1 | PWM | M4(前右)正转/反转 |
| TIM5 CH1/CH2 | 编码器 | M1(后左)编码器 |
| TIM4 CH1/CH2 | 编码器 | M3(后右)编码器 |
| TIM2 | 定时中断 (50ms) | PID控制 (20Hz) + ADC触发 |

- 轮半径 65mm，同侧双电机并联驱动
- PID: Kp=0.5, Ki=0.15, Kd=0

### lhl_car_4ominiwheel_test — 四轮麦克纳姆轮

| 定时器 | 模式 | 用途 |
|---|---|---|
| TIM1 CH1-CH4 | PWM | M1(左前) + M4(右后)电机驱动 |
| TIM9 CH1/CH2 | PWM | M2(右前)电机驱动 |
| TIM10 CH1 / TIM11 CH1 | PWM | M3(左后)正转/反转 |
| TIM5 CH1/CH2 | 编码器 | M1(左前)编码器 |
| TIM4 CH1/CH2 | 编码器 | M2(右前)编码器 |
| TIM3 CH1/CH2 | 编码器 | M3(左后)编码器 |
| TIM2 CH1/CH2 | 编码器 | M4(右后)编码器 |
| TIM2 | 定时中断 (50ms) | PID控制 (20Hz) |
| TIM8 | 定时中断 | 里程计+消息发送 + ADC触发 |

- 轮半径 65mm，轴距 200mm，轮距 200mm
- 4个独立PID控制器: Kp=0.5, Ki=0.15, Kd=0

---

## 软件架构

```
上位机(RDK) / 蓝牙(HC-08)
        |
   UART1 / UART2 中断接收
        |
   环形缓冲区 (myserial.c)
        |
   命令解析 & 校验 (while_Command_GetCommand)
        |
   use_msg() -- 区分 RDK / 蓝牙来源
        |
   逆运动学分解 (kinematic.c)
        |
   设置各轮目标速度 --> PID控制器
        |
   定时器中断 (20Hz/50Hz):
     编码器读取 -> PID计算 -> PWM输出
     里程计更新 -> 状态帧发送(DMA)
```

### 核心模块

| 模块 | 文件 | 功能 |
|---|---|---|
| 电机控制 | `motor.c/h` | PWM设置、编码器读取、PID闭环控制 |
| 运动学 | `kinematic.c/h` | 正/逆运动学解算（差速/全向/麦轮） |
| 串口通信 | `myserial.c` | 环形缓冲区、命令解析、调试打印 |
| 协议处理 | `msg.c` | 状态帧发送、控制命令解析 |
| 电池检测 | `adc_dma.c` | ADC+DMA采集、低通滤波、电压转换 |
| IMU驱动 | `QMI8658.c` | 6轴IMU读写、Mahony姿态融合（预留） |
| 外设驱动 | `led.c` / `key.c` | LED和按键控制 |

---

## 通信协议

自定义二进制帧协议，速度数据以 **int16 x 1000** 传输（保留3位小数精度）。

**发送帧**（向上位机回传状态）:
```
帧头(0xAA) + 电池电压(2B) + 速度数据 + 里程计(x,y,angle) + 校验和 + 帧尾(0x7E)
```

**接收帧**（上位机/蓝牙命令）:
```
帧头(0xAA) + 速度命令 + 任务字段 + 校验和 + 帧尾(0x01=RDK, 0x02=HC-08)
```

---

## 运动学模型

### 差速 (2diff / 4diff)
- 逆运动学: `v_left = v - w * L/2`, `v_right = v + w * L/2`
- 正运动学: `v = (v_left + v_right) / 2`, `w = (v_right - v_left) / L`

### 三轮全向 (3ominiwheel, Y型120°布局)
- 逆运动学:
  - `w1 = vx + R * wz`
  - `w2 = -0.5 * vx + (sqrt(3)/2) * vy + R * wz`
  - `w3 = -0.5 * vx - (sqrt(3)/2) * vy + R * wz`
- 正运动学:
  - `vx = (2*w1 - w2 - w3) / 3`
  - `vy = (w2 - w3) / sqrt(3)`
  - `wz = (w1 + w2 + w3) / (3*R)`

### 麦克纳姆轮 (4ominiwheel)
- 逆运动学:
  - `v_LF =  vx - vy - (a+b) * wz`
  - `v_RF =  vx + vy + (a+b) * wz`
  - `v_LR =  vx + vy - (a+b) * wz`
  - `v_RR =  vx - vy + (a+b) * wz`
- 正运动学:
  - `vx = (v1 + v2 + v3 + v4) / 4`
  - `vy = (-v1 + v2 + v3 - v4) / 4`
  - `wz = (-v1 + v2 - v3 + v4) / (4*(a+b))`

---

## 使用说明

1. 使用 STM32CubeMX 打开对应子项目下的 `.ioc` 文件查看/修改硬件配置
2. 使用 Keil MDK-ARM V5 打开 `MDK-ARM/` 目录下的工程文件编译
3. 通过 SWD 接口下载固件到 STM32F407VET6
4. 通过 USART1 连接上位机或 USART2 连接蓝牙模块进行控制
