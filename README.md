# Bluetooth Paper Flip — ESP32-H2 BLE 翻页遥控器

## 概述

本项目是一个基于 **ESP32-H2** 芯片的低功耗蓝牙（BLE）翻页遥控器固件，通过两个物理按键向电纸书（E-Ink）设备发送翻页、确认、返回和关机等指令。设备以 BLE Peripheral 模式运行，广播名称为 `RC-DD-X4`，扫描响应名称为 `DDT`，连接后通过 Notify 向主机推送单字节命令码。

## 硬件平台

| 项目 | 说明 |
|------|------|
| 芯片 | ESP32-H2（RISC-V 单核，支持 BLE 5.0 + Thread/Zigbee） |
| 开发框架 | Arduino-ESP32（PlatformIO / Arduino IDE） |
| RGB LED | WS2812，接 GPIO10，通过 RMT 外设驱动（`esp32-hal-rgb-led.h`） |
| 按键 1（下一页） | GPIO11，INPUT_PULLUP，Active LOW |
| 按键 2（上一页） | GPIO12，INPUT_PULLUP，Active LOW |
| 串口调试波特率 | 115200 |

> 启动时串口输出 `esp32h2 paper turner` 确认芯片型号。

## BLE 协议

### 命令码定义

| 命令码 | 宏定义 | 触发方式 | 说明 |
|--------|--------|----------|------|
| 0x01 | `CMD_UP` | 短按「下一页」键释放 | 下一页 |
| 0x02 | `CMD_DOWN` | 短按「上一页」键释放 | 上一页 |
| 0x03 | `CMD_BACK` | 长按「下一页」键 1.5s | 返回 |
| 0x04 | `CMD_CONFIRM` | 长按「上一页」键 1.5s | 确认 |
| 0x05 | `CMD_SLEEP` | — | 休眠（固件预留） |
| 0x06 | `CMD_SHUTDOWN` | 双键同时长按 2.5s | 关机 |
| 0x07 | `CMD_REMOTE_OFF` | — | 遥控关闭（固件预留） |

## 核心功能

### 1. 按键消抖与边沿检测

采用 30ms 消抖窗口（`DEBOUNCE_MS`），每个 loop 周期采样原始电平，仅在电平稳定超过消抖窗口后才更新稳定状态并检测下降沿（按下事件）。只有在按键释放时才发送翻页命令，以区分短按与长按。

### 2. 短按 / 长按识别

- **短按**：按下后在 1.5s 内释放 → 发送翻页命令（`CMD_UP` / `CMD_DOWN`）
- **长按**：单键持续按下 ≥ 1500ms（`LONG_PRESS_CONFIRM_MS`）→ 发送功能命令（`CMD_BACK` / `CMD_CONFIRM`），长按触发后不再发送翻页命令

### 3. 双键同时按下检测

- 设置 50ms 宽限窗口（`BOTH_PRESS_GRACE_MS`）：单键按下后延迟 50ms 再处理，以等待另一键是否同时按下
- 两键同时按下时取消所有待处理单键命令
- 双键持续长按 ≥ 2500ms（`BOTH_PRESS_SHUTDOWN_MS`）→ 发送 `CMD_SHUTDOWN`

### 4. LED 状态指示

| 状态 | LED 表现 |
|------|----------|
| BLE 未连接 | 红灯闪烁，500ms 周期 |
| BLE 连接成功 | 绿灯闪烁 3 次后熄灭，200ms 周期 |
| 连接后正常工作 | LED 熄灭 |
| BLE 断开 | 取消绿灯闪烁，恢复红灯等待 |

LED 亮度通过 `BRIGHTNESS_SCALE`（168/255 ≈ 66%）进行降亮处理，避免刺眼。

### 5. 连接时序

BLE 连接成功后强制等待 3 秒再开始处理按键事件，确保主机端完成服务发现与特征订阅，避免丢失早期命令。

## 引脚映射

| GPIO | 功能 |
|------|------|
| 10 | WS2812 RGB LED 数据线 |
| 11 | 按键「下一页」（CMD_UP），上拉 |
| 12 | 按键「上一页」（CMD_DOWN），上拉 |

## 编译与烧录

1. 安装 [Arduino-ESP32](https://github.com/espressif/arduino-esp32) 核心库（需支持 ESP32-H2）
2. 选择开发板：**ESP32H2 Dev Module**
3. 上传 `src/main.cpp` 中的固件
4. 串口监视器设置波特率 115200 查看调试日志

## 文件结构

```
bluetooth_paper_flip/
├── src/
│   └── main.cpp      # 主固件源码
└── README.md          # 本文档
```

## 状态机简图

```
[上电] → [BLE 广播 + 红灯闪烁]
              │
         连接成功
              │
    [绿灯闪烁 3 次] → [等待 3s]
              │
    ┌─────────┴──────────┐
    │                    │
 短按释放              长按 ≥1.5s
 CMD_UP/DOWN          CMD_BACK/CONFIRM
    │                    │
    └────────┬───────────┘
             │
      双键同按 ≥2.5s
       CMD_SHUTDOWN
```
