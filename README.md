# UFB - Universal Filament Buffer base on BMCU

这是一个给 BMCU 板子用的独立耗材缓冲器固件。

它不和打印机通信，不跑 AMS 协议，也不要求接到 Bambu 打印机上。这个仓库的目标只有一个：把这块板子改成一个离线的、可独立工作的耗材缓冲器控制板。

## 当前逻辑

这版固件现在的行为是：

- 平时按独立耗材缓冲器工作。
- 第一个微动触发时，启动一次自动进料。
- 自动进料一旦开始，会先完整执行完，再回到普通缓冲器逻辑。
- 正常缓冲器补料和自动进料使用同样的前进速度。
- 高阻力版只提高电机输出，不改变触发灵敏性和整体逻辑。

这版不做的事情：

- 不和打印机通信
- 不依赖 AMS 总线
- 不暴露用户侧“手动长按进退料”模式

## LED 状态

每个通道前面有两个 LED：

- `ONLINE`：只反映微动 `KS` 状态
- `STU`：只反映电机动作状态

### `ONLINE` LED

| LED 显示 | `KS` | 含义 |
|---|---:|---|
| 灭 | `0` | 两个微动都没触发 |
| 蓝 | `2` | 只有第一个微动触发 |
| 白 | `1` | 第一个和第二个微动都触发 |
| 红闪 | `3` | 只有第二个微动触发 |

### `STU` LED

| LED 显示 | 含义 |
|---|---|
| 灭 | 电机空闲/停止 |
| 绿 | 前送、自动吸入、送丝 |
| 青蓝 | on-use 压力控制、手动前送 |
| 紫闪 | 回抽、退料、手动回退 |
| 红 | 卡料、故障、停止锁存 |

## 先看你该用哪个版本

### `filament_buffer_bmcu`

标准版，适合普通 PTFE 长度和阻力。

### `filament_buffer_bmcu_high_force`

高阻力版，适合更长、更弯、摩擦更大的送丝路径。

这个版本现在只提高电机输出，不改灵敏性。和标准版相比，关键差异是：

```cpp
// 标准版
autoload/feed pwm = 850
retract pwm       = 850

// 高阻力版
autoload/feed pwm = 960
retract pwm       = 900
```

## 你需要准备什么

开始前需要：

- `git`
- `python3`
- `platformio`
- 本地可执行的 `wchisp`

安装 PlatformIO：

```bash
python3 -m pip install -U platformio
```

确认安装：

```bash
pio --version
```

## 如何编译

在仓库根目录执行。

编译标准版：

```bash
env PLATFORMIO_CORE_DIR=$PWD/.platformio pio run -e filament_buffer_bmcu
```

编译高阻力版：

```bash
env PLATFORMIO_CORE_DIR=$PWD/.platformio pio run -e filament_buffer_bmcu_high_force
```

生成产物：

```text
.pio/build/filament_buffer_bmcu/firmware.bin
.pio/build/filament_buffer_bmcu_high_force/firmware.bin
```

## 如何刷写

先确认设备处于 ISP 模式并能被识别：

```bash
./wchisp probe
```

刷标准版：

```bash
./wchisp flash .pio/build/filament_buffer_bmcu/firmware.bin
```

刷高阻力版：

```bash
./wchisp flash .pio/build/filament_buffer_bmcu_high_force/firmware.bin
```

刷写成功后通常会看到：

```text
Verify OK
Device reset
```

## 仓库结构

```text
src/
  app/
    main.cpp
  control/
    buffer_constants.h
    hall_calibration.cpp
    hall_calibration.h
    motion_control.cpp
    motion_control.h
  hardware/
    adc_dma.cpp
    adc_dma.h
    as5600_multi_soft_i2c.cpp
    as5600_multi_soft_i2c.h
    ws2812.cpp
    ws2812.h
  model/
    filament_state.cpp
    filament_state.h
  platform/
    debug_log.cpp
    debug_log.h
    runtime_api.h
    hal/
      irq_wch.h
      time_hw.c
      time_hw.h
  storage/
    nvm_storage.cpp
    nvm_storage.h
reference/
  original_bmcu/
```

最关键的文件：

- `src/app/main.cpp`：启动流程和主循环入口
- `src/control/motion_control.cpp`：缓冲器主逻辑、电机控制、自动进料
- `src/control/buffer_constants.h`：集中管理当前固件里的行为参数和阈值
- `src/control/hall_calibration.cpp`：上电校准逻辑

## `reference/original_bmcu` 是什么

这是原始上游项目的参考副本，只用于对照原始实现，不参与当前固件编译。

可以把它理解成：

- 当前仓库是你真正要刷进板子的独立版本
- `reference/original_bmcu` 只是查资料用的参考代码
