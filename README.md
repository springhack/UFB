# filament buffer bmcu

这是一个给 BMCU 板子用的独立耗材缓冲器固件。

它的目标很简单：不接打印机通信，不跑 AMS 协议，只把这块板子当成一个带电机的耗材缓冲器来用。

如果你是第一次打开这个仓库，可以直接把它理解成：

- 这不是原版 AMS 固件。
- 这是一个已经裁剪过的离线版本。
- 你只需要关心怎么编译、怎么刷写、选哪个模式。

## 这个固件能做什么

- 平时作为一个离线耗材缓冲器工作。
- 插入新料后自动吸入一点，把缓冲器拉回平衡区。
- 空通道时，缓冲器持续拉动或推动超过 `1s`，可以进入手动连续进料或退料。
- 通道里已经有料时，会禁用这套手动连续进退料触发，避免打印时误动作。

## 这个固件不做什么

- 不和打印机通信。
- 不依赖 Bambu AMS 总线。
- 不需要把它接到打印机上才能工作。

如果你只是想把这块板子当成一个“独立耗材缓冲器控制板”，那这个仓库就是给这个用途准备的。

## 先看你该用哪个版本

这个仓库目前有两个编译环境。

### `filament_buffer_bmcu`

标准版。

适合：

- 常规长度的 PTFE 管
- 阻力普通的送丝路径
- 你已经验证标准版工作正常

### `filament_buffer_bmcu_high_force`

高阻力版。

适合：

- PTFE 管更长
- 路径更弯
- 摩擦更大
- 你希望电机推进更有力

这个版本现在只提高电机输出，不改变标准版的触发灵敏性、缓冲阈值和整体动作逻辑。

直观差异只有这 3 个参数：

```cpp
// 标准版
STANDALONE_AUTOLOAD_PWM_PUSH  = 850
STANDALONE_MANUAL_PWM_FEED    = 850
STANDALONE_MANUAL_PWM_RETRACT = 850

// 高阻力版
STANDALONE_AUTOLOAD_PWM_PUSH  = 960
STANDALONE_MANUAL_PWM_FEED    = 950
STANDALONE_MANUAL_PWM_RETRACT = 900
```

## 你需要准备什么

在开始前，你需要：

- `git`
- `python3`
- `platformio`
- 板子对应的刷写工具 `wchisp`

安装 PlatformIO 的常见方式：

```bash
python3 -m pip install -U platformio
```

安装完可以检查一下：

```bash
pio --version
```

## 第一次编译

建议在仓库根目录执行命令。

编译标准版：

```bash
env PLATFORMIO_CORE_DIR=$PWD/.platformio pio run -e filament_buffer_bmcu
```

编译高阻力版：

```bash
env PLATFORMIO_CORE_DIR=$PWD/.platformio pio run -e filament_buffer_bmcu_high_force
```

生成出来的固件在这里：

```text
.pio/build/filament_buffer_bmcu/firmware.bin
.pio/build/filament_buffer_bmcu_high_force/firmware.bin
```

说明：

- 第一次编译时，PlatformIO 会自动下载工具链和框架。
- 这里把 `PLATFORMIO_CORE_DIR` 指到仓库内的 `.platformio/`，是为了把缓存留在项目目录里，避免写到用户主目录。

## 如何刷写到板子

本仓库默认不提交 `wchisp`，所以你需要自己在本地准备它。

先确认板子能被识别：

```bash
./wchisp probe
```

刷写标准版：

```bash
./wchisp flash .pio/build/filament_buffer_bmcu/firmware.bin
```

刷写高阻力版：

```bash
./wchisp flash .pio/build/filament_buffer_bmcu_high_force/firmware.bin
```

如果刷写成功，通常会看到类似输出：

```text
Verify OK
Device reset
```

## 上电后会发生什么

- 开机后会做静默校准。
- 不会再执行旧版那种逐通道等待手动拨动的交互式校准。
- 如果某一路传感器异常，仍然可能通过 LED 表现出来。

也就是说，正常情况下你只需要：

1. 编译一个版本
2. 刷进去
3. 上电测试

## 仓库结构

如果你只是使用固件，这一节可以先跳过。

```text
src/
  app/
    main.cpp
  control/
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

几个最重要的文件是：

- `src/app/main.cpp`
  启动流程和主循环入口。
- `src/control/motion_control.cpp`
  缓冲器主逻辑、电机控制、自动吸料、手动进退料逻辑都在这里。
- `src/control/hall_calibration.cpp`
  上电校准逻辑。

## `reference/original_bmcu` 是什么

这个目录只是原始上游项目的参考副本。

你可以把它理解成：

- 现在这个仓库是“我们真正要用的独立版本”
- `reference/original_bmcu` 是“留着查资料的原项目”

它的作用只是方便对照和参考，不参与当前固件编译，也不是当前功能的一部分。

如果你完全不关心原项目，可以先忽略这个目录。
