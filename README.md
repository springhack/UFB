# filament buffer bmcu

## 目标

这个工程是一个不接打印机总线的独立耗材缓冲器固件，只保留霍尔缓冲、电机控制、必要校准和本地存储。

当前行为：

- 平时作为离线缓冲器运行，不参与任何打印机通信。
- 插入新料后自动吸入，把缓冲器拉回平衡区。
- 空通道时，拉住或推住缓冲器超过 `1s`，进入手动连续进料或退料；松手停止。
- 已经有料时，禁用这套手动连续进退料触发，避免打印时误动作。

## 代码结构

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

模块职责：

- `app/main.cpp`
  负责启动流程、RGB 初始化、校准启动、主循环。
- `control/motion_control.*`
  负责缓冲器主状态机、电机控制、霍尔逻辑、自动吸料、空通道手动模式。
- `control/hall_calibration.*`
  负责上电校准和霍尔中心点保存。
- `hardware/adc_dma.*`
  负责 ADC 采样。
- `hardware/as5600_multi_soft_i2c.*`
  负责 4 路 AS5600 读取。
- `hardware/ws2812.*`
  负责 LED。
- `model/filament_state.*`
  保留运行时通道状态和每路耗材状态。
- `storage/nvm_storage.*`
  负责 Flash 中的校准参数、运动参数和通道状态保存。
- `platform/*`
  放底层时间、IRQ 和可选调试接口。
- `reference/original_bmcu`
  作为原始上游仓库参考，不参与当前固件构建。

## 依赖安装

需要的软件：

- `git`
- `python3`
- `platformio`

推荐安装方式：

```bash
python3 -m pip install -U platformio
```

验证：

```bash
pio --version
```

## 编译模式

- `filament_buffer_bmcu`
  默认版本，适合常规阻力路径。

- `filament_buffer_bmcu_high_force`
  高阻力版本，适合更长、更弯、摩擦更大的 PTFE 路径。
  这个模式会启用更激进的送料参数，并把独立缓冲器的补料/手动进料 PWM 一并提高。

## 编译方法

默认版本：

```bash
env PLATFORMIO_CORE_DIR=$PWD/.platformio pio run -e filament_buffer_bmcu
```

高阻力版本：

```bash
env PLATFORMIO_CORE_DIR=$PWD/.platformio pio run -e filament_buffer_bmcu_high_force
```

编译产物：

```text
.pio/build/filament_buffer_bmcu/firmware.bin
.pio/build/filament_buffer_bmcu_high_force/firmware.bin
```

第一次编译时，PlatformIO 会自动下载：

- `platform-ch32v`
- `framework-wch-noneos-sdk`
- `toolchain-riscv`

如果不想把缓存写到用户主目录，可以像上面的命令一样把 `PLATFORMIO_CORE_DIR` 指到仓库内的 `.platformio/`。

## 刷写方法

本地刷写工具 `wchisp` 被加入了 `.gitignore`，不会提交到仓库。

先探测设备：

```bash
./wchisp probe
```

刷写默认版本：

```bash
./wchisp flash .pio/build/filament_buffer_bmcu/firmware.bin
```

刷写高阻力版本：

```bash
./wchisp flash .pio/build/filament_buffer_bmcu_high_force/firmware.bin
```

如果设备能被识别，刷写结束会看到：

```text
Verify OK
Device reset
```

## 首次上电说明

- 上电会做静默校准，不再执行旧版本那种逐通道等待手动拨动的交互校准。
- 如果某一路 AS5600 或霍尔状态异常，仍然可能通过 LED 报错。
- 正常工作时，这个固件完全不依赖打印机连接。

## 参考仓库

原始仓库会作为子模块放在：

```text
reference/original_bmcu
```

这个目录只用于参考，不参与当前独立缓冲器固件的实现和编译。
