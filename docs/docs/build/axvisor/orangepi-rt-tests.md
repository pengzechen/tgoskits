---
sidebar_position: 5
sidebar_label: "OrangePi RT 测试"
---

# OrangePi Axvisor RT 测试构建

本文记录 OrangePi-5-Plus 上 Axvisor reserved-core RT 测试镜像的构建和 FIT/ITB 打包流程。相关测试程序都编译进 `os/axvisor`，由 board build config 选择不同 feature；最终生成的 ELF 位于 `target/aarch64-unknown-linux-musl/release/axvisor`，再通过 `rust-objcopy` 和 `mkimage` 转换为可由 U-Boot 加载的 `tmp/orangepi-rt-fit/axvisor.itb`。

## 1. 测试镜像

RT 测试镜像复用同一套 OrangePi-5-Plus 启动和 I2C/UART bring-up 代码，但每个 board config 只选择一个面向硬件的测试场景。`os/axvisor/src/realtime.rs` 负责把 feature-gated RT task 加入 `RT_TASKS`，`os/axvisor/src/i2c_rt.rs` 和 `os/axvisor/src/uart_rt.rs` 负责具体 I2C5、UART7 的 MMIO 初始化和轮询传输。

### 1.1 配置选择

这些配置文件都位于 `os/axvisor/configs/board/`。它们共同设置 `realtime`、`rt-selftest`、`AX_RT_CPU = "7"` 和 `SMP = "8"`，差异在于启用哪个硬件 RT task。

| 测试场景 | Board config | 关键 feature | 主要代码入口 |
| --- | --- | --- | --- |
| LU9685 I2C 舵机 | `orangepi-5-plus-rt-sd-i2c.toml` | `rt-i2c` | `i2c_rt::i2c_servo_task()` |
| LU9685 UART 舵机 | `orangepi-5-plus-rt-sd-uart.toml` | `rt-uart` | `uart_rt::uart_task()` |
| MPU6050 I2C 采样 | `orangepi-5-plus-rt-sd-mpu6050.toml` | `rt-mpu6050` | `i2c_rt::i2c_mpu6050_task()` |

`rt-mpu6050` 依赖 `rt-i2c` 来复用 I2C5 host-side bring-up，但 `realtime.rs` 在启用 `rt-mpu6050` 时不会再加入 LU9685 servo task。这样 MPU6050-only 镜像只采样 MPU6050，不会同时向 LU9685 地址发包并产生干扰日志。

### 1.2 引脚约束

OrangePi-5-Plus 40-pin header 上 I2C5 使用 `GPIO1_B6/B7`（物理引脚 27/28），UART7 使用 `GPIO1_B4/B5`（物理引脚 24/26），两路是独立的 pin group，不共享引脚。I2C5 使用 `GPIO1_B6` 作为 SCL、`GPIO1_B7` 作为 SDA；UART7 使用 `GPIO1_B4` 作为 RX、`GPIO1_B5` 作为 TX。

| 信号 | I2C5 测试 | UART7 测试 |
| --- | --- | --- |
| `GPIO1_B6` (pin 28) | SCL | - |
| `GPIO1_B7` (pin 27) | SDA | - |
| `GPIO1_B4` (pin 24) | - | UART7 RX |
| `GPIO1_B5` (pin 26) | - | UART7 TX |
| GND | 外设公共地 | 外设公共地 |
| 3V3 | MPU6050 VCC | 不适用于 UART 信号本身 |

I2C 设备可以共享同一 I2C5 总线，但地址必须不冲突。当前 LU9685 I2C 地址为 `0x00`，MPU6050 探测 `0x68` 和 `0x69`，地址上可以共存；不过调试单个外设时建议使用对应的 single-task config，减少日志和总线访问干扰。

## 2. 编译流程

Axvisor RT 测试镜像必须从 `os/axvisor` 目录执行 `cargo xtask build --config ...`。这个命令会读取 board config，设置 `AX_TARGET`、`AX_RT_CPU`、`SMP` 和 Cargo features，并输出适合后续打包的 Axvisor ELF。

### 2.1 LU9685 I2C

LU9685 I2C 镜像启用 `rt-i2c`，在 host side 初始化 RK3588 I2C5 的 CRU、IOC pinmux、pull-up 和 controller，然后 RT core 周期性写入 LU9685 的 `[channel, angle]` 协议。构建命令如下。

```bash
cd os/axvisor
cargo xtask build --config configs/board/orangepi-5-plus-rt-sd-i2c.toml
```

构建成功后，ELF 仍然固定输出到 workspace 的 `target/aarch64-unknown-linux-musl/release/axvisor`。如果后续马上运行打包命令，生成的 `axvisor.itb` 就是 LU9685 I2C 测试版本。

### 2.2 LU9685 UART

LU9685 UART 镜像启用 `rt-uart`，使用 RK3588 UART7 和 LU9685 的 `FA address channel angle FE` 串口协议。UART7 的 TX 位于 `GPIO1_B5`（物理引脚 26），接到 LU9685 的 UART RX；`GPIO1_B4`（物理引脚 24）作为 RX 预留，当前 TX-only 协议不读取。该镜像与 I2C5 镜像使用不同的 header pin group，接线时按各自协议的引脚走，不要接错。

```bash
cd os/axvisor
cargo xtask build --config configs/board/orangepi-5-plus-rt-sd-uart.toml
```

该命令会覆盖同一个 `target/aarch64-unknown-linux-musl/release/axvisor` 产物。每次切换测试程序后，都需要重新执行 ITB 打包步骤，避免把上一个测试程序的 ELF 烧写到板子上。

### 2.3 MPU6050 I2C

MPU6050 镜像启用 `rt-mpu6050`，会在 I2C5 上探测 `0x68` 和 `0x69`，读取 `WHO_AM_I`，然后按 ESP32 standalone test 的寄存器配置初始化 MPU6050。RT task 每 `100 ms` 从 `ACCEL_XOUT_H` 连续读取 14 字节，并输出 raw 和缩放后的观察值。

```bash
cd os/axvisor
cargo xtask build --config configs/board/orangepi-5-plus-rt-sd-mpu6050.toml
```

运行时关键日志来自 `i2c_rt::report_mpu6050_sample()`。输出会分为 `acc_raw[3]`、`acc_mg[3]`、`gyro_raw[3]` 和 `gyro_dps_x100[3]`，其中 `acc_mg` 的 `1000` 表示 `1g`，`gyro_dps_x100` 的 `100` 表示 `1 deg/s`。

## 3. ITB 打包

`cargo xtask build` 只保证生成 Axvisor ELF；OrangePi U-Boot 需要加载 FIT image，所以还要把 ELF 转成裸二进制并用 `tmp/orangepi-rt-fit/axvisor.its` 生成 `axvisor.itb`。这一步必须在每次切换测试配置并重新编译后执行。

### 3.1 打包输入

FIT 打包目录固定为 `tmp/orangepi-rt-fit/`。`axvisor.its` 描述 kernel 和 FDT 的 load/entry 地址，`orangepi-5-plus.dtb` 是随 ITB 一起封装的设备树，`axvisor.bin` 是从最新 ELF 转换出来的裸二进制。

| 文件 | 作用 | 维护注意点 |
| --- | --- | --- |
| `target/aarch64-unknown-linux-musl/release/axvisor` | 最新 Axvisor ELF | 由最后一次 `cargo xtask build --config ...` 生成 |
| `tmp/orangepi-rt-fit/axvisor.bin` | FIT kernel payload | 由 `rust-objcopy` 覆盖生成 |
| `tmp/orangepi-rt-fit/axvisor.its` | FIT image source | 保存 load/entry、FDT 路径和镜像描述 |
| `tmp/orangepi-rt-fit/orangepi-5-plus.dtb` | OrangePi DTB | 被 `axvisor.its` 引用 |
| `tmp/orangepi-rt-fit/axvisor.itb` | 最终烧写/加载产物 | 由 `mkimage` 覆盖生成 |

当前 `axvisor.its` 使用的 Axvisor kernel load/entry 地址是 `0x02080000`，FDT load 地址是 `0x0a100000`。如果修改 U-Boot 环境或内存布局，需要同步检查这些地址是否仍然和板卡启动脚本一致。

### 3.2 生成命令

以下命令从 workspace 根目录执行，先把最新 ELF 转为 `axvisor.bin`，再调用 `mkimage` 生成 `axvisor.itb`。命令本身不区分 I2C、UART 或 MPU6050；最终包含哪个测试程序只取决于前一次 `cargo xtask build --config ...` 使用的 board config。

```bash
rust-objcopy -O binary --strip-all \
  target/aarch64-unknown-linux-musl/release/axvisor \
  tmp/orangepi-rt-fit/axvisor.bin

mkimage -f \
  tmp/orangepi-rt-fit/axvisor.its \
  tmp/orangepi-rt-fit/axvisor.itb
```

如果你习惯停留在 `os/axvisor` 目录，也可以使用相对路径访问 workspace 根目录下的产物。两种写法等价，但推荐从 workspace 根执行，减少路径层级错误。

```bash
rust-objcopy -O binary --strip-all \
  ../../target/aarch64-unknown-linux-musl/release/axvisor \
  ../../tmp/orangepi-rt-fit/axvisor.bin

mkimage -f \
  ../../tmp/orangepi-rt-fit/axvisor.its \
  ../../tmp/orangepi-rt-fit/axvisor.itb
```

`mkimage` 成功时会打印 FIT description、kernel data size、architecture、load address、entry point 和 FDT load address。确认输出里 `Load Address` 和 `Entry Point` 都是 `0x02080000`，并且生成时间晚于刚才的构建时间。

### 3.3 一步构建

下面的命令把 MPU6050 构建和 ITB 打包连起来，适合每次修改 `i2c_rt.rs` 或输出格式后快速生成可烧写镜像。要切换到 LU9685 I2C 或 UART，只需要替换第一行 build config，后两步保持不变。

```bash
cd os/axvisor
cargo xtask build --config configs/board/orangepi-5-plus-rt-sd-mpu6050.toml
rust-objcopy -O binary --strip-all \
  ../../target/aarch64-unknown-linux-musl/release/axvisor \
  ../../tmp/orangepi-rt-fit/axvisor.bin
mkimage -f \
  ../../tmp/orangepi-rt-fit/axvisor.its \
  ../../tmp/orangepi-rt-fit/axvisor.itb
```

打包完成后，`tmp/orangepi-rt-fit/axvisor.itb` 就是要给 U-Boot 加载的文件。这个文件会被每次打包覆盖，因此记录或传输镜像前应确认最后一次构建使用的是目标测试配置。

## 4. 验证日志

板卡启动后，RT task 通过 `ax_rt::rt_output_write()` 写入 RT output ring，host shell 或串口日志中会出现对应测试程序的输出。观察日志时先确认只有期望的 task 在运行；如果 MPU6050-only 镜像仍出现 LU9685 行，说明烧写的不是最新 `orangepi-5-plus-rt-sd-mpu6050.toml` 构建出的 ITB。

### 4.1 LU9685 输出

LU9685 I2C 成功时会周期性打印当前物理角度和 LU9685 raw angle。失败时 `report_servo_failure()` 会打印 `CON`、`IPD`、`CLKDIV` 和 `IEN`，用于判断是 NACK、START/STOP timeout，还是 controller 状态异常。

```text
RT i2c5 LU9685@I2C5 set physical=60 raw=40 addr=0x00
RT i2c5 LU9685 write FAIL timeout physical=66 raw=44 CON=... IPD=... CLKDIV=... IEN=...
```

UART 版本成功控制舵机时不会走 I2C5 failure path；如果同一轮测试中看到 I2C5 和 UART7 输出混在一起，应回到 board config 检查 feature 是否误合并。

### 4.2 MPU6050 输出

MPU6050 初始化成功后先打印初始化参数，然后每个采样周期输出三轴 raw 值和换算值。`acc_mg[3]` 用于判断重力落在哪个芯片轴上，静止水平时某一个轴通常接近 `+1000` 或 `-1000`；`gyro_dps_x100[3]` 用于观察转动角速度，静止时应接近 `0`，较大的稳定值表示零偏或读数异常。

```text
RT i2c5 MPU6050@0x68 initialized: sample=100Hz dlpf=44Hz gyro=+/-250dps accel=+/-2g
RT i2c5 MPU6050@0x68
  acc_raw[3]  x=-15522 y=-2638 z=-1234
  acc_mg[3]   x=-947 y=-161 z=-75
  gyro_raw[3] x=-483 y=523 z=292
  gyro_dps_x100[3] x=-368 y=399 z=222 temp_raw=-1012
```

如果 `acc_mg[3]` 显示重力主要在 `x` 轴而不是 `z` 轴，说明当前模块摆放方向下芯片坐标的 X 轴接近竖直方向。判断 roll、pitch、yaw 前应先用静止状态确认三轴和板子长边、短边、法线方向的实际对应关系。
