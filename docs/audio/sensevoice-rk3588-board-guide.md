# StarryOS SenseVoice RK3588 NPU 上板验证操作手册

> 本手册给出从"拿到板子"到"验证 SENSEVOICE_RKNN_TEST_PASSED + 实时控车"的完整步骤。
> 所有开发环境侧的工作已完成并验证（prebuild、L0、实时模式逻辑、rootfs 注入）。

## 前置条件

- Orange Pi 5 Plus（RK3588），已能从 SD 卡启动 StarryOS（麦克风驱动第一版已收官）
- WSL2 Ubuntu-24.04，仓库在 `/root/work/tgoskits-audio`（分支 `audio/rk3588-capture`）
- 板子与主机在同一网络，或通过 ostool-server 连接

## 第一步：构建完整镜像

在 WSL 仓库目录执行：

```bash
cd /root/work/tgoskits-audio

# 1a. 拉取 base rootfs（Alpine aarch64，~2GB，只需一次）
cargo xtask starry rootfs --arch aarch64

# 1b. 构建内核（含 rknpu + rk3588-audio feature，~40s）
cargo xtask starry build -c apps/starry/sensevoice-rknn/build-aarch64-unknown-none-softfloat.toml

# 1c. 验证 app 被发现
cargo xtask starry app list | grep sensevoice
# 期望输出: board  sensevoice-rknn prebuild
```

## 第二步：准备 L2/L3 参考音频

`sensevoice-rknn-test.sh` 的 L2/L3 需要两个参考音频，放在 `apps/starry/sensevoice-rknn/testwavs/`：

| 文件 | 期望识别文本 | 来源 |
|------|------------|------|
| `test.wav` | `开饭时间早上9点至下午5点` | 已有（ModelScope iic/SenseVoiceSmall 官方 zh.mp3 转换，5.62s 16k/mono/s16） |
| `en.wav` | `the tribal chieftain` | 已有（ModelScope 官方 en.mp3 转换，7.18s 16k/mono/s16） |

`en.wav` 准备方式（任选其一）：
- 从 sherpa-onnx sense-voice 模型包提取（`sherpa-onnx-sense-voice-*.tar.bz2` 内含 test wavs）
- 用板子 Linux 侧 `arecord -D hw:3,0 -f S16_LE -r 16000 -c 1 en.wav` 录制英文朗读
- 用 ffmpeg 转换任意英文音频：`ffmpeg -i input.mp3 -f wav -acodec pcm_s16le -ac 1 -ar 16000 en.wav`

两个参考音频均已从 ModelScope 官方 SenseVoice 仓库的 example/ 目录下载 mp3 并转换为 16k/mono/s16 wav，
prebuild.sh 会自动注入 rootfs 的 /opt/sensevoice/testwavs/。

## 第三步：上板（板子连接 ostool-server）

```bash
# 板子需先启动 ostool-server（参考仓库 board 管理文档）
# 然后构建+上传+运行：
cargo xtask starry app board \
  --test-case sensevoice-rknn \
  --board-type OrangePi-5-Plus \
  --server <板子IP> --port <端口>
```

构建系统会自动：
1. 跑 `prebuild.sh`（装 python3+numpy、复制 librknnrt.so、下载模型、生成 tokens.txt）
2. 注入 overlay 到 rootfs 镜像
3. 上传到板子并启动 StarryOS
4. 执行 `init.sh` → `sensevoice-rknn-test.sh`（L0-L3）

**期望输出**：
```
[perf] model load: X.XXs
SENSEVOICE_RKNN_TEST_PASSED
```

## 第四步：实时语音控制（L4）

L0-L3 通过后，进入 StarryOS shell 手动启动实时模式：

```bash
# 在板子串口 shell 里
export LD_LIBRARY_PATH=/opt/sensevoice/lib:/usr/lib:/lib
/opt/sensevoice/bin/python3 /opt/sensevoice/python/sensevoice_rknn_npu.py --live
```

对着麦克风说指令词：
- 前进 / 向前 / 往前
- 后退 / 向后 / 往后
- 左转 / 向左 / 往左
- 右转 / 向右 / 往右
- 停止 / 停下 / 停

识别成功会经 `/dev/console` 发 `@@RT <command>` 指令给 RT 控车。

## 故障排查

### VAD 灵敏度（板端话筒模拟支路灵敏度低，见项目记录）
```bash
# 调低阈值（默认 0.012，板子话筒可试 0.006）
SENSEVOICE_VAD_THRESHOLD=0.006 \
  /opt/sensevoice/bin/python3 /opt/sensevoice/python/sensevoice_rknn_npu.py --live
```

### NPU 推理报错
- 确认 `/dev/dri/card1` 存在（`ls -l /dev/dri/card1`），NPU 驱动已加载
- 确认 `librknnrt.so` 版本与 NPU 驱动匹配（当前复用 uvc-rknn 的 2.x 版本）

### 模型加载失败
- 检查 `/opt/sensevoice/model/` 下 6 个文件齐全
- `sense-voice-encoder.rk3588.fp16-scaled.rknn` 必须为 **490649722 字节**（~468MiB），
  sha256 `00978fd943e73f29feb58f1ed162f2d46cc27a29c4320d93955e4d26d2ac3c1d`。
  文件大小对不上就是下载被打断留下的残件，必须重下 —— 截断文件依然有合法 `RKNN` magic，
  只有 `rknn_init` 会晚到地报 `-6`：
  `E RKNN: parseRKNN: exportDataSize large then model size: 490648640 vs <实际>!`
  （2026-08-29 上板实测踩到：缓存里是 352063735 字节的残件，占完整长度 71.75%。）

### python3 / numpy 找不到
- `LD_LIBRARY_PATH` 必须含 `/opt/sensevoice/lib:/usr/lib:/lib`（Alpine musl）
- 确认 `/usr/bin/python3` 存在且可执行

## 已验证项（开发环境，无需板子）

| 项 | 验证方式 | 结果 |
|----|---------|------|
| app 被构建系统发现 | `cargo xtask starry app list` | ✓ board sensevoice-rknn prebuild |
| 内核编译（rknpu+audio feature） | `cargo xtask starry build` | ✓ 14MB starryos.bin |
| prebuild 脚本 | qemu-user apk 装 python3.12+numpy 2.3.5 | ✓ 298MiB/65 包 |
| L0（python+numpy+ctypes+脚本import） | qemu-user | ✓ --help 正常 |
| 实时模式逻辑（重采样+VAD+指令） | qemu-user | ✓ 480→160样本、VAD检出utterance |
| rootfs overlay 注入 | debugfs 注入 2626 文件 | ✓ /opt/sensevoice 全齐 |
| L2/L3 参考音频实际识别内容 | CPU 版 FunASR SenseVoiceSmall ONNX | ✓ test.wav→开饭时间早上9点至下午5点；en.wav→The tribal chieftain...（test.sh 期望文本已对齐） |
| NPU 脚本前端管线（fbank→LFR→CMVN→encoder input） | verify_frontend.py 纯 numpy | ✓ test.wav→[1,98,560]；en.wav→[1,124,560]；CMVN 后 ±3 量级、有限 |

## 待板子验证项

| 项 | 说明 |
|----|------|
| L1 缺模型报错 | 真机 shell，无 NPU 依赖 |
| L2 中文 wav NPU 识别 | 需真 NPU + test.wav |
| L3 英文 wav NPU 识别 | 需真 NPU + en.wav |
| L4 实时 VAD+ASR+控车 | 需真 NPU + 麦克风 |
## L2/L3 期望文本调整说明

`test.sh` 在 grep 匹配期望文本**之前**会先打印实际识别结果（`L2 transcript: ...` / `L3 transcript: ...`）。

- 如果首次上板 L2/L3 报 "transcript mismatch"，先看打印的实际识别结果。
- L2 的 test.wav 来自 ModelScope 官方 zh.mp3，内容应为"开饭时间早上9点至下午5点"（ITN 规范化后阿拉伯数字），与 test.sh 的 grep 期望已对齐。
- L3 的 en.wav 来自 ModelScope 官方 en.mp3，期望文本"the tribal chieftain"经 CPU 版 FunASR 验证确认为实际识别内容的前缀（完整为"The tribal chieftain called for the boy and presented him with 50 pieces of gold."），test.sh 用 `grep -qi`（忽略大小写）匹配，已对齐。
- 这是上板首次验证时的预期调试步骤，不是代码缺陷。