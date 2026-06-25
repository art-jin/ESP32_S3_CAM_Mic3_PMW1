# ESP32-S3-CAM × 3DMIC-291 六向声源定位

360° 六方向（0°/60°/120°/180°/240°/300°）声源定位，跑在 GOOUUU ESP32-S3-CAM + 3DMIC-291 三麦 MEMS 阵列上。算法用 1024 点 FFT + GCC-PHAT + 三麦几何求解，输出通过 UART 报告为钟点方向（"声源位于 6 点方向"）。

## 项目状态

**已跑通并标定（v1.3）**。前置预加重滤波器让**正常说话音量**就能驱动 3-mic 定位（之前只有吹哨/大声才有效）。

| 用户位置 | 期望 α | 实测 α | 偏差 | 模式 |
|---|---|---|---|---|
| 6 点钟（M3 后方）| 180° | 184.1° | +4.1° | 3-mic |
| 12 点钟（正北）| 0° | 6.6° | +6.6° | 3-mic |
| 3 点钟（正东）| 90° | 83.9° | -6.1° | 3-mic |
| 10 点钟（M2 后方）| 300° | 291.3° | -8.7° | 3-mic + 2-mic 混合 |
| 9 点钟（M1 对侧盲区）| 270° | 306° | +36° | 2-mic fallback |

**9 点钟是 3 麦阵列的几何盲区**：用户正好在 M1（2oc）的对侧，M1 信号弱导致 L/R 折叠，算法降级到 2-mic 半平面报告（"M2 side"——9oc 确实在 M2-M3 基线的 M2 半平面）。这是任何 3 麦平面阵列的固有限制，不是 bug。

单帧方位噪声约 ±15°，`stable_sextant`（3 帧滞后）锁定后不抖。

舵机驱动（GPIO38 + JS6620 + 15T/50T 内齿圆盘减速）**v1.3 完成**——`stable_sextant` 驱动舵机让 M3 跟踪声源，**feed-forward 补偿**消除闭环振荡，**前置预加重**让语音也能驱动跟踪。

**性能**（v1.2，270° 舵机 + feed-forward）：

| 测试位置 | 响应时间 | 最终 servo | 状态 |
|---|---|---|---|
| 6 点钟（home）| 不动 | 0° | ✓ 完美稳定（swing=0°）|
| 6→7 点钟切换 | **0.5 秒** | **+30°** | ✓ 锁定，零反弹 |
| 6→5 点钟切换 | **3.9 秒** | **-30°** | ✓ 锁定，零反弹 |
| 3 / 9 点钟（超范围）| 抑制 | 保持上次 | ✓ 不振荡 |

**可跟踪弧**：M3 在 **4:54 → 7:06 点钟**之间（±33°，2 小时 12 分弧）。

**关键算法**：`α_room = α_array + β_servo` ——把声源方位从阵列坐标系换算到房间坐标系，让 target 不随舵机转动变化。详见 `CLAUDE.md` Pitfalls §6。

**已知限制**：
- ±33° 是软件软限位（机械极限 ±40.5°，留 7.5° 安全余量）
- 没有 IMU 时，feed-forward 是最佳算法（不需要硬件改动）
- Phase 4 UART 调试因硬件限制（CH343 单向）未完成

开发历史见 `SERVO_PLAN.md`。

**关键算法**：`α_room = α_array + β_servo` ——把声源方位从阵列坐标系换算到房间坐标系，让 target 不随舵机转动变化。详见 `CLAUDE.md` Pitfalls §6。

**已知限制**：
- ±20° 是软件软限位（机械极限 ±27°，但止位震动会触发 L/R 折叠反馈）
- 没有 IMU 时，feed-forward 是最佳算法（不需要硬件改动）
- Phase 4 UART 调试因硬件限制（CH343 单向）未完成

开发历史见 `SERVO_PLAN.md`。

## 关键发现

C3 和 S3_CAM_MIC3 两个同类项目都把 DAT0 上 L/R 时隙的"折叠"当成了硬件限制，放弃了 3-mic 定位。**这个判断是错的**——L/R 折叠是 SNR 条件性的，用户真实语音下 PDM2PCM 会自动分离 L/R 槽，3-mic 360° 定位可行。

详细对比和历史见 `CLAUDE.md` 的 "Pitfalls encountered" 一节，以及 `TECH_NOTE_for_S3_CAM_MIC3.md`。

## 硬件接线

| 3DMIC-291 | ESP32-S3-CAM GPIO | 角色 |
|---|---|---|
| 3.3V | 3v3 | 供电 |
| GND | GND | 地 |
| CLK0 | GPIO 1 | I²S PDM RX CLK |
| DAT0 | GPIO 2 | I²S PDM RX DIN[0] — M2 (L slot) + M1 (R slot) |
| CLK1 | GPIO 14 | GPIO 矩阵扇出的同一 I²S CLK 副本 |
| DAT1 | GPIO 42 | I²S PDM RX DIN[1] — M3 |
| JS6620 舵机 PWM | GPIO 38 | LEDC 50 Hz PWM（计划中，详见 `SERVO_PLAN.md`）|

> ⚠️ S3 的 I²S PDM RX 外设只暴露一个 CLK 输出。CLK1 必须通过 GPIO 矩阵把 I²S0 RX WS 信号（`I2S0I_WS_OUT_IDX`）复制到 GPIO14，否则两颗时钟各自漂移，DOA 失效。实现见 `main/mic_capture.c`。

麦克风几何（等边三角形，边长 10 mm，以板心为原点）：

```
    M2 (c0) @ 10oc        12oc (north, α=0°)
       ●
      /   \                ↗ 0°
     /     \             ↑
    /       \            |
   ●─────────●           +────→ east
 M3 (c2) @ 6oc    M1 (c1) @ 2oc
```

> **板子方向**：3DMIC-291 PCB **翻面安装**（芯片朝下，集音孔朝上）。这把布局沿 12oc-6oc 轴镜像：丝印上的 M1（10oc）现在物理位置在 2oc，丝印上的 M2（2oc）现在在 10oc，M3 不动。如果你把板子装成芯片朝上，需要把 `doa.c` 里 `sin_a = sm_01 / DOA_K` 改回负号。

> 通道到物理麦的映射是 tap test 实测得出的（2026-06-21），不是按 README 猜的。如果你想移植到别的板子，**务必重做 tap test**——参见 `CLAUDE.md` 的校准章节。

## 工具链

- ESP-IDF v6.0.1（`~/.espressif/v6.0.1/esp-idf`）
- 目标：`esp32s3`
- CMake + Ninja
- 编译器：`xtensa-esp32-elf-gcc`

## 构建 & 烧录

```bash
# 1. source IDF 环境
. ~/.espressif/v6.0.1/esp-idf/export.sh

# 2. 编译
idf.py build

# 3. 进 bootloader（GOOUUU 板的自动复位电路在 CH343 上不工作）
#    - 按住 BOOT
#    - 按一下 RST 并松开
#    - 松开 BOOT

# 4. 烧录 + 监视器
idf.py -p /dev/cu.usbmodem21201 flash monitor
# 退出 monitor: Ctrl-]
```

烧录后 USB-CDC 设备名通常是 `/dev/cu.usbmodem21201`（CH343 USB-UART）。

## 输出格式

板子启动后每 500 ms 打印一行 DOA 结果：

```
>>> 3-MIC  az=181.8°  sect=3 ( 6 o'clock)  stable=3 ( 6 o'clock)  conf=0.47
          lag01=+0.05 lag02=+1.01 lag12=+0.85  | ac 70/73/61  ρ01=0.83 peak01=0.59
```

每 5 秒打印一次模式直方图：

```
[5s] 46 frames: 3-mic=36 (78%)  2-mic=10 (22%)  bad=0
```

字段含义：
- **`az`** — 方位角 0..360°，0° = 12 点钟（正北），顺时针增加
- **`sect`** — 6 个扇区索引：0=12oc, 1=2oc, 2=4oc, 3=6oc, 4=8oc, 5=10oc
- **`stable_sextant`** — 3 帧一致才更新的滞后版本，**实际使用时看这个**
- **`conf`** — GCC-PHAT 峰值平均，0..1
- **`lag01/02/12`** — 三对麦的 TDOA（采样点，K=1.4 为物理上限）
- **`ac`** — 三通道 AC RMS（LSB），反映信号强度
- **`ρ01`** — c0 和 c1 的 Pearson 互相关，>0.95 表示 L/R 折叠
- **`peak01`** — c0 vs c1 的 GCC-PHAT 峰值

## 代码结构

```
main/
├── main.c           FreeRTOS mic 任务：capture → deinterleave → DOA → UART log
├── mic_capture.{c,h}  I²S PDM RX 多-DIN 初始化 + 阻塞读取 + GPIO-matrix CLK 扇出
├── doa.{c,h}        纯算法：FFT、GCC-PHAT、3-mic / 2-mic 几何求解、输出滞后
└── CMakeLists.txt   idf_component_register(SRCS main.c mic_capture.c doa.c ...)
```

- 音频任务在 `configMAX_PRIORITIES - 2`，8 KB 栈
- 窗口：100 ms @ 48 kHz × 3 ch × int16 = 28 800 bytes / DMA read
- FFT 1024 点（~21 ms 窗口）
- K = d·fs/c ≈ 1.40 samples（边长 10 mm，48 kHz，声速 343 m/s）

## 算法概览

**Pairwise GCC-PHAT**（符号约定见 CLAUDE.md 的 pitfalls，坑过一次）：

```
lag_01 = gcc_phat(c0,c1) = arrival(M2) − arrival(M1) = +K·sin(α)
lag_02 = gcc_phat(c0,c2) = arrival(M2) − arrival(M3) = +K·sin(α−60°)
lag_12 = gcc_phat(c1,c2) = arrival(M1) − arrival(M3) = −K·sin(α+60°)
```

**3-mic 几何求解**：

```
sin α = +lag_01 / K              # 正号——因为板子翻面安装，M1/M2 位置镜像
cos α = (lag_01 − 2·lag_02) / (K·√3)
α = atan2(sin α, cos α)  →  0..360°
```

**2-mic fallback**（DAT0 L/R 折叠时）：

只用 M2 和 M3，输出半平面方位 [0°, 180°]，前后方向有歧义，分 3 个 bin。

**质量门**：
- ρ01 < 0.95（L/R 未折叠）才考虑 3-mic 模式
- peak_01 ≥ 0.40 且 peak_02 ≥ 0.40 才接受 3-mic 结果
- peak_02 ≥ 0.30 才接受 2-mic 结果
- 3 帧连续相同 raw sextant 才更新 `stable_sextant`

## 进一步阅读

- **`CLAUDE.md`** — 给 AI 协作者的完整项目文档。包括所有踩过的坑（4 个）、几何方程推导、标定步骤、舵机硬件规格、参考项目对比。
- **`TECH_NOTE_for_S3_CAM_MIC3.md`** — 写给同类项目 S3_CAM_MIC3 的移植建议（GPIO 接线、算法补丁、校准步骤）。
- **`SERVO_PLAN.md`** — 舵机声源跟踪的开发计划（硬件约束、软件架构、噪音抑制策略、分阶段实施）。

## 相关项目

- [`ESP32_C3_Mic3`](../ESP32_C3_Mic3) — 同一目标在 ESP32-C3 上的尝试。C3 的 I²S PDM RX 只支持 1 个数据线，L/R 折叠是真正的硬件天花板。
- [`ESP32_S3_CAM_MIC3`](../ESP32_S3_CAM_MIC3) — S3 + 3DMIC-291 + 摄像头 + 舵机。同样误判 L/R 折叠为硬件限制，只实现了 2-mic 1D DOA。本项目推翻了这个结论并跑通了 6 向 360° 定位。

## 许可

本项目采用 [Apache License 2.0](LICENSE)。
