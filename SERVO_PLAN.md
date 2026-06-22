# 舵机声源跟踪开发计划

> **状态**：计划已定稿，待实施
> **日期**：2026-06-22
> **目标**：驱动 GPIO38 上的 JS6620 舵机，让 3 麦阵列的 **6 点钟方向（M3 麦）** 尽可能指向当前说话的声源。算法和硬件限制下容忍延迟与误差，详见下文。

## 1. 目标与约束

### 1.1 核心目标

**输入**：`doa_result_t.stable_sextant`（已 3 帧滞后平滑的扇区号 0..5）+ `azimuth_deg`
**输出**：舵机 PWM 脉宽（500–2500 µs）
**反馈环**：舵机转动改变麦阵方向 → DOA 读到的 α 也跟着变 → 舵机再次微调

终极目标：用户绕着板子走，舵机跟随，**阵列的 M3 始终大致朝向用户嘴巴**。

### 1.2 硬件约束（来自 CLAUDE.md "Servo hardware"）

| 项 | 值 |
|---|---|
| 舵机型号 | JS6620 标准舵机（~180° 行程） |
| 驱动齿轮 | 15 齿外齿 |
| 圆盘齿轮 | 50 齿内齿（内啮合） |
| 减速比 | 50 / 15 = **3.33 : 1** |
| 圆盘有效行程 | 180° / 3.33 = **~54°** |
| 机械跟踪范围 | ±27°（绕"home"位置） |
| 安装位置 | 阵列 12oc 下方 12 cm |

### 1.3 关键限制

**这不是一个 360° 跟踪系统。** 54° 行程 < 60° 一个扇区。系统只能在物理限制内做**局部微调跟踪**。

三种策略可选（实施时选 A，预留升级到 C 的接口）：

| 策略 | 描述 | home 位置 | 覆盖范围 |
|---|---|---|---|
| **A. 固定 home + 微调（推荐）** | home 设为用户最常出现的方向（如 6oc），舵机在 ±27° 内微调 | 阵列 6oc | 声源 α ∈ [153°, 207°] |
| B. 扇区中心 home | home 跟着 `stable_sextant` 跳（粗调），舵机在扇区内微调 | 最后锁定扇区中心 | 整圈但每次切换 home 要人工干预 |
| C. 双舵机或连续旋转 | 换连续旋转舵机或加第二轴 | — | 真正 360°，需改硬件 |

策略 A 是当前硬件下最实用、最简单的方案。下面所有阶段都基于 A。

### 1.4 性能预期（实话实说）

| 指标 | 预期值 | 原因 |
|---|---|---|
| 跟踪延迟 | ~1.5 秒 | `stable_sextant` 需要 3 帧 × 500 ms |
| 静态指向精度 | ±15° | 单帧 DOA 噪声 + 舵机死区 |
| 动态响应 | 慢，用户走动时滞后明显 | hysteresis + 舵机机械时间常数 |
| 声源范围 | home ± 27°（约 54° 弧） | 齿轮减速机械限制 |
| 范围外行为 | 钳制到最近极限 | 舵机不能硬推过冲 |

**结论**：适合"用户坐着说话"的场景，不适合"绕房间走动"的场景。

## 2. 软件架构

### 2.1 新增模块

```
main/
├── servo.{c,h}          [新] LEDC PWM 驱动 + 角度到脉宽映射
├── tracker.{c,h}        [新] DOA → 舵机目标角度的策略层
└── main.c               [改] 集成 tracker，处理 motion-pause 逻辑
```

`doa.{c,h}` 和 `mic_capture.{c,h}` **不动**——保持算法与硬件解耦。

### 2.2 数据流

```
  I²S DMA ─→ mic_capture ─→ deinterleave ─→ doa_process
                                                │
                                                ▼
                                         doa_result_t
                                         (az, stable_sextant)
                                                │
                                                ▼
                                       tracker_update()
                                                │
                                  ┌─────────────┴────────────┐
                                  ▼                          ▼
                          motion-pause gate         target_angle = α - 180°
                          (suppress DOA              clamp to ±27°
                           during motion)
                                  │                          │
                                  └─────────────┬────────────┘
                                                ▼
                                        servo_set_angle()
                                                │
                                                ▼
                                         LEDC PWM out
                                                │
                                                ▼
                                            GPIO38
```

### 2.3 模块接口（草案）

**`servo.h`** — 纯驱动层，不涉及跟踪逻辑：

```c
#define SERVO_GPIO             GPIO_NUM_38
#define SERVO_PWM_FREQ_HZ      50
#define SERVO_PULSE_MIN_US     500     /* JS6620 满行程左极限 */
#define SERVO_PULSE_MAX_US     2500    /* JS6620 满行程右极限 */
#define SERVO_PULSE_CENTER_US  1500    /* home 位置 */

/* 机械限位：±27°（见 SERVO_PLAN §1.2）。超出会被 clamp。 */
#define SERVO_ANGLE_MIN_DEG    -27.0f
#define SERVO_ANGLE_MAX_DEG    +27.0f

esp_err_t servo_init(void);

/* 直接设脉宽，单位 µs。自动 clamp 到 [500, 2500]。 */
void servo_set_pulse_us(uint32_t us);

/* 设角度（°），0 = home（圆盘 home 位置），正值顺时针。自动 clamp。 */
void servo_set_angle_deg(float angle_deg);

/* 查询当前目标角度（运动完成后的稳态值）。 */
float servo_get_angle_deg(void);

/* 查询舵机是否在运动中（用于 tracker 的 motion-pause）。 */
bool servo_is_moving(void);
```

**`tracker.h`** — 跟踪策略层：

```c
typedef enum {
    TRACKER_MODE_IDLE,        /* 无声源，舵机回 home */
    TRACKER_MODE_TRACKING,    /* 跟踪中 */
    TRACKER_MODE_SETTLING,    /* 舵机刚停，等噪音消散 */
    TRACKER_MODE_DISABLED,    /* 手动屏蔽，便于调试 */
} tracker_mode_t;

/* 配置：home 角度（°，相对舵机中心）、跟踪死区、最大角速度。 */
typedef struct {
    float home_deg;           /* 通常 0.0，需要时偏移整个跟踪窗口 */
    float deadband_deg;       /* 目标变化 < deadband 不动，省得舵机一直响。默认 3° */
    uint32_t settle_ms;       /* 舵机停后多久恢复 DOA 更新。默认 250 ms */
} tracker_config_t;

void tracker_init(const tracker_config_t *cfg);

/* 每帧调用一次。根据 DOA 结果更新舵机目标。 */
void tracker_update(const doa_result_t *doa);

tracker_mode_t tracker_get_mode(void);
```

**`main.c`** 改动：

- `app_main` 加 `servo_init()` 和 `tracker_init()` 调用
- `mic_task` 里 `doa_process` 之后加 `tracker_update(&r)`
- 如果 `tracker_get_mode() == TRACKER_MODE_TRACKING` 或 `TRACKER_MODE_SETTLING`，**跳过 `doa_process`**（或清零 `doa_result_t`），避免舵机噪音反馈到算法

## 3. 分阶段实施

每阶段独立交付，可单独测试和回滚。

### Phase 1 — 舵机驱动（预计 0.5 天） ✅ 2026-06-22 完成

**目标**：`servo_set_angle_deg(0)` 让舵机回 home；`servo_set_angle_deg(+20)` 转到 +20°；钳位 ±27° 正确。

**任务**：
- [x] 创建 `main/servo.{c,h}`
- [x] LEDC 定时器配置：`LEDC_TIMER_1`（避开 `LEDC_TIMER_0`，预留给未来可能的摄像头 XCLK）、`LEDC_LOW_SPEED_MODE`、14-bit duty
- [x] LEDC 通道配置：`LEDC_CHANNEL_1`、GPIO 38
- [x] `servo_set_pulse_us(us)` 实现（脉宽 µs → duty 比例）
- [x] `servo_set_angle_deg(angle)` 实现：`pulse = 1500 + (angle / 27.0) * 1000`（home=1500, +27°=2500, -27°=500）
- [x] `servo_is_moving()` 实现（基于 `esp_timer_get_time()` 距上次命令的时差，holdoff 750 ms）
- [x] 在 `main.c::app_main` 加 `servo_init()` 调用
- [x] 启动自检 sweep task（13 步 × 1s，覆盖 ±27° 全范围）替代 UART 命令测试

**验收结果（2026-06-22）**：用户启动板子后观察到舵机完成完整 sweep 序列（0° → +27° → 0° → -27° → 0°，~13 秒），每个位置都能稳定保持。UART 日志里 `servo=+0.0°` 字段在 self-test 完成后稳定显示。Phase 1 通过。

### Phase 2 — 跟踪策略骨架（预计 0.5 天） ✅ 2026-06-22 完成

**目标**：用户说话时舵机自动微调，朝向声源。

**任务**：
- [x] 创建 `main/tracker.{c,h}`
- [x] `tracker_init()` 初始化配置
- [x] `tracker_update()` 核心逻辑：
  1. 如果 `doa->mode == DOA_MODE_INVALID`，不动（保持上次目标）
  2. 如果 `doa->mode == DOA_MODE_3MIC`：target = `doa->azimuth_deg - 180.0 + cfg->home_deg`，clamp ±20°
  3. 如果 `doa->mode == DOA_MODE_2MIC`：不更新目标（半平面歧义太多），仅记录到日志
- [x] deadband：如果 `|target - servo_get_angle_deg()| < deadband_deg`，不调 `servo_set_angle_deg`
- [x] 在 `main.c::mic_task` 的 `doa_process` 之后调用 `tracker_update(&r)`
- [x] UART 日志加 `servo=%+.1f°` 字段

**验收结果**：tracker 逻辑跑通，舵机确实根据 3-mic DOA 跟随声源。但暴露了正反馈振荡和方向反两个新问题（Phase 3 解决）。

### Phase 3 — Motion-pause 噪音抑制 + 稳定性（预计 1 天） ✅ 2026-06-22 完成

**目标**：舵机运动期间及之后 750 ms 内，tracker 跳过 DOA 更新。避免舵机噪音反馈成假方位。同时解决单帧噪声驱动舵机跳到极限的问题。

**任务**：
- [x] `servo_is_moving()`：基于 `esp_timer_get_time()` 距上次命令的时差，holdoff 750 ms
- [x] `tracker_update()` 在最前面加 motion-pause gate
- [x] **机械限位 ±20°**（原计划 ±27°，但发现机械止位震动严重，缩到 ±20°）
- [x] **超范围抑制** `out_of_range_deg=45°`：如果目标超出 ±45°（远超机械范围），不追，避免极限之间振荡
- [x] **方向反转逻辑** `SERVO_SHAFT_INSTALLED_DOWN` 开关，实测确认 = 0（即不需要反转，内齿圆盘 + servo 自带方向恰好抵消）
- [x] **2-frame agreement 滤波** `target_agreement_deg=10°`：连续 2 帧 raw target 差距 < 10° 才命令运动。单帧 GCC-PHAT 暂态（如 servo buzz 漏过 motion-pause）被滤掉。真实位置变化跨多帧持续，能通过。

**验收结果（2026-06-22）**：

| 测试位置 | 期望 servo | 实测 servo | 结果 |
|---|---|---|---|
| 6 点钟（M3 后方）| 0° | -0.7°（稳定 15s）| ✓ |
| 7 点钟（M3 左侧）| +20° | -4° → +20°（跟踪到位）| ✓ |
| 3 点钟（远超范围）| 抑制不动 | 抑制不动 | ✓ |

**意外发现**（写入 CLAUDE.md/SERVO_PLAN）：
1. 舵机在机械止位（±27°）会持续震动，PCB 耦合到 DAT0 双麦 → ρ01=1.0 → L/R 折叠 → 偶尔产生幻像 3-mic 方位 → 反馈到 tracker → 振荡。**解法**：限制 ±20° + motion-pause。
2. 舵机轴向下安装时，方向**需要**在代码里翻转（`SERVO_SHAFT_INSTALLED_DOWN=1`）。这与第一次直觉相反——内齿圆盘和舵机的方向惯例**不**完全抵消。**确认方法**：用户在 7 点钟 → servo 应命令 +20° → M3 物理上转到 7oc 方向（朝用户）。如果 M3 转到 5oc（背离用户），方向反了，toggle 这个宏。
3. 用户在 M3 正后方（6 点钟）时，M1/M2 在远端听到弱信号 → L/R 折叠严重 → 3-mic 帧稀少（每 5 秒 0-2 帧）。这是 3 麦阵列的几何盲区。2-frame agreement 滤波在这种稀疏率下仍能工作，但响应慢。

### Phase 4 — UART 调试 + 调参（预计 0.5 天） ⚠️ 受硬件限制未完成

**目标**：方便现场调整参数，不用每次改代码重烧。

**任务**：
- [x] UART 命令解析（`console.c` 完整实现，支持 `help` / `status` / `servo` / `tracker` / `cfg`）
- [x] Console 任务（直接用 `uart_read_bytes` + `uart_write_bytes`，绕开 VFS 层）
- [ ] **实测 UART RX 不工作** — 发送 `help\r\n` 等命令后，console task 不响应。Task 已运行（boot log 显示 `console ready`），但 `uart_read_bytes` 始终无数据

**根本原因（推测）**：这块 GOOUUU S3-CAM 板的 CH343 USB-UART 芯片**只接了单向（ESP32 → host）**，用于 log 输出。Host → ESP32 方向（GPIO44 RX）的物理连接可能不存在或被切断。这是板级硬件设计问题，软件无法绕过。

**修复路径（如果未来需要）**：
1. 用万用表测 CH343 TX pin 是否连到 GPIO44
2. 如果没连，需要飞线（用户不能焊接，所以这不可行）
3. 或者用 OTG + 外置 USB-UART 接其他 GPIO 做 console
4. 或者用 ESP32-S3 的原生 USB-CDC（需要改 sdkconfig 的 `CONFIG_ESP_CONSOLE_USB_CDC=y`，并接到不同的 USB 端口）

**已交付的代码**：`main/console.{c,h}` 完整保留，等未来硬件支持时直接可用。

**临时调参方法**：当前改参数还是要改 `main/main.c` 里的 `tracker_init(&cfg)` 或 `main/tracker.h` 里的 `TRACKER_DEFAULT_CONFIG` 宏，重新烧录。常调的几个：
- `deadband_deg`（默认 3°）：增大可减少抖动但响应慢
- `out_of_range_deg`（默认 45°）：减小则跟踪范围更窄（保守），增大则更激进但易振荡
- `min_confidence`（默认 0.45）：增大可过滤更多噪声帧但 3-mic 占比下降

### Phase 5 — 集成测试 + 文档（预计 0.5 天）

**任务**：
- [ ] 端到端测试：用户从 5oc 走到 7oc，舵机跟随；走到 3oc，舵机钳制在 +27°
- [ ] 测试噪音抑制：舵机运动期间和静止期间 DOA 读数对比
- [ ] 更新 `README.md` 把"舵机尚未实现"改成"已实现，见 SERVO_PLAN 结果"
- [ ] 更新 `CLAUDE.md` 加实测跟踪精度表
- [ ] 打 tag `servo-tracking-1.0`

## 4. 舵机噪音抑制策略（详细说明）

### 4.1 问题

JS6620 舵机噪音特征：
- **持续保持位置时**：几乎无声（PWM 占空比稳定，电机不转）
- **运动时**：电机驱动 ~200-800Hz 啸叫 + 齿轮宽带噪音
- **到达目标后短暂余震**：100-300ms

对 GCC-PHAT 的影响：
- 舵机在阵列下方 12cm 固定位置 → 3 个麦以略微不同时间收到同一噪音源
- GCC-PHAT 把这个固定 TDOA 当成"声源方位"→ 报错
- 典型症状：舵机一动，DOA 突然指向舵机方向（12oc 上方），不论用户实际在哪

### 4.2 Motion-pause（主策略，Phase 3 实施）

最简单、最有效。逻辑：

```
当 servo_set_angle() 实际改变目标 → 进入 motion 状态
motion 状态持续到舵机内部控制环稳定（粗略估计 500ms）
之后进入 settle 状态再持续 250ms
settle 结束 → 回到 normal

motion + settle 期间，doa_process 仍然运行（消耗 CPU）但结果被丢弃
tracker 不更新目标（继续往原方向运动）
```

这种"丢弃而非冻结"的好处是：算法状态不污染，settle 结束后能立即给出可靠读数。

### 4.3 高通滤波（备选，增强）

如果 motion-pause 不够（比如舵机持续微调、长期不进入 settle），可以在 `doa.c::gcc_phat` 的输入 PCM 上加一个 1 阶 IIR high-pass：

```c
/* 截止 ~300Hz，y[i] = 0.8589 * (y[i-1] + x[i] - x[i-1]) */
static float hp_state[3] = {0, 0, 0};
static int16_t hp_prev[3] = {0, 0, 0};
for (int i = 0; i < n; i++) {
    float x = c0[i];
    float y = 0.8589f * (hp_state[0] + x - hp_prev[0]);
    hp_state[0] = y; hp_prev[0] = x;
    // ...同样处理 c1, c2
}
```

代价：
- 男性低音（80-200Hz 基频）信息丢失
- 女性声音 / 高频辅音不受影响
- GCC-PHAT 在 1024 点 / 48kHz / 21ms 窗口下，300Hz 对应约 6.4 个 FFT bin，过滤掉前 6 个 bin

**建议**：Phase 5 完成后，如果实测发现 motion-pause 不够（比如舵机一直在微调），再加这一层。

### 4.4 不采用的方案

- **自适应噪音消除（LMS / NLMS）**：需要一个专门的参考麦放在舵机旁边。硬件没这条件，跳过。
- **谱减法（spectral subtraction）**：估计舵机频谱并减去。复杂度高，对 GCC-PHAT 这种相位敏感的算法可能帮倒忙。
- **多帧平均消抖**：不解决根因（幻像 TDOA），只是平滑。

## 5. 测试计划

### 5.1 Phase 1 验收（舵机驱动）

| 测试 | 预期 |
|---|---|
| `servo_init()` 后舵机回 home | 转盘指向预设 home |
| `servo_set_angle_deg(0)` | 舵机不动 |
| `servo_set_angle_deg(+27)` | 转盘转到 +27° 极限 |
| `servo_set_angle_deg(+30)` | 钳位到 +27° |
| `servo_set_angle_deg(-27)` | 转盘转到 -27° 极限 |
| 多次来回 `+27 → -27 → +27` | 无失步、无异响 |

### 5.2 Phase 2 验收（跟踪）

| 场景 | 预期 |
|---|---|
| 用户在阵列 6oc 正后方 30cm 说话 | 舵机不动（α=180°, target=0°） |
| 用户移到 5oc（α=150°） | 舵机转到 -27°（target=150-180=-30 钳位到 -27） |
| 用户移到 7oc（α=210°） | 舵机转到 +27° |
| 用户在 6oc 附近 ±5° 内移动 | 舵机不动（deadband） |
| 用户走到 3oc | 舵机钳制在 +27°，不抖 |

### 5.3 Phase 3 验收（噪音抑制）

| 测试 | 预期 |
|---|---|
| 静默时舵机从 home 转到 +20° | 转动过程中 DOA 输出 INVALID（不产生幻像方位） |
| 舵机到位后 250ms | DOA 恢复正常读数 |
| 持续触发舵机（每 500ms 跳一次） | DOA 长时间不输出，但系统不挂死 |

### 5.4 Phase 5 端到端

完整流程测试：
1. 用户坐在阵列 6oc 前方 50cm（home 位置）
2. 开始说话 → 舵机不动（已在 home）
3. 用户身体微倾 → 舵机微调跟随（< 5° 变化）
4. 用户起身走到 5:30 方向 → 舵机转过 ~-15°
5. 用户走回 6oc → 舵机回 home
6. 全程 DOA `stable_sextant` 应锁定 sect=3（6oc）

## 6. 已知限制 / 未解决的问题

| 问题 | 当前状态 | 缓解 |
|---|---|---|
| **54° 行程限制** | 硬件固有 | 文档化，建议用户在 home 方向 ±27° 内使用 |
| **~1.5 秒 hysteresis 延迟** | `stable_sextant` 必需 | 文档化 |
| **9oc 几何盲区** | M1 在用户对侧时 L/R 折叠 | tracker 应忽略 DOA_MODE_2MIC 的"broadside"读数，避免误跟 |
| **跟踪振荡** | 闭环反馈 | deadband ≥ 3° |
| **舵机长期运动** | 寿命 + 发热 | tracker 应有"长时间无更新回 home"策略 |
| **多声源场景** | 当前算法假设单源 | GCC-PHAT 会报混合方位，tracker 应在 conf 突然下降时不动 |

## 7. 不在范围内

以下功能本次计划**不做**，留作未来工作：

- 连续旋转舵机 / 360° 跟踪
- 双轴云台（俯仰）
- 多声源分离与选择
- 自动校准（用户走一圈自动确定 home）
- 蜂鸣器 / LED 状态反馈
- WiFi / 蓝牙远程控制

## 8. 实施顺序总结

```
Phase 1 (0.5d) ── servo 驱动可单独验证
    │
    ▼
Phase 2 (0.5d) ── 端到端跟踪跑通（可能有振荡和噪音问题）
    │
    ▼
Phase 3 (1.0d) ── motion-pause，解决噪音反馈
    │
    ▼
Phase 4 (0.5d) ── 现场调参
    │
    ▼
Phase 5 (0.5d) ── 集成测试 + 文档 + tag
```

总计 **3 个工作日**（保守估计，可能因调试发现新问题而延长）。

## 9. 参考

- `ESP32_S3_CAM_MIC3/main/hw_validate.c` 的 `servo_init()` / `servo_set_pulse_us()` — 直接可用的 LEDC PWM 模板
- 乐鑫官方 `ledc` API 文档：[esp-idf LEDC](https://docs.espressif.com/projects/esp-idf/en/v6.0.1/esp32s3/api-reference/peripherals/ledc.html)
- JS6620 舵机规格（标准 hobby 舵机脉宽：500-2500µs，50Hz）
