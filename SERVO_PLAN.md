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

### Phase 1 — 舵机驱动（预计 0.5 天）

**目标**：`servo_set_angle_deg(0)` 让舵机回 home；`servo_set_angle_deg(+20)` 转到 +20°；钳位 ±27° 正确。

**任务**：
- [ ] 创建 `main/servo.{c,h}`
- [ ] LEDC 定时器配置：`LEDC_TIMER_1`（避开 `LEDC_TIMER_0`，预留给未来可能的摄像头 XCLK）、`LEDC_LOW_SPEED_MODE`、14-bit duty
- [ ] LEDC 通道配置：`LEDC_CHANNEL_1`、GPIO 38
- [ ] `servo_set_pulse_us(us)` 实现（脉宽 µs → duty 比例）
- [ ] `servo_set_angle_deg(angle)` 实现：`pulse = 1500 + (angle / 27.0) * 1000`（home=1500, +27°=2500, -27°=500）
- [ ] `servo_is_moving()` 占位返回 `false`（运动检测放到 Phase 3）
- [ ] 在 `main.c::app_main` 加 `servo_init()` 调用
- [ ] 在 UART 命令解析里加 `servo <angle>` 测试命令（可选，调试用）

**测试方法**：烧固件，板子启动后舵机回 home。手动通过 UART 输入 `servo +10`、`servo -20` 等命令，看舵机是否正确转动。用尺子量转盘角度验证 ±27° 行程。

**模板**：直接借用 `ESP32_S3_CAM_MIC3/main/hw_validate.c` 的 `servo_init()` / `servo_set_pulse_us()` 实现。常量已经一致（SERVO_GPIO=38, 50Hz, 500-2500µs）。

### Phase 2 — 跟踪策略骨架（预计 0.5 天）

**目标**：用户说话时舵机自动微调，朝向声源。

**任务**：
- [ ] 创建 `main/tracker.{c,h}`
- [ ] `tracker_init()` 初始化配置
- [ ] `tracker_update()` 核心逻辑：
  1. 如果 `doa->mode == DOA_MODE_INVALID`，不动（保持上次目标）
  2. 如果 `doa->mode == DOA_MODE_3MIC`：target = `doa->azimuth_deg - 180.0 + cfg->home_deg`，clamp ±27°
  3. 如果 `doa->mode == DOA_MODE_2MIC`：不更新目标（半平面歧义太多），仅记录到日志
- [ ] deadband：如果 `|target - servo_get_angle_deg()| < deadband_deg`，不调 `servo_set_angle_deg`
- [ ] 在 `main.c::mic_task` 的 `doa_process` 之后调用 `tracker_update(&r)`
- [ ] UART 日志加 `target=%+.1f°` 字段

**测试方法**：用户在 6oc ± 27° 范围内（即 5oc–7oc 之间）移动，舵机应该平滑跟随。走到 4oc 或更远，舵机应该钳制在 +27° 或 -27° 不动。

**已知坑（提前防）**：
- **跟踪振荡**：DOA 读到 α=180° → 舵机回到 0° → 但用户实际在 178° → 阵列转了 -2° → DOA 又读到 180° → 循环。**解法**：deadband ≥ 3° 或加 hysteresis。这是必须的。
- **迟延叠加**：`stable_sextant` 已经 1.5 秒滞后，再加上舵机 0.5 秒机械响应，用户走动时滞后 2 秒。文档化即可。

### Phase 3 — Motion-pause 噪音抑制（预计 1 天）

**目标**：舵机运动期间及之后 250 ms 内，DOA 不更新。避免舵机噪音反馈成假方位。

**任务**：
- [ ] `servo.c` 里实现 `servo_is_moving()`：
  - 记录每次 `servo_set_angle_deg` 的目标值和当前 duty
  - 启动一个 FreeRTOS 软定时器或 task，每次调用 reset 计时器
  - 计时器归零后 250 ms 内 `is_moving` 仍为 true
- [ ] `main.c::mic_task` 修改：
  ```c
  if (tracker_get_mode() == TRACKER_MODE_SETTLING) {
      // 跳过 doa_process，但仍然读 DMA 防止 I²S buffer 满
      // 清零 doa_result_t 或标记 mode=INVALID
      continue;
  }
  doa_process(...);
  tracker_update(&r);
  ```
- [ ] `tracker.c` 实现模式切换：
  - 收到新目标 → 检查是否 > deadband → 如果是，进 TRACKING，调 servo_set_angle
  - servo_set_angle 触发 motion → 等舵机内部 motion 完成（或固定等 500ms）→ 进 SETTLING
  - SETTLING 等 250ms → 回 IDLE/TRACKING

**测试方法**：让舵机运动期间对着麦说话——不应该报出与舵机位置相关的固定"幻像方位"（之前未抑制时常见症状）。

**备选增强（可选，先跳过）**：
- 在 `doa.c::gcc_phat` 之前对 PCM 加 300 Hz high-pass（1阶 IIR），滤掉舵机 200-800Hz 噪音带。代价：低频语音信息丢失，GCC-PHAT 在静音/低 SNR 下更敏感。Phase 3 完成后如果发现 motion-pause 不够再加。

### Phase 4 — UART 调试 + 调参（预计 0.5 天）

**目标**：方便现场调整参数，不用每次改代码重烧。

**任务**：
- [ ] UART 加一行命令解析（用 `esp_console` 或简单的 `fgets`+`sscanf`）：
  - `servo <angle>` — 手动设舵机角度
  - `tracker on/off` — 启用/禁用自动跟踪
  - `cfg deadband <deg>` — 实时改死区
  - `cfg home <deg>` — 实时改 home 偏移
  - `cfg settle <ms>` — 实时改 settle 时间
- [ ] 把所有配置存到 NVS，重启后恢复（可选）

**测试方法**：通过 UART 实时调参，找到 deadband / settle 的最佳值。

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
