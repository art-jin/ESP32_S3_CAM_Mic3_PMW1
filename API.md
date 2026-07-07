# API 参考文档

> 设备：ESP32-S3-CAM + 3DMIC-291 声源定位云台
> 版本：v2.0
> 更新：2026-07-05

---

## 一、设备能力（Skills）

本设备具备两种独立工作技能，通过 REST API 切换：

### Skill 1：声源跟踪（TRACK）

**模式标识**：`track`

**行为**：设备自动检测声源方位（GCC-PHAT 3 麦阵列算法），驱动舵机云台转动使 M3 麦（6 点钟方向）指向声源。无需外部干预。

**技术参数**：
- 定位精度：±5°（正常说话，30-50cm 距离）
- 响应时间：0.5-4 秒（取决于 3-mic 帧率）
- 覆盖范围：6 点钟 ± 100°（约 2:40 → 9:20 点钟）
- 帧率：16.6 Hz（DMA 50ms 窗口）
- 静默 10 秒后自动回 home (0°)
- 舵机速度：300°/s（soft-start），运动后暂停 200-500ms

**适用场景**：
- 用户坐在设备前方说话（桌面交互）
- 演示/展示（设备自动跟随说话者）
- 任何需要"麦阵对准声源"的自动场景

**限制**：
- 有效距离 30-50cm（正常说话）/ 1-1.5m（大声）/ 2-3m（吹哨）
- 9 点钟方向为几何盲区（M1 对侧），精度下降
- 舵机运动期间 DOA 暂停（motion-pause），避免机械噪音反馈

### Skill 2：指令转向（COMMAND）

**模式标识**：`command`

**行为**：声源跟踪完全停止。舵机只响应 REST API 的 `/api/point` 指令，转向外部系统指定的方向。DOA 算法仍在运行（数据可通过 `/api/status` 查询），但不驱动舵机。

**技术参数**：
- 可达方向：2 点钟 → 10 点钟（每整点一个位置，共 9 个）
- 也可指定任意角度（-100° 到 +100°）
- 舵机速度：300°/s（soft-start）
- 指令间隔限制：500ms 最小间隔
- 超时：5 分钟无指令自动切回声源跟踪

**适用场景**：
- 外部 AI Agent 决定云台朝向（"转向声源"后再微调）
- 多设备协同（主设备指定其他设备看向某方向）
- 手动远程控制（通过手机/电脑 curl 命令）
- 配合摄像头/传感器固定方向采集

**与 Skill 1 的协作**：
```
外部系统工作流示例：
  1. GET /api/status → 查看当前声源方位（azimuth）
  2. POST /api/mode {"mode":"command"} → 接管控制权
  3. POST /api/point {"dir":"7oc"} → 转向目标方向
  4. 执行任务（如拍照、录音）
  5. POST /api/mode {"mode":"track"} → 交还控制权给自动跟踪
```

### Skill 3：摇动（SHAKE）

**模式标识**：`command`（摇动是 Skill 2 指令模式下的子功能）

**行为**：在当前指向位置的 ±10° 范围内左右摇动舵机。摇动模式：先快速摇动 3 次（用于吸引注意），暂停 2 秒，再摇动 2 次（节奏变化），结束后回到原始指向位置。

**摇动序列**：
```
当前位置 P（记录）
振幅 hi = min(P+10, +100)    ← 边界自适应
振幅 lo = max(P-10, -100)

第 1 组（3 次左右）：
  P → hi → lo → hi → lo → hi → lo    ~2.8s

暂停 2s

第 2 组（2 次左右）：
  P → hi → lo → hi → lo              ~2.0s

回到原位 → P                         ~0.2s

总计 ~7s
```

**边界处理**：如果当前指向接近机械极限（如 P=95°），摇动范围自动收缩到 [85°, 100°]。不强制 ±10° 对称，但保证不超机械范围。

**技术参数**：
- 振幅：±10°（边界自适应）
- 每个端点停留：400ms
- 两组间暂停：2s
- 总时长：~7s
- 阻塞式：API 调用阻塞到摇动完成后返回
- 期间 `/api/point` 和 `/api/mode` 返回 409 Conflict

**适用场景**：
- AI Agent 吸引用户注意（"看这里"）
- 寻找/定位目标后确认指向（"我在看这个方向"）
- 提示/通知（通过机械动作传递信号）
- 示威/展示效果

### 三种技能对比

| 维度 | 声源跟踪（TRACK） | 指令转向（COMMAND） | 摇动（SHAKE） |
|---|---|---|---|
| 舵机驱动方 | DOA 算法自动 | REST API 外部指令 | REST API 内置序列 |
| DOA 运行 | ✅ 运行且驱动舵机 | ✅ 运行但不驱动舵机 | ✅ 运行但不驱动舵机 |
| `/api/point` | ❌ 返回 403 | ✅ 接受并执行 | ❌ 返回 409 |
| `/api/shake` | ❌ 返回 403 | ✅ 接受并执行 | ❌ 返回 409 |
| `/api/mode` | ✅ 可切换 | ✅ 可切换（摇动中 409）| — |
| 结束位置 | 持续跟踪 | 停在最后指令位置 | **回到摇动前位置** |
| 超时 | 无 | 5 分钟 → 自动回 TRACK | 无（7s 自动结束）|
| 开机默认 | ✅ | — | — |

---

## 二、设备发现

### mDNS 自动发现

设备启动并连接 WiFi 后，自动注册 mDNS 服务：

```
Hostname: esp32-mic-<MAC后4位十六进制>.local
示例:     esp32-mic-24f8.local
Service:  _http._tcp on port 80
```

发现方法（局域网内任意设备）：
```bash
# macOS / Linux
dns-sd -B _http._tcp local.

# 或直接访问
curl http://esp32-mic-24f8.local/api/ping
```

### 设备标识

| 项 | 值 | 获取方式 |
|---|---|---|
| Device ID | NVS 随机生成 6 位 [A-Z0-9]，每台板子不同 | UART 启动日志：`Device ID: XXXXXX` |
| mDNS Hostname | `esp32-mic-XXXX.local` | mDNS 扫描 |
| IP 地址 | DHCP 分配 | UART 日志或路由器查 |

**重要**：Device ID 是鉴权密钥，**仅通过 UART 日志获取**，不在网络层暴露。

---

## 三、API 规格

### 基础信息

| 项 | 值 |
|---|---|
| 协议 | HTTP（明文，适用于局域网） |
| 端口 | 80 |
| Content-Type | `application/json` |
| CORS | `Access-Control-Allow-Origin: *` |
| 鉴权方式 | URL 查询参数 `?device_id=XXXX` |

### 鉴权

除 `/api/ping` 外，所有 API 必须在 URL 中携带 `device_id` 参数：

```
GET /api/status?device_id=XXXXXX
POST /api/point?device_id=XXXXXX
```

未携带或错误的 device_id 返回 401。

---

### GET /api/ping

心跳检查，无需鉴权。

**请求**：
```
GET /api/ping
```

**响应**（200）：
```json
{"ok": true}
```

**用途**：确认设备在线、HTTP 服务正常。

---

### GET /api/status

查询完整设备状态。

**请求**：
```
GET /api/status?device_id=XXXXXX
```

**响应**（200）：
```json
{
  "ok": true,
  "mode": "track",              // "track" 或 "command"
  "servo": 30.0,               // 当前舵机角度（度，-100 到 +100）
  "moving": false,             // 舵机是否正在运动
  "azimuth": 210,              // 最新 DOA 方位角（0-360）
  "sect": "7 o'clock",         // 稳定扇区
  "conf": 0.55,                // DOA 置信度（0-1）
  "wifi": "connected",         // WiFi 状态
  "ip": "192.168.1.105",      // IP 地址
  "host": "esp32-mic-24f8"    // mDNS 主机名
}
```

**字段说明**：

| 字段 | 类型 | 说明 |
|---|---|---|
| `mode` | string | 当前工作模式 |
| `servo` | float | 舵机当前角度。0° = M3 指向 6 点钟；正值 = 顺时针（向 7-10oc）；负值 = 逆时针（向 5-2oc） |
| `moving` | bool | 舵机是否在 soft-start 过渡中 |
| `azimuth` | float | DOA 测量的声源方位。0° = 12oc，90° = 3oc，180° = 6oc，270° = 9oc |
| `sect` | string | 3 帧一致后的稳定扇区（DOA hysteresis） |
| `conf` | float | DOA 置信度。< 0.35 = 噪声，0.35-0.55 = 可用，> 0.55 = 高质量 |
| `wifi` | string | "connected" 或 "disconnected" |

**注意**：command 模式下 `azimuth` 仍然返回实时值（DOA 算法持续运行），外部系统可利用此特性做"固定指向 + 监测声源"。

---

### POST /api/mode

切换工作模式。

**请求**：
```
POST /api/mode?device_id=XXXXXX
Content-Type: application/json

{"mode": "command"}
```

**可选参数**：
```json
{"mode": "command", "timeout": 0}    // timeout=0 表示不自动切回 track
{"mode": "command", "timeout": 600}   // 10 分钟后自动切回 track
```

**响应**（200）：
```json
{"ok": true, "mode": "command"}
```

**错误**：

| HTTP Code | error | 条件 |
|---|---|---|
| 400 | bad_request | body 缺少 `mode` 字段或值不合法 |
| 401 | unauthorized | device_id 缺失或错误 |

**行为细节**：
- `track → command`：立即停止 tracker，舵机保持当前位置
- `command → track`：重置 tracker 状态（EMA、agreement buffer），从当前角度恢复跟踪
- 默认超时 300 秒（5 分钟），`timeout` 参数可覆盖

---

### POST /api/point

指令舵机转向指定方向。仅在 COMMAND 模式下可用。

**请求**（方式 A - 时钟方向）：
```
POST /api/point?device_id=XXXXXX
Content-Type: application/json

{"dir": "7oc"}
```

**请求**（方式 B - 原始角度）：
```
POST /api/point?device_id=XXXXXX
Content-Type: application/json

{"angle": 30}
```

**方向映射表**：

| dir 值 | servo 角度 | 物理方向 | 可达 |
|---|---|---|---|
| `"2oc"` | -100°（clamp） | 2 点钟 | ⚠️ 超机械范围，clamp |
| `"3oc"` | -90° | 3 点钟 | ✅ |
| `"4oc"` | -60° | 4 点钟 | ✅ |
| `"5oc"` | -30° | 5 点钟 | ✅ |
| `"6oc"` | 0° | 6 点钟（home） | ✅ |
| `"7oc"` | +30° | 7 点钟 | ✅ |
| `"8oc"` | +60° | 8 点钟 | ✅ |
| `"9oc"` | +90° | 9 点钟 | ✅ |
| `"10oc"` | +100°（clamp） | 10 点钟 | ⚠️ 超机械范围，clamp |

角度含义：0° = M3 指向 6 点钟（home），正值 = 顺时针，负值 = 逆时针。范围 ±100°。

**响应**（200）：
```json
{"ok": true, "servo": 30.0}
```

**响应**（clamped）：
```json
{"ok": true, "servo": 100.0, "clamped": true}
```

**错误**：

| HTTP Code | error | 条件 |
|---|---|---|
| 400 | bad_request | body 缺少 `dir` 和 `angle`，或 dir 值不在映射表中 |
| 401 | unauthorized | device_id 缺失或错误 |
| 403 | mode_is_track | 当前是 track 模式，需先 POST /api/mode 切换 |
| 429 | rate_limited | 500ms 内重复调用 |

**行为细节**：
- **异步返回**：API 立即返回目标角度，不等舵机到位。调用方通过 `/api/status` 的 `moving` 字段判断是否到位
- **速率限制**：最小 500ms 间隔。超频返回 429
- **每次调用重置 command 超时计时器**

---

### POST /api/shake

摇动舵机 ±10°（边界自适应），吸引注意或确认指向。仅在 COMMAND 模式下可用。

**请求**：
```
POST /api/shake?device_id=XXXXXX
```

无需 body。使用固定参数（3+2 组、±10°、400ms 停留、2s 暂停）。

**响应**（200，阻塞 ~7s 后返回）：
```json
{"ok": true}
```

**摇动序列**：
```
记录当前位置 P
hi = min(P + 10, +100)     ← 边界自适应
lo = max(P - 10, -100)

第 1 组：3 次左右（各 400ms 停留）
  P → hi → lo → hi → lo → hi → lo
  ≈ 2.8s

暂停 2s

第 2 组：2 次左右
  P → hi → lo → hi → lo
  ≈ 2.0s

回到 P（原位）
  ≈ 0.2s

总计 ≈ 7s
```

**边界示例**：
- P = 0° → hi=10, lo=-10（完整 ±10°）
- P = 95° → hi=100, lo=85（正向收缩到 +5°，负向保持 -10°）
- P = -98° → hi=-88, lo=-100（负向收缩到 +8°）

**行为细节**：
- **阻塞式**：HTTP 连接保持 ~7s，摇动完成后返回。调用方需设置 ≥10s 超时
- **摇动期间** `/api/point` 返回 409 Conflict
- **摇动期间** `/api/mode` 返回 409 Conflict
- **速率限制**：与 `/api/point` 共享 500ms 间隔
- **结束位置**：精确回到摇动前的位置

**错误**：

| HTTP Code | error | 条件 |
|---|---|---|
| 401 | unauthorized | device_id 缺失或错误 |
| 403 | mode_is_track | 当前是 track 模式 |
| 409 | shaking | 摇动进行中（重复调用） |
| 429 | rate_limited | 500ms 内重复调用 |

---

## 四、错误响应统一格式

所有错误响应遵循：

```json
{
  "ok": false,
  "error": "<error_code>",
  "message": "<human_readable_message>"
}
```

| HTTP Code | error | message | 触发条件 |
|---|---|---|---|
| 400 | `bad_request` | body 空/过长/JSON 解析失败/字段缺失 | POST 请求格式错误 |
| 401 | `unauthorized` | invalid or missing device_id | 鉴权失败 |
| 403 | `mode_is_track` | switch to command mode first | track 模式下调用 /api/point 或 /api/shake |
| 409 | `shaking` | shake in progress, wait for completion | 摇动期间调用 /api/point 或 /api/mode |
| 429 | `rate_limited` | min 500ms between commands | /api/point 或 /api/shake 高频调用 |
| 500 | `internal` | unexpected error | 意料外错误 |

---

## 五、集成示例

### Python

```python
import requests

BASE = "http://192.168.1.105"
DEVICE_ID = "XXXXXX"

# 查状态
r = requests.get(f"{BASE}/api/status", params={"device_id": DEVICE_ID})
status = r.json()
print(f"模式: {status['mode']}, 方位: {status['azimuth']}°, 舵机: {status['servo']}°")

# 切到指令模式 + 指向 7oc + 摇动 + 切回跟踪
requests.post(f"{BASE}/api/mode", params={"device_id": DEVICE_ID}, json={"mode": "command"})
requests.post(f"{BASE}/api/point", params={"device_id": DEVICE_ID}, json={"dir": "7oc"})
time.sleep(1)  # 等 point 完成 + 避免 rate limit

# 摇动（阻塞 ~7s）
r = requests.post(f"{BASE}/api/shake", params={"device_id": DEVICE_ID}, timeout=10)
print(f"摇动完成: {r.json()}")

time.sleep(1)
requests.post(f"{BASE}/api/mode", params={"device_id": DEVICE_ID}, json={"mode": "track"})
```

### JavaScript (浏览器)

```javascript
const BASE = "http://esp32-mic-24f8.local";
const DEVICE_ID = "XXXXXX";

// 查状态
const res = await fetch(`${BASE}/api/status?device_id=${DEVICE_ID}`);
const status = await res.json();
console.log(`模式: ${status.mode}, 方位: ${status.azimuth}°`);

// 指向 9oc
await fetch(`${BASE}/api/mode?device_id=${DEVICE_ID}`, {
    method: "POST",
    body: JSON.stringify({mode: "command"})
});
await fetch(`${BASE}/api/point?device_id=${DEVICE_ID}`, {
    method: "POST",
    body: JSON.stringify({dir: "9oc"})
});

// 摇动（阻塞 ~7s）
await new Promise(resolve => setTimeout(resolve, 600)); // 避免 rate limit
const shakeRes = await fetch(`${BASE}/api/shake?device_id=${DEVICE_ID}`, {
    method: "POST"
});
console.log("摇动完成:", await shakeRes.json());
```

### curl

```bash
# 查状态
curl "http://esp32-mic-24f8.local/api/status?device_id=XXXXXX"

# 切到指令模式
curl -X POST "http://esp32-mic-24f8.local/api/mode?device_id=XXXXXX" -d '{"mode":"command"}'

# 指向 7 点钟
curl -X POST "http://esp32-mic-24f8.local/api/point?device_id=XXXXXX" -d '{"dir":"7oc"}'

# 指向自定义角度 -45°
curl -X POST "http://esp32-mic-24f8.local/api/point?device_id=XXXXXX" -d '{"angle":-45}'

# 摇动舵机（阻塞 ~7s，记得设超时）
curl --max-time 10 -X POST "http://esp32-mic-24f8.local/api/shake?device_id=XXXXXX"

# 切回声源跟踪
curl -X POST "http://esp32-mic-24f8.local/api/mode?device_id=XXXXXX" -d '{"mode":"track"}'
```

---

## 六、典型工作流

### 场景 A：AI Agent 外设视觉

```
Agent（Mac）                    ESP32 设备
   │                               │
   ├── GET /api/status ──────────→ │ 返回 azimuth=210 (声源在 7oc)
   │                               │
   ├── （Agent 决定指向 7oc）      │
   │                               │
   ├── POST /api/mode command ──→  │ tracker 停止
   ├── POST /api/point 7oc ─────→  │ 舵机转到 +30°
   │                               │
   ├── （Agent 执行其他任务）      │ 舵机固定在 7oc
   │                               │
   └── POST /api/mode track ────→  │ tracker 恢复
                                   │
                                   ▼
                              自动跟踪恢复
```

### 场景 B：多声源监测

```
Agent 想知道不同方向有没有声源：

1. POST /api/mode command          → 接管舵机
2. POST /api/point 3oc             → 指向 3oc
3. GET  /api/status                → 读 azimuth + conf（3oc 方向有无声音）
4. POST /api/point 9oc             → 指向 9oc
5. GET  /api/status                → 读 azimuth + conf（9oc 方向有无声音）
6. POST /api/mode track            → 恢复自动跟踪
```

### 场景 C：纯跟踪（不需要 REST API）

```
开机 → boot sweep → WiFi 连接 → 进入 TRACK 模式 → 自动跟踪声源
（REST API 可用但不需要调用，设备完全自主工作）
```

### 场景 D：摇动吸引注意

```
Agent 检测到用户走神，需要吸引注意：

1. POST /api/mode command          → 接管舵机
2. POST /api/point 6oc             → 指向用户方向
3. POST /api/shake                 → 摇动 ±10°，3+2 组（~7s）
                                    用户注意到设备在动
4. POST /api/mode track            → 恢复自动跟踪
```
