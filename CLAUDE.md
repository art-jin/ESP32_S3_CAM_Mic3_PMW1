# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

**Working (v1.1).** 360° six-direction sound source localization + servo tracking on GOOUUU ESP32-S3-CAM + 3DMIC-291 3-mic array is implemented and verified.

- DOA: 6-position calibration, mean azimuth offset **< ±5°** at 30–50 cm voice distance.
- Servo tracking: ±20° mechanical range, **8.7s response time**, **perfectly stable (zero rebound)** thanks to feed-forward compensation. See "Pitfalls §6" for the breakthrough.
- Direction output: UART log + physical servo pointing.

Tagged `v1.1`. Earlier tags `servo-stable-1.0` (slow but stable, pre-feed-forward), `servo-tracking-1.0` (Phase 1-3 done), `3麦阵列测试完成1.0` (DOA only).

## Servo hardware (Phase 1-3 implemented, 2026-06-22)

| Component | Spec |
|---|---|
| Servo model | JS6620 (standard 180° hobby PWM servo) |
| PWM signal | GPIO38, LEDC 50 Hz, pulse 500–2500 µs |
| Servo shaft orientation | Points **down** (away from mic array) |
| Drive gear (on servo shaft) | 15 teeth, external |
| Driven gear (disc) | 50 teeth, **internal** (teeth face inward) — ring gear |
| Gear mesh style | Pinion runs inside ring (internal mesh, same rotation direction) |
| Reduction ratio | 50 / 15 = **3.33 : 1** (servo rotates 3.33× for disc to rotate 1×) |
| Disc coverage for 180° servo travel | 180° / 3.33 = **~54°** of arc |
| Soft clamp | **±20°** (tighter than 27° mechanical limit — see "Pitfalls §5") |
| Disc mounting position | 12 cm below the mic array, at the 12 o'clock direction |

### Performance (v1.1, with feed-forward)

| Test | Result |
|---|---|
| 6oc user, 12s capture | swing = 0.0° (perfectly stable) |
| 6oc → 7oc transition | first motion at **8.7s**, servo → +20° |
| 7oc user, 30s long-term | swing = 0.0°, zero rebound |
| 3oc/9oc (out of range) | suppressed, servo holds last position |

### Tracking pipeline

```
I²S DMA → doa_process → tracker_update → servo_set_angle → LEDC PWM → GPIO38
                              │
                              ▼
                  Motion-pause gate (500ms holdoff)
                  Out-of-range gate  (|target| > 45° → suppress)
                  2-frame agreement (within 5° across consecutive frames)
                  FEED-FORWARD       (α_room = α_array + β_servo, see §6)
                  Deadband           (skip if Δtarget < 3°)
                  Clamp              (±20°)
```

### Phase 4 limitation: UART console

Console code exists (`main/console.{c,h}`) but UART RX on this board doesn't work — CH343 USB-UART appears to be wired for TX only (ESP32 → host log output). Command input from host isn't physically received by the chip. Bypassing the VFS layer with direct `uart_read_bytes` also failed. Code retained for future hardware that supports bidirectional UART.

Tuning still requires editing `TRACKER_DEFAULT_CONFIG` in `main/tracker.h` and reflashing.

## Project goal

Implement **360° six-direction sound source localization** on a GOOUUU ESP32-S3-CAM board using the 3DMIC-291 three-microphone MEMS array. The user cannot solder, so all mic-array channel selection is done in software (I²S PDM L/R channel selection, GPIO matrix clock fan-out) rather than by re-wiring the board. The localization result is reported over UART as a clock-face direction (e.g. "声源位于 6 点方向").

## Toolchain

- **ESP-IDF v6.0.1** at `/Users/arthurjin/.espressif/v6.0.1/esp-idf` (set in `.vscode/settings.json` as `idf.currentSetup`). Source `export.sh` before any `idf.py` command.
- **Target:** `esp32s3`.
- **Build:** CMake + Ninja. Compile commands at `build/compile_commands.json` for clangd.
- **Compiler:** `xtensa-esp32-elf-gcc` from `~/.espressif/tools/xtensa-esp-elf/esp-15.2.0_20251204/...`.

### Common commands

```bash
. /Users/arthurjin/.espressif/v6.0.1/esp-idf/export.sh
idf.py build
idf.py -p /dev/cu.usbmodem21201 flash        # see "Flashing" below for BOOT-button procedure
idf.py -p /dev/cu.usbmodem21201 monitor      # Ctrl-] to exit
```

### Flashing — manual BOOT-button entry required

This GOOUUU board's auto-reset circuit does **not** work with esptool over the CH343 USB-UART. Flashing requires manual bootloader entry:

1. Hold **BOOT**.
2. Press and release **RST**.
3. Release **BOOT**.
4. Run `idf.py -p /dev/cu.usbmodem21201 flash` immediately.

After RST the USB-CDC device name changes (observed: from `cu.usbmodem1234561` to `cu.usbmodem21201`). Use the CH343 name (`usbmodem21201`) for both flash and monitor. Do **not** toggle DTR/RTS from a host script to reset — that re-enters bootloader mode.

### Reading UART without `monitor`

`monitor` needs a TTY. To capture from a non-TTY shell:

```bash
python3 -c "
import serial, time, sys
s = serial.Serial('/dev/cu.usbmodem21201', 115200, timeout=1)
deadline = time.time() + 15
while time.time() < deadline:
    line = s.readline()
    if line: sys.stdout.write(line.decode('utf-8', errors='replace'))
"
```

## Hardware wiring (verified)

Per `ArthurReadMe.md`:

| 3DMIC-291 | ESP32-S3 GPIO | Role |
|---|---|---|
| 3.3V | 3v3 | power |
| GND | GND | ground |
| CLK0 | GPIO1  | I²S PDM RX CLK — drives DAT0's two mics |
| DAT0 | GPIO2  | I²S PDM RX DIN[0] — M2 (L slot) + M1 (R slot) via PDM clock phase |
| CLK1 | GPIO14 | same I²S CLK, fanned out via GPIO matrix (`I2S0I_WS_OUT_IDX` → GPIO 14) |
| DAT1 | GPIO42 | I²S PDM RX DIN[1] — M3 (L slot) |
| Servo | GPIO38 | (planned) LEDC PWM 50 Hz for hobby servo |

The S3 I²S PDM RX peripheral exposes only one CLK output. CLK1 is a GPIO-matrix copy of CLK0 so both 3DMIC clock inputs see the same hardware edge — without this, M3↔M1 TDOA is meaningless.

### Channel → physical mic mapping (verified by tap test 2026-06-21)

**This mapping is NOT what you'd guess from the README.** Tap each physical mic and watch which of `c0/c1/c2` spikes:

| DMA slot | Channel | Physical mic | Clock position (current install) |
|---|---|---|---|
| LINE0_L (`dins[0]` L) | **c0** | **M2** | 10 o'clock |
| LINE0_R (`dins[0]` R) | **c1** | **M1** | 2 o'clock |
| LINE1_L (`dins[1]`)   | **c2** | **M3** | 6 o'clock |

The README suggests "DAT0 = M3 + M2, DAT1 = M1" but in reality DAT0 carries M2 + M1 (c0 + c1) and DAT1 carries M3 (c2). The geometry equations in `main/doa.c` use this corrected mapping.

**Board orientation:** The 3DMIC-291 PCB is installed **component-side down, sound holes up** (flipped relative to the silkscreen's intended orientation). This mirrors the layout across the 12oc-6oc axis: M1 and M2 swap clock positions compared to the silkscreen (M1 silkscreened at 10oc is now physically at 2oc, M2 silkscreened at 2oc is now physically at 10oc). M3 stays at 6oc. The channel-to-mic-label wiring is unchanged — only the physical positions of M1 and M2 swap.

Geometry (equilateral triangle, side d=10 mm, centred at origin):

```
    M2 (c0) @ 10oc        12oc (north, α=0°)
       ●
      /   \                ↗ 0°
     /     \             ↑
    /       \            |
   ●─────────●           +────→ east
 M3 (c2) @ 6oc    M1 (c1) @ 2oc
```

## Code architecture

```
main/
├── main.c           FreeRTOS mic task: capture → deinterleave → DOA → UART log
├── mic_capture.{c,h}  I²S PDM RX multi-DIN init + blocking read; GPIO-matrix CLK fan-out
├── doa.{c,h}        Pure algorithm: FFT, GCC-PHAT, 3-mic / 2-mic geometry solve,
│                    output hysteresis. No hardware deps.
└── CMakeLists.txt   idf_component_register(SRCS main.c mic_capture.c doa.c ...)
```

Audio path runs in a single `mic` task at `configMAX_PRIORITIES - 2` on an 8 KB stack. Window: 100 ms at 48 kHz × 3 channels × int16 = 28 800 bytes per DMA read. FFT is 1024 points (~21 ms).

### Pairwise GCC-PHAT (sign convention — see "Pitfalls" below)

```
lag_01 = gcc_phat(c0,c1) = arrival(M2) − arrival(M1) = +K·sin(α)
lag_02 = gcc_phat(c0,c2) = arrival(M2) − arrival(M3) = +K·sin(α−60°)
lag_12 = gcc_phat(c1,c2) = arrival(M1) − arrival(M3) = −K·sin(α+60°)
```

3-mic solve:

```
sin α = +lag_01 / K
cos α = (lag_01 − 2·lag_02) / (K·√3)
α = atan2(sin α, cos α)    →    0..360°
```

Note: the `sin α` equation has a **positive** sign because the 3DMIC-291 is installed component-side down, mirroring M1/M2 positions. If the board is reinstalled component-side up, the sign flips back to negative — see "Pitfalls" §1 for the derivation.

K = d·fs/c ≈ 1.40 samples at d=10 mm, fs=48 kHz. `K` is also the max possible pairwise TDOA in samples — `lag_*` outside ±K means a noise peak.

## Calibration results (2026-06-21 / 2026-06-22)

User walked to known clock positions at 30–50 cm distance, spoke continuously. Two rounds: pre-flip (board component-side up) and post-flip (board component-side down — current installed orientation).

### Post-flip (current, tag `3麦阵列测试完成1.0`)

| User position | Expected α | Measured α | Offset | Primary mode |
|---|---|---|---|---|
| 6 o'clock     | 180° | 184.1° | +4.1°  | 3-mic |
| 12 o'clock    |   0° |   6.6° | +6.6°  | 3-mic (mirror-axis invariant) |
| 3 o'clock     |  90° |  83.9° | -6.1°  | 3-mic |
| 10 o'clock    | 300° | 291.3° | -8.7°  | 3-mic + 2-mic |
| 9 o'clock     | 270° | 306°   | +36°   | 2-mic fallback (geometric blind spot) |

**4 of 5 positions are within ±10°** of expected — well inside the 60° sextant tolerance. The 9 o'clock position is the array's inherent blind spot: M1 (at 2oc) is on the opposite side of the board from the source, so DAT0's L-slot signal is weak, L/R collapses, and the algorithm correctly falls back to 2-mic half-plane reporting ("M2 side", which 9oc genuinely is). This is not a bug — any 3-mic planar array has this limitation at the "anti-mic" direction.

Single-frame azimuth noise ≈ ±15° at voice SNR; the **`stable_sextant` field (3-frame hysteresis) is rock-solid** once it locks.

### Pre-flip (for historical reference)

| User position | Expected α | Measured α | Offset |
|---|---|---|---|
| 6 o'clock     | 180° | 181.8° | +1.8° |
| 12 o'clock    |   0° |   6.6° | +6.6° |
| 3 o'clock     |  90° | 106.4° | +16° |

## Pitfalls encountered (and how they were fixed)

### 1. GCC-PHAT sign convention

Textbook derivation suggests `gcc_phat(a,b)` returns `arrival(b) − arrival(a)`. **Actual convention in this code is `arrival(a) − arrival(b)`** (a consequence of computing `A·conj(B)` in the frequency domain and IFFT'ing — see derivation in commit history). Initial code had the wrong sign, giving a 180° offset between reported α and physical position. If you ever rewrite `gcc_phat`, re-verify by standing behind a known mic.

### 2. L/R slot collapse is real but conditional — not a hard hardware limit

The S3 on-chip PDM2PCM collapses DAT0's L and R slots into the **same** data stream at low SNR. The C3 reference project (`ESP32_C3_Mic3`) and S3 reference (`ESP32_S3_CAM_MIC3/hw_validate.c`) both concluded this was a hardware ceiling and gave up on 3-mic localization. **It isn't.** When the source emits real audio (AC RMS > ~100 LSB), the L/R slots decorrelate and full 3-mic 360° works. The "collapse" only matters for noise-only frames, where the algorithm should fall back to 2-mic anyway.

Detection that works (in `doa.c`):
- **ρ01 = Pearson correlation of c0,c1**: 0.95+ collapsed, 0.6–0.9 independent. Cleanest signal.
- **max|c0−c1| scaled by AC RMS**: threshold `max(25, 0.5·RMS_c0)`. Better than absolute threshold because it scales with signal level.
- **GCC-PHAT peak height** per pair: `peak_01 ≥ 0.40` required for 3-mic, `peak_02 ≥ 0.30` for 2-mic. Below this the GCC "peak" is a noise artifact and the resulting azimuth is random.

### 3. Output hysteresis must not mix scales

`stable_sextant` requires `DOA_STABLE_STREAK` (3) consecutive identical raw sextant readings before updating. Initially it was fed by both 3-mic sextants (scale 0..5) and 2-mic bins (scale 0..2) — the 2-mic value 2 ("M3 side") collides with the 3-mic value 2 ("4 o'clock"). Fix: in 2-mic mode, **pass `sextant = -1` to the hysteresis function** so it doesn't advance the streak but still echoes the last stable value. See `doa.c` 2-mic branch.

### 4. clangd diagnostics you can ignore

clangd will report `'sys/features.h' file not found` and unused-include warnings on `freertos/FreeRTOS.h`. These are editor-config artifacts (clangd doesn't see the full ESP-IDF include path); the actual `idf.py build` succeeds. Do not chase them.

### 5. Servo mechanical-endpoint buzzing causes L/R collapse feedback

The JS6620 servo buzzes continuously when commanded to its mechanical limits (±27° gear-reduced). The buzz is acoustic energy that couples through the 3DMIC-291 PCB into both mics on DAT0 (they share the same physical board), making their outputs highly correlated. This trips the `ρ01 > 0.95` L/R collapse detector, which produces phantom 3-mic DOA readings, which feed back to the tracker, which commands more servo motion, which creates more buzz... infinite loop.

**Fix**: clamp the servo range tighter than the mechanical limit (`±20°` instead of `±27°`). The buzzing threshold is somewhere between 20° and 27°; 20° is safely below it.

### 6. Closed-loop feedback oscillation in moving-array DOA — and the feed-forward fix

**This is the single most important pitfall in the servo work.** Without addressing it, you can have either fast response OR stability, not both. The breakthrough (v1.1) was recognizing the root cause and applying a one-line algebraic fix.

**Symptom**: when the servo tracks a source, the array rotates, which shifts the source's perceived azimuth (α_array) in the array's frame. If the tracker uses α_array directly to compute the next servo target, the target shifts after every motion. If the shift crosses a sextant boundary, `stable_sextant` flips, the tracker commands the opposite direction, the array rotates back, α_array flips again — sustained oscillation at ~7s period.

**Things we tried that did NOT fix the root cause** (only masked it):
- Conservative mode (use `stable_sextant` instead of raw α): just slows the oscillation, sextant still flips eventually
- Stricter 2-frame agreement filter: slows response but doesn't break the feedback loop
- Higher deadband: filters small changes but a full sextant flip is still a huge change
- Longer motion-pause: just delays the inevitable

**Fix (feed-forward compensation)**: convert the source's azimuth from the array's rotating frame to the **room's fixed frame** before computing the target:

```c
float alpha_room = doa->azimuth_deg + servo_get_angle_deg();
float target = alpha_room - 180.0f + s_cfg.home_deg;
```

Mathematically, `α_array` shifts by exactly `-β_servo` when the array rotates by `+β_servo`. So `α_room = α_array + β_servo` is invariant to servo position. The tracker's target becomes invariant too, and the feedback loop is broken at the algebra level.

This unlocked aggressive parameters (`target_agreement_deg` 10°→5°, `motion-pause` 750ms→500ms) that were previously unstable. Final result: **8.7s response, zero rebound** (was 30s response with strict params pre-feed-forward, or 0.3s response with aggressive params but severe oscillation pre-feed-forward).

**Required condition**: `servo_get_angle_deg()` must return the **actual commanded angle**, not a stale or measured value. We have this (it's just the last commanded value from `servo_set_angle_deg()`). If you ever add servo position feedback (e.g., read servo potentiometer), use that instead.

**References** — closed-loop SSL (Sound Source Localization) with moving arrays is a known research area. The feed-forward trick is standard robotics (convert local frame → world frame using known pose). Useful papers:
- [Closed-loop SSL in neuromorphic systems](https://research.rug.nl/en/publications/closed-loop-sound-source-localization-in-neuromorphic-systems)
- [Modified 3D Kalman (M3K) tracking](https://www.researchgate.net/publication/325975958) — statistical version of the same idea, handles measurement noise better but needs ~100 LoC and parameter tuning

### 7. Servo direction sign — empirical, not derivable

For shaft-down servo installation with internal-mesh ring gear, the relationship between commanded sign and physical rotation direction is NOT derivable from theory (gear conventions cancel partially but not fully). Just test: command `+20°`, observe whether M3 moves CW or CCW from above, flip `SERVO_SHAFT_INSTALLED_DOWN` if wrong.

For this build: `SERVO_SHAFT_INSTALLED_DOWN=1` is correct (positive command = CW rotation viewed from above = M3 toward 7oc from 6oc home).

## Reference projects (sibling directories)

- `~/PycharmProjects/ESP32_C3_Mic3/` — Same goal on ESP32-C3. C3's I²S PDM RX supports only **1 data line**, so the L/R collapse there is a true hardware ceiling. Its `localize.c` (cross-correlation + parabolic peak + 3-mic azimuth solver) is the ancestor of this project's `doa.c`. Read its CLAUDE.md for the failed-C3 history.
- `~/PycharmProjects/ESP32_S3_CAM_MIC3/main/hw_validate.c` — S3 + 3DMIC-291 + camera + servo. Has a working **GCC-PHAT + 1024-pt FFT + parabolic peak** implementation (we reused the FFT/GCC-PHAT code) but concluded only 2-mic 1D DOA was achievable, then spent the rest of the code on camera/servo/WiFi/HTTP. Its `SERVO_LEDC_*` constants and `servo_init()` / `servo_set_bin()` are the template for adding servo output to this project.

## Implementation direction still open

1. **UART console for runtime tuning** — `console.c` is written but the board's CH343 USB-UART appears to be wired for log output only (no host→ESP32 RX path). To unblock, either jumper a real USB-UART to other GPIOs, or switch to native USB-CDC via `CONFIG_ESP_CONSOLE_USB_CDC=y` (requires USB cable to the S3's native USB port, not the CH343 port).
2. **Voice-activity detection gate** — currently the algorithm tries to localize every frame; adding a simple energy gate (skip if `AC_RMS < 30 LSB` on all channels) would suppress more noise frames and free CPU.
3. **PSRAM not used** — all buffers are internal SRAM (`s_dma`, FFT scratch, lag history). If window size or FFT order is increased, move them to PSRAM (board has 8 MB octal PSRAM).
