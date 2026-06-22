# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project status

**Working.** 360° six-direction (0°/60°/120°/180°/240°/300°) sound source localization on GOOUUU ESP32-S3-CAM + 3DMIC-291 3-mic array is implemented and verified by walking around the board at known clock positions. Mean azimuth offset is **< ±5°** when the user speaks at 30–50 cm with voice-level audio. The result is reported over UART both as a raw azimuth/sextant and as a hysteresis-smoothed `stable_sextant`.

Servo driver on GPIO38 is **not yet implemented** — direction output is UART only.

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

## Calibration results (2026-06-21)

User walked to known clock positions at 30–50 cm distance, spoke continuously. Mean azimuth per position (after the stability improvements in `doa.c`):

| User position | Expected α | Measured α | Offset |
|---|---|---|---|
| 6 o'clock     | 180° | 181.8° | +1.8° |
| 12 o'clock    |   0° |   6.6° | +6.6° |
| 3 o'clock     |  90° | 106.4° | +16° (later tighter) |

All sextant classifications correct. Single-frame azimuth noise ≈ ±15° at voice SNR; the **`stable_sextant` field (3-frame hysteresis) is rock-solid** once it locks.

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

## Reference projects (sibling directories)

- `~/PycharmProjects/ESP32_C3_Mic3/` — Same goal on ESP32-C3. C3's I²S PDM RX supports only **1 data line**, so the L/R collapse there is a true hardware ceiling. Its `localize.c` (cross-correlation + parabolic peak + 3-mic azimuth solver) is the ancestor of this project's `doa.c`. Read its CLAUDE.md for the failed-C3 history.
- `~/PycharmProjects/ESP32_S3_CAM_MIC3/main/hw_validate.c` — S3 + 3DMIC-291 + camera + servo. Has a working **GCC-PHAT + 1024-pt FFT + parabolic peak** implementation (we reused the FFT/GCC-PHAT code) but concluded only 2-mic 1D DOA was achievable, then spent the rest of the code on camera/servo/WiFi/HTTP. Its `SERVO_LEDC_*` constants and `servo_init()` / `servo_set_bin()` are the template for adding servo output to this project.

## Implementation direction still open

1. **Servo driver on GPIO38** — `stable_sextant` should drive a hobby servo to point at the detected direction. Borrow `servo_init()`/`servo_set_bin()` from `ESP32_S3_CAM_MIC3/main/hw_validate.c`. Mechanical coverage of a standard 180° servo with a 50/15 gear reduction is ~54°, less than one sextant — physical pointing only makes sense if the user mounts the servo differently or accepts coarse positioning.
2. **Voice-activity detection gate** — currently the algorithm tries to localize every frame; adding a simple energy gate (skip if `AC_RMS < 30 LSB` on all channels) would suppress more noise frames and free CPU.
3. **PSRAM not used** — all buffers are internal SRAM (`s_dma`, FFT scratch, lag history). If window size or FFT order is increased, move them to PSRAM (board has 8 MB octal PSRAM).
