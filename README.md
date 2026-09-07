# NexoHHD

Firmware for a sellable handheld dynamometer: ZELL TSA 300 kg load cell → CS1237 ADC (1280 Hz, PGA 128, external ref) → ESP32-WROOM-32 → BLE stream to a companion app. TP4056 charger, AP2112K regulators with a power latch, one WS2812B status LED.

## Pinout (from "Sch celda de carga")

| Function | GPIO | Notes |
|---|---|---|
| Power latch (AP2112 EN) | 25 | HIGH = stay on; asserted every loop pass |
| Power button | 36 | input-only; assumed active HIGH — **verify at bring-up** |
| Battery sense | 39 | ADC1, divider 220k/47k ≈ 5.68 |
| Status LED (WS2812B) | 26 | |
| LED strip (WS2812B) | 23 | unused by firmware so far; count unconfirmed |
| USB detect (VBUS divider) | 34 | digital, HIGH = USB present |
| CS1237 SCLK | 13 | |
| CS1237 DOUT/DRDY | 14 | bidirectional during register ops |

TP4056 CHRG/STDBY are not wired to the MCU: "charging" = "USB present".

## CS1237 notes

- Config `0x7C`: external ref, **1280 Hz**, PGA 128, channel A. Effective rate runs +0.57% above nominal → every sample carries its own `esp_timer` timestamp; nothing assumes the nominal interval.
- The config register is **volatile and reverts to 0x0C (10 Hz) on supply dips the ESP32 survives**. `cs1237Service()` runs a revert watchdog: no sample for 50 ms → rewrite config, count it, flag it in the stream.
- SCLK must never stay high >100 µs (chip powers down); every high phase sits inside a critical section. DOUT is sampled while SCLK is high.
- DRDY is a **level** (DOUT low until the result is clocked out), so the interrupt is low-level triggered, not edge triggered: a missed edge would stall the stream forever, which is exactly what the first hardware run showed with a falling-edge ISR. A storm guard cuts the interrupt if DOUT is stuck low, and the loop reads a waiting result itself if the ISR has not taken it within ~1.5 intervals. Both are counted and reported in `STATUS` (`IRQ`, `SPUR`, `MISS`, `STORM`); healthy is `IRQ` ≈ samples and the rest 0.
- After any config write: 2 ms settle + 4 discarded conversions.
- The 46-pulse register-op grouping (24+3+2+7+1+8+1) was cross-checked against the bench bring-up sketch that produced the noise figures; remaining differences are documented in the `Cs1237.cpp` header. Hardware confirmation comes from `Diagnostics/SignalChainTest` (`CFG` → 0x7C, ~1287 SPS).

## BLE (protocol v1 — PROVISIONAL until the app team confirms)

Service `8e7c1237-a5b4-46f1-93f5-1b0c5a6e0001`. Requires MTU ≥ 143 (firmware allows up to 517).

- **Data** `...0002` (notify): binary packets, little-endian.
- **Control** `...0003` (write): text commands.
- **Status** `...0004` (read/notify): text status.

### Data packet (12-byte header + payload)

| Offset | Type | Meaning |
|---|---|---|
| 0 | u8 | protocol version (1) |
| 1 | u8 | flags: b0 config-revert since last pkt, b1 drops since last pkt, b2 payload is raw counts |
| 2 | u16 | sequence number — the app must check for gaps |
| 4 | u32 | timestamp of first sample, µs (low 32 bits) |
| 8 | u16 | total dropped samples, saturating |
| 10 | u8 | sample count (32 default → 40 pkt/s at 1280 Hz) |
| 11 | u8 | reserved |
| 12… | f32×N | Newtons (or raw counts if flag b2) |

Dropped counts and revert flags are part of the data contract on purpose: a force curve with silently missing samples must never be presentable as valid.

### Commands

`START` / `STOP` · `TARE` · `CAL0` · `CALW,<newtons>` · `DEC,<1..32>` · `RAW,<0|1>` · `STATUS` · `OFF`. Every reply is `OK:...` or `ERR:...` — invalid input always fails loudly.

## Per-unit calibration (required for every production unit)

Calibration lives in NVS (`nexo` namespace), not in code. Firmware defaults are nominal datasheet values and are flagged `unit NOT calibrated` on boot.

1. Rig the unit unloaded, let it warm up ~1 min.
2. `CAL0` → stores the zero-load offset.
3. Apply a known reference load (dead weight, in Newtons).
4. `CALW,<newtons>` → stores the scale.
5. `STATUS` → record `CALOFF`/`CALSC` in the unit's production record.

`RAW,1` streams raw counts for verification against the reference.

## Bring-up: `Diagnostics/SignalChainTest`

Flash this **before** the main firmware on any new board. It is the signal chain only (CS1237 → ESP32 → serial), the ErgoJump-style power latch/button/auto-off, and the status LED (blue = on, red = power-off hold reached). No BLE, no battery monitor. It uses verbatim copies of the main sketch's `Cs1237.*`, `PowerManager.*`, `LedController.*` and `Config.h`, so what it validates is the shipped driver. It streams one line per averaging window (default one per second; `WIN,25` for 40 lines/s) with SPS, force, RMS and pk-pk noise, raw mean, sample count, worst sample gap and worst loop gap, in the format of the original bench sketch, and `MEAS,<s>` prints that sketch's measurement result block. Also `STREAM,0|1` / `CFG` / `TARE` / `DUMP,<n>`. Pass criteria and failure signatures are in its README.

## Building

`arduino-cli` ships inside the Arduino IDE; no separate install is needed:

```powershell
$cli = "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
& $cli compile --fqbn esp32:esp32:esp32 --warnings all "C:\Users\marti\OneDrive\Desktop\NexoHHD\Current project\NexoHHD"
```

Board: ESP32 Dev Module, ESP32 core 3.3.x, library `Adafruit NeoPixel`. The main sketch currently builds to ~1.13 MB, which is 86% of a 1.25 MB OTA slot in the default 4 MB partition scheme (see the OTA notes in `CLAUDE.md`).

## Firmware ground rules (from the ErgoJump post-mortem)

1. **No `delay()` on any steady-state path.** All housekeeping is non-blocking state machines. Nothing is "paused during measurement" because nothing ever blocks. (ErgoJump's ~110 ms blocking battery read swallowed plate contacts → the "picos" bug.)
2. **Timestamp in the ISR, decide in the loop.** 64-bit `esp_timer_get_time()` only; never `micros()`/`millis()` on the measurement path.
3. **Queue every sample, never sample state.** Ring overflow drops oldest and increments a **reported** counter.
4. **Diagnostics are part of the data contract**: dropped counts, revert recoveries, max sample gap, and max loop gap all ride in the stream or `STATUS`. The `LOOP=` figure is the guardrail that flags any future blocking addition immediately.
5. **Commands run on the loop task only.** BLE callbacks enqueue and return.
6. **Bounded waits everywhere** (boot button release, CS1237 readiness).
7. **Brown-out detector disabled only during the boot-latch window**, restored at end of `setup()`.
8. Magic numbers that must be tuned per unit live in **NVS via a procedure**, not in `Config.h` edits.
