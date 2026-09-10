# NexoHHD — where things stand (2026-09-10)

Handheld dynamometer: ZELL TSA 300 kg load cell → CS1237 24-bit ADC (1280 Hz,
PGA 128) → ESP32-WROOM-32 → force in Newtons over BLE to a companion app.
Custom PCB "Sch celda de carga", one WS2812B status LED, TP4056 charger,
AP2112K-3.3 regulators behind a power latch. Full detail lives in `CLAUDE.md`
(project context and open items) and `README.md` (protocol, calibration,
building). This file is the short version.

## What was done, in order

1. **Firmware v0.1.0 written** (`Current project/NexoHHD/`): CS1237 driver,
   force calibration in NVS, BLE service, battery monitor, power manager,
   LED controller, main pipeline. Compiles; the full sketch has run on
   hardware only once, before the driver fixes below.
2. **Standalone bring-up sketch** (`Diagnostics/SignalChainTest/`, now
   v0.4.1): signal chain only, plus the same latch / button / auto-off
   behaviour as ErgoJump and a blue status LED. It streams one line per
   averaging window in the format of the old bench sketch (SPS, N, RMS,
   pk-pk, raw mean, max gap, loop gap, fault markers) and has a `MEAS` block,
   `TARE`, `DUMP`, `CFG`, `CLK`, `BLE,1`, `STATUS`. Its driver, config, power,
   LED and BLE files are **verbatim copies** of the main sketch so it tests
   what ships; re-copy after any edit (README has the check).
3. **Interrupt path fixed.** A falling-edge DRDY interrupt never delivered a
   sample (DRDY is a level, one missed edge = permanent stall). Now
   low-level triggered with a storm guard, a loop-side fallback read, and
   `IRQ/SPUR/MISS/STORM` counters. A second bug (reading the clock before
   the last-sample timestamp → unsigned wrap → phantom reads and spurious
   reconfigures) fixed the same day. Rule: read `lastSampleUs` before `now`.
4. **Signal chain characterised** on PCB unit #1 (bench-powered).
5. **BLE added to the diag** to measure noise under real streaming load
   (protocol-v1 packets to nRF Connect, 40/s). Noise did not move, but reads
   got corrupted while the radio transmitted; traced to bit-bang timing
   margin; SCLK half-period made runtime-settable and A/B'd.
6. **Bench power caveat found** (2026-09-08): the PCB is fed 3.3 V from an
   ESP32 devkit into the battery node, so both regulators sit in dropout.
   Every RF-corruption figure was taken in that worst case.

## Measured (unit #1)

| Quantity | Value |
|---|---|
| Sample rate | 1306.3 SPS (+2.05 % vs 1280 nominal; varies per chip) |
| Noise, radio off / adv / streaming | 0.086 N RMS in all three (1 s windows); 0.096 N in 30 s |
| pk-pk / RMS | ≈ 6.1 (Gaussian; no mechanical dependence) |
| Zero drift | 0.1–0.2 N/min after handling, decaying; ~2 N over 6 min from cold |
| Nominal scale | 4.553e-4 N/count, agrees with bench calibration to 0.03 % |
| Config register | 0x7C written and read back on production pins |
| Corrupted reads under BLE streaming | 1 µs SCLK: ~52/min; 2 µs: ~1 per 18 min; 3–4 µs: 0 in 12 min (bench-powered) |

Error budget per single sample ≈ 0.6 N, dominated by the cell's own
combined error (0.59 N), not the electronics. The 1 N target is lost only
through calibration span error or an un-tared drift.

## The mode we settled on

- **Always sample at 1280 Hz, decimate per SKU.** 32-sample packets, 40/s.
- **Timestamp in the ISR, decide in the loop.** The DRDY ISR stamps each
  sample with 64-bit `esp_timer_get_time()` and clocks out 24 bits with
  per-bit critical sections (SCLK never high > 100 µs), into a 512-sample
  ring; overflow drops oldest and counts. The loop drains, calibrates,
  decimates, packs. Never `micros()`, never the nominal interval.
- **Nothing blocks on a steady-state path.** Housekeeping is state
  machines. No `delay()`. Commands run on the loop task; BLE callbacks only
  enqueue.
- **Diagnostics are part of the data contract**: dropped samples, config
  reverts, max sample gap, max loop gap travel in the packet / STATUS.
- **Self-healing driver**: config-revert watchdog (register is volatile,
  drops to 10 Hz on supply dips), storm guard, fallback read, bounded waits.
- **SCLK half-period 3 µs** (`CS1237_SCLK_HALF_US`), pending the
  pack-powered test.
- **App derives dt from packet timestamps** (u32 µs, wraps every 71.6 min),
  never from sample counts: on this unit a counting app would report RFD
  2 % low. Hand this to the app team with the protocol spec.
- **Per-unit calibration in NVS via a procedure**, never hand-edited into
  Config.h. Tare at the start of every session, after the unit is in place.
- **Corrupted samples must be flagged in the packet, not passed silently**
  (a sign-bit flip is 3819 N). Production detector still to be written.

## Open, in priority order

1. **Repeat the RF test on a charged pack** (devkit wired GND/TX/RX only):
   `CLK,1` for 60 s as positive control, then 10 min at `CLK,3`. Decides
   whether the corruption is a bench artefact or a PCB respin item (RC on
   CS1237 DVDD).
2. Run the **full main sketch** on hardware with the fixed driver.
3. Cheap checks: 1 L water bottle → expect 9.8 N; press the button → `BTN 1`;
   USB-C in → `USB 1`; battery monitor on a real pack.
4. Production spike flagging in the data contract; BLE format freeze with
   the app team (protocol v1 is provisional).
5. OTA: partition scheme must be chosen before units ship; main firmware is
   already 86 % of a default OTA slot, so this is coupled to Bluedroid vs
   NimBLE. NVS schema version before the first OTA.

## Working on it

- Build: `arduino-cli.exe compile --fqbn esp32:esp32:esp32 --warnings all
  --build-path <scratch> "<sketch dir>"` (bundled with Arduino IDE 2; ESP32
  core 3.3.8, Adafruit NeoPixel 1.15.4). Compile-check every change, both
  sketches.
- Diag serial: 115200, newline line ending, uppercase commands, `HELP`.
- Ground rules from the ErgoJump post-mortem are in `CLAUDE.md`; do not
  regress them.
