## SignalChainTest (standalone bring-up sketch)

Flashes on its own, independent of the main firmware. It checks that the
load cell → CS1237 → ESP32 chain samples properly, and nothing else:

- CS1237 configured to `0x7C` (1280 Hz, PGA 128, external ref) with readback
- every conversion timestamped in the DRDY ISR and queued (the shipped driver)
- ErgoJump-style power: latch on GPIO25 asserted before `setup()`, long-press
  (800 ms) on GPIO36 → red LED → release → off, 10 min inactivity auto-off
  (skipped while USB is present)
- the single WS2812B on GPIO26: **solid blue = device on**, solid red = hold
  threshold reached

Not included: BLE, battery monitor / safety cutoff, calibration, NVS. Do not
leave a unit running this sketch on battery unattended — only the inactivity
auto-off protects the cell.

`Config.h`, `Cs1237.*`, `PowerManager.*`, `LedController.*` and `BleService.*`
are **verbatim copies** of `Current project/NexoHHD/`. That is the point: the
test exercises the driver and the BLE module the product ships. Before
trusting a result, confirm they still match:

```powershell
cd "C:\Users\marti\OneDrive\Desktop\NexoHHD"
foreach ($f in "Config.h","Cs1237.h","Cs1237.cpp","PowerManager.h","PowerManager.cpp","LedController.h","LedController.cpp","BleService.h","BleService.cpp") { fc.exe /b "Current project\NexoHHD\$f" "Diagnostics\SignalChainTest\$f" | Select-String "no differences|FC:" }
```

### Flashing

Arduino IDE: board **ESP32 Dev Module**, serial monitor at **115200**, library
`Adafruit NeoPixel`. Pick the same partition scheme you intend to ship
(**Default 4MB with spiffs** keeps the two OTA slots); the diagnostic does not
care, but flashing it with a different table on a unit that already has the
production table forces a full reflash later.

Command line (arduino-cli is bundled with the IDE, nothing to install):

```powershell
$cli = "$env:LOCALAPPDATA\Programs\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
& $cli compile --fqbn esp32:esp32:esp32 "C:\Users\marti\OneDrive\Desktop\NexoHHD\Diagnostics\SignalChainTest"
& $cli upload  --fqbn esp32:esp32:esp32 -p COM5 "C:\Users\marti\OneDrive\Desktop\NexoHHD\Diagnostics\SignalChainTest"
```

### What you should see

Boot:

```
NexoHHD SignalChainTest v0.2.0 (main FW 0.1.0)
pins: LATCH 25  BTN 36  USB 34  LED 26  SCLK 13  DOUT 14
boot levels: BTN 1 (active-HIGH assumed)  USB 1
BTN raw level -> LOW at 812 ms
CS1237: configured 0x7C (1280 Hz, PGA 128, ext ref); DRDY irq type=4 ena=0x01 (expect 4 / non-zero)
streaming one line per 1000 ms window (~1280 samples at 1280 Hz). WIN,<ms> changes it; HELP for commands.
```

`type=4` is the low-level trigger and `ena` non-zero means the pin's interrupt
is routed to a CPU. If either is off, the interrupt path is not set up and
every sample will come through the loop-side fallback (`!MISS` on every line).

Then the stream: every sample is folded into a time window measured on the
ISR timestamps, one line per window. Default window 1000 ms, so one line per
second built from ~1287 samples, the same 1 s cadence as the old bench
sketch's live line. `WIN,25` gives the 40 lines/s pack view (~32 samples each,
the BLE packet size); `WIN,<ms>` accepts 25..10000.

```
  123 1280Hz cell   1287.1 SPS |    -0.003 N tared | RMS 0.0961 N | pkpk 0.6123 N | raw   1234567 | n 1287 | gap  780 us | loop  310 us
```

| Field | What it is |
|---|---|
| `123` | line sequence number (gaps here = the serial monitor dropped lines, not the ADC) |
| `1280Hz cell` | the config we *write*; `CFG` / `MEAS` show what is on the chip |
| `SPS` | samples in the window divided by its exact span (first sample of this window to first sample of the next), from ISR timestamps |
| `N` | window mean at the nominal scale (4.553e-4 N/count); `abs` until you `TARE` |
| `RMS`, `pkpk` | noise inside the window. At 1000 ms this is the real figure; at 25 ms it wobbles ±12 % line to line |
| `raw` | window mean in counts. Watch this one for bit-alignment bugs |
| `n` | samples in the window (~1287 at 1000 ms) |
| `gap` | worst inter-sample interval in the window, including the boundary from the previous one |
| `loop` | worst gap between `loop()` passes during the window: the guardrail from the ErgoJump post-mortem |

Markers are appended when something is wrong: `!DROP` (ring overflowed since
the previous pack), `!RECFG` (config-revert watchdog fired), `!MISS` (the
loop had to read a waiting result because the interrupt did not), `!STORM`
(DOUT stuck low, interrupt cut and recovered), `!BAD` (reads of `0xFFFFFF` or
±full scale), `!RATE` (SPS outside 1257–1317), `!GAP` (an interval over 2 ms).

`STATUS` adds the interrupt-path counters: `irq` (ISR entries, should track
the sample count), `spurious` (entries with DOUT already high, harmless),
`miss` and `storm` (both should stay 0), and `order` (samples whose
timestamp went backwards: always a driver bug, never hardware; must be 0). The first hardware run of the main
firmware showed polling working while a falling-edge ISR never fired, which
is why DRDY is now level-triggered; these counters are how you confirm the
fix on the next flash.

`MEAS,10` (or plain `MEAS`) runs a 10 s measurement while the stream keeps
going, then prints the old sketch's result block:

```
=== BEGIN RESULT ===
config      0x7C requested, 0x7C on chip
setting     1280Hz  PGA128  ch=cell
duration    10000 ms
samples     12871
eff rate    1287.10 SPS (nominal 1280)
max gap     812 us
dropped     0
reverts     0
bad reads   0
loop max    1120 us
mean        1234567 counts
tared       -3 counts | -0.001 N | -0.0001 kg
RMS noise   211.0 counts | 0.0961 N
pk-pk       1345 counts | 0.6123 N
drift       12 counts | 0.0055 N  (first vs last 10%)
=== END RESULT ===
```

Pass criteria (unloaded cell, after ~1 min warm-up):

| Field | Expect | Meaning if not |
|---|---|---|
| `CFG` / `config ... on chip` | `0x7C` | `0xFF`: chip not driving DOUT (wiring, AVDD, or a pulse-count bug). Other value: write not taken |
| `SPS` / `eff rate` | 1280–1295 (≈1287: +0.57 % oscillator) | ≈10: config reverted and the watchdog is not recovering. ≈640: wrong config bits |
| `RMS noise` (MEAS) | ≈ 0.07–0.10 N (≈ 150–220 counts); bench reference 0.096 N | Much higher: excitation/reference noise, cable, or bit misalignment. Exactly 0: stuck data line |
| `pk-pk` (MEAS) | < ~0.7 N | |
| `gap` / `max gap` | ≈ 780 µs, never > 2000 | Missed DRDY: something blocked the ISR (flash write, long critical section) |
| `dropped`, `reverts`, `bad reads` | 0, 0, 0 | `bad`: always a protocol/wiring fault |
| `loop` / `loop max` | < ~2 ms (NeoPixel show costs ~0.3 ms) | Any new blocking call shows up here first |
| `drift` | a few counts over 10 s once warm | Large: thermal transient, or the cell is still settling |
| `N` | ~0 tared; ≈ +9.8 N per kg hung (sign per cell wiring) | No response to load: cell not on A+/A−, or bridge unpowered |

`DUMP,50` prints consecutive raw samples as `idx,dt_us,raw,hex` (stream paused
meanwhile). `dt_us` should sit at ~777 µs; the hex column makes bit-shift
bugs obvious (constant leading bits, values walking by powers of two).

### Noise under radio load (BLE)

The radio is the noisiest thing on the board and the product streams while
it measures, so the noise floor has to be measured in three states. The
sketch boots with the radio **off**; every stream line and the `MEAS` block
carry the radio state after the channel label: `off`, `adv`, `conn`, `stream`.

1. **Radio off.** Let ~10 lines go by. This is the baseline (0.086 N RMS on unit #1).
2. **`BLE,1`.** The production BLE module starts and advertises as `NexoHHD`
   (label `adv`). Ignore the line the command ran on: Bluedroid init blocks
   the loop for a few hundred ms and touches flash, so expect `!GAP` and
   maybe `!DROP` or a few `miss`, once. Let ~10 lines go by.
3. **Connect with nRF Connect** (label `stream`). Open the service ending in
   `0001`, enable notifications on the characteristic ending in `0002`: those
   are the real protocol-v1 data packets, 140 bytes at ~41 per second, and
   `pkt` on each line should read about 41. On Android request MTU 517
   first (menu → Request MTU); with the default 23-byte MTU the stack sends
   each packet truncated to 20 bytes, which loads the radio the same way but
   is not what the app will see. Let ~10 lines go by. `MEAS,30` here gives
   the definitive figure.
4. Optional: `TX,0` for connected-but-idle (label `conn`).

Compare RMS and pk-pk per label. The radio cannot be stopped cleanly, so a
reset is the way back to `off`. Writing a command to the characteristic
ending in `0003` runs it exactly like the serial monitor does (replies still
go to serial).

**First result (unit #1, 2026-09-04):** the noise floor does not move with
the radio (`off` 0.087 N, `adv` 0.087 N, clean `stream` lines 0.087 N), but
while the radio is active individual samples get corrupted: lines with pk-pk
of 3819 N contain exactly one sample whose sign bit read as 0 (a deviation
of 2^23 counts), and 1 to 3 N outliers are single lower bits. On-time reads,
no gaps, no other markers. That is a bit-bang timing margin problem under
RF bursts, not ADC noise.

### Spike detector and the SCLK A/B

A **spike** is a lone sample more than 1500 counts (0.68 N, about 8 sigma)
away from both neighbours while the neighbours agree with each other. No real
force does that inside one sample at 1306 Hz, so every spike is a corrupted
read. Each one prints a `SPIKE` line with the previous, offending and next
raw values in hex and the XOR against the previous sample, so the flipped
bit is named outright:

```
SPIKE #3: prev 0xFFF59C mid 0x7FF59C next 0xFFF5A0 (+8388608 cts, +3819.44 N) -> single bit 23
```

Lines carry `!SPIKE` and `STATUS` counts them (`spikes`). `CLK,<us>` sets the
SCLK half-period at runtime (default now 2 µs; the first BLE run used 1 µs).
To A/B: with nRF Connect connected and `pkt` at ~41, `CLR`, `CLK,1`, wait a
minute, `STATUS`; then `CLR`, `CLK,2`, a minute, `STATUS`. If spikes vanish at
2 µs the margin theory holds and the default stays. If they persist, the
coupling is on the board (SCLK/DOUT routing or the CS1237 digital supply
during TX bursts) and the next step is hardware, not firmware.

### Open items this sketch settles

1. **Button polarity on GPIO36.** The boot line prints the raw level and every
   debounced change is logged. Released should read `LOW`, pressed `HIGH`. If
   the log shows `WARN: BTN still HIGH 5 s after boot`, the LED goes red about
   a second after boot, and the first *press* powers the unit off, the input is
   active-LOW: flip the comparisons in `PowerManager.cpp` and this sketch.
2. **46-pulse register-op boundaries** (`Cs1237.cpp`): a `CFG` readback of
   `0x7C` plus ~1287 SPS proves the write, the read and the 27-pulse data
   frame. The grouping was already cross-checked against the bench sketch
   (`Downloads/cs1237_bringup/cs1237_bringup.ino`); this confirms it on the
   production pins.
3. **USB detect on GPIO34**: `USB` in `STATUS` should read 1 with the cable
   in, 0 without.

### Commands

| Command | Effect |
|---|---|
| `HELP` | list commands |
| `STATUS` | lifetime counters, tare, stream/meas state, pin map |
| `STREAM,0` / `STREAM,1` | pause / resume the stream (handy while reading a result block) |
| `WIN,<ms>` | averaging window per line, 25..10000 ms (default 1000; 25 = 40 lines/s) |
| `BLE,1` | start the radio and advertise (production BLE module); reset to stop |
| `TX,0` / `TX,1` | send protocol-v1 data packets while connected (default on) |
| `CLK,<us>` | SCLK half-period 1..10 µs (default 2); timing-margin A/B under radio load |
| `MEAS` / `MEAS,<s>` | measurement run of s seconds (default 10, max 600); reads the chip config first, prints the result block at the end |
| `CFG` | read back the config register (detaches DRDY for one bounded op; expect one ~2-interval gap) |
| `TARE` | average the next 256 samples as zero; `N` then shows `tared` |
| `DUMP,<n>` | next n raw samples (1..256) as CSV, stream paused meanwhile |
| `CLR` | reset sketch-side maxima (burst, loop gap, bad count) |
| `OFF` | release the latch |

Any command counts as user activity for the auto-off timer.
