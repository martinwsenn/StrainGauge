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
read, a glitch on the DOUT pin, or a very short mechanical impulse. Each one
prints a `SPIKE` line with uptime, radio state and SCLK setting, the
previous / offending / next raw values in hex, and a classification:

```
SPIKE #3 @412s ble=stream clk=1us: prev 0xFFF59C mid 0x7FF5A1 next 0xFFF5A0 (+8388613 cts, +3819.44 N) -> bit 23 (jump +8388613 = +2^23 +5)
SPIKE #4 @413s ble=stream clk=1us: prev 0x0012AB mid 0x001AC0 next 0x0012B9 (+2069 cts, +0.94 N) -> bit 11 (jump +2069 = +2^11 +21)
SPIKE #5 @980s ble=off clk=2us:    prev 0xFFF8C0 mid 0xFFFFFF next 0xFFF8B2 (+1855 cts, +0.84 N) -> all-ones read (DOUT not driven)
SPIKE #6 @981s ble=off clk=2us:    prev 0xFFF8C0 mid 0xFFE100 next 0xFFF8B2 (-6080 cts, -2.77 N) -> no bit pattern (xor 0x0019C0)
```

- **bit k**: the jump from both neighbours is 2^k within noise (±700 counts).
  A single flipped bit in the read: bit 23 is the sign (3819 N), bits 11–13
  are 1–4 N. This is the RF timing-margin signature.
- **all-ones read**: the sample was 0xFFFFFF. DOUT was never driven low
  during the read, so the ISR entered on something other than DRDY (a glitch
  on the pin, e.g. while handling the bare board).
- **no bit pattern**: anything else — a mechanical tap, multi-bit corruption.

Lines carry `!SPIKE`; `STATUS` shows `spikes N (bit-like B, all-ones A)`, so
the classification survives even if the `SPIKE` lines scroll away. `CLR`
resets all three. `CLK,<us>` sets the SCLK half-period at runtime (default
3 µs since 2026-09-08; 2 µs before that, the first BLE run used 1 µs).

**A/B procedure.** `WIN,10000` (one line per 10 s keeps the log short).
Radio off: `CLR`, 3 min, `STATUS` (expect 0). `BLE,1`, connect, subscribe to
`0002` (the firmware only transmits packets to a subscribed central; without
it the radio is nearly idle and the test proves nothing). Then per setting:
`CLR`, `CLK,n`, 5 min untouched, `STATUS`, keep every `SPIKE` line.

**Results (unit #1):**

| Date | Condition | Half-period | Duration | Spikes |
|---|---|---|---|---|
| 2026-09-04 | stream | 1 µs | ~60 s | 53 |
| 2026-09-04 | stream | 2 µs | 84 s | 2 (bits unknown, lines lost) |
| 2026-09-08 | off | 2 µs | 251 s | 0 |
| 2026-09-08 | stream | 2 µs | 594 s | 0 |
| 2026-09-08 | stream | 3 µs | 369 s | 0 |
| 2026-09-08 | stream | 4 µs | 353 s | 0 |
| 2026-09-08 (2nd) | off, boot + placing + TARE | 2 µs | 106 s | 0 |
| 2026-09-08 (2nd) | stream, positive control | 1 µs | 60 s | 52 (24 single-bit, 28 multi-bit) |
| 2026-09-08 (2nd) | stream | 2 µs | 489 s | 1 (multi-bit) |

The positive control reproduced the first session (52 vs 53 per minute at
1 µs), so the radio load in these runs is real and comparable. Pooled: 2 µs
= 1 corrupted sample in ~18 min of streaming; 3–4 µs = 0 in 12 min. **The
default is now 3 µs** (`CS1237_SCLK_HALF_US`): a 400× reduction from 1 to
2 µs says margin is the lever, and 3 µs costs nothing measurable (21% ISR
duty, loop gap unchanged). Keep testing at the shipped setting so the
evidence accumulates where it matters.

What the classified lines showed at 1 µs: single-bit errors were bit 11
(≈1 N) and bit 23 (the sign, 3819 N); the multi-bit ones included the same
corrupted word twice in separate events (`mid 0xFFE01F`: bits 12–5 read 0,
bits 4–0 read 1, regardless of the true bits). Data-independent runs like
that mean DOUT itself was not presenting data for ~13 bit times: the
CS1237 output stage is being starved during the TX burst, which slows its
edges past the sample point. Slower clocks tolerate slower edges, hence the
trend. A scope on DOUT and the CS1237 supply pin during a BLE burst at
`CLK,1` would show it directly (respin input).

The 16 pre-baseline spikes from the first 2026-09-08 session did not
reproduce (0 through boot, placing and taring in the second session), so
they are attributed to handling the bare board; the all-ones class in the
detector will catch a recurrence.

**Noise in 10 s windows** is not comparable with the 1 s figures: RMS reads
0.094–0.104 N radio-off (drift inside the longer window adds to it). In the
first 2026-09-08 session it crept from ~0.100 to ~0.118 N over 30 min on
battery; in the second (12 min) it stayed at 0.093–0.112 N and `MEAS,30`
under streaming at 2 µs gave **0.0962 N RMS, 1.12 N pk-pk, drift −0.01 N**.
Cause of the earlier creep unknown, so `STATUS` prints the pack voltage
(`bat`).

**`bat` read 3.23–3.28 V throughout the second session.** That is not a
pack: on the bench the PCB is powered from an ESP32 devkit's 3.3 V rail into
the battery node (the devkit is also the USB-UART bridge), and the reading
is that rail within ~0.05 V, so the divider and ADC path are roughly right.
It also means **both AP2112 regulators are in dropout during every bench
run** (3.3 V in, 3.3 V out): nothing rejects the ESP32's TX current pulses
before they reach the CS1237's supply. That is the worst possible case for
the read-corruption finding above. **All spike counts in the table were
taken in that condition.** The next run must be on a charged pack, with the
devkit wired for GND/TX/RX only: repeat the 1 µs positive control (52/min
on the devkit rail) and a 10 min run at the shipped 3 µs. If the pack run
is clean even at 1 µs, the corruption is a bench artefact; if it persists,
the CS1237 supply filtering goes on the respin as planned.

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
| `STATUS` | lifetime counters, spike classes, tare, radio state, pack voltage, pin map |
| `STREAM,0` / `STREAM,1` | pause / resume the stream (handy while reading a result block) |
| `WIN,<ms>` | averaging window per line, 25..10000 ms (default 1000; 25 = 40 lines/s) |
| `BLE,1` | start the radio and advertise (production BLE module); reset to stop |
| `TX,0` / `TX,1` | send protocol-v1 data packets while connected (default on) |
| `CLK,<us>` | SCLK half-period 1..10 µs (default 3); timing-margin A/B under radio load |
| `MEAS` / `MEAS,<s>` | measurement run of s seconds (default 10, max 600); reads the chip config first, prints the result block at the end |
| `CFG` | read back the config register (detaches DRDY for one bounded op; expect one ~2-interval gap) |
| `TARE` | average the next 256 samples as zero; `N` then shows `tared` |
| `DUMP,<n>` | next n raw samples (1..256) as CSV, stream paused meanwhile |
| `CLR` | reset sketch-side maxima and counts (burst, loop gap, bad reads, spikes) |
| `OFF` | release the latch |

Any command counts as user activity for the auto-off timer.
