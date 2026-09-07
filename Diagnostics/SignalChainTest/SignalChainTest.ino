// ============================================================
// NexoHHD - SignalChainTest (standalone bring-up sketch)
//
// Exercises the signal chain and nothing else:
//   load cell -> CS1237 (0x7C: 1280 Hz, PGA 128, ext ref) -> ESP32 -> serial
// plus the ErgoJump-style power latch / button / inactivity auto-off, and
// the single WS2812B on GPIO26: solid BLUE = device is on, solid RED = the
// power-off hold threshold was reached (release the button to switch off).
// No BLE, no battery monitor, no calibration / NVS.
//
// Cs1237.*, PowerManager.*, LedController.* and Config.h are VERBATIM
// copies of Current project/NexoHHD/, so this tests the driver the product
// ships. Keep them identical (diff before trusting a result).
//
// Serial 115200. Samples are averaged over a time window measured on the
// DRDY-ISR timestamps (default 1000 ms = one line per second, ~1287
// samples; WIN,<ms> changes it, e.g. WIN,25 gives 40 lines/s) and every
// window is printed as one line mirroring the bench bring-up sketch
// (cs1237_bringup.ino), never using loop-observation time:
//   seq rate chan SPS | force N | RMS N | pk-pk N | raw mean | n samples |
//   worst inter-sample gap | worst loop gap | !markers
//   (DROP RECFG MISS STORM BAD RATE GAP)
// MEAS,<s> ports the bench sketch's measurement run ("=== BEGIN RESULT ===").
//
// BLE is OFF at boot (radio-off noise baseline). BLE,1 starts the
// production BLE module (BleService.cpp, verbatim copy) and advertises;
// once a central (nRF Connect) is connected, the pipeline sends the real
// protocol-v1 data packets, so noise is measured under the exact radio
// load the product will have. Every line carries the radio state
// (off / adv / conn / stream) like the bench sketch did.
// Commands: HELP STATUS STREAM,0|1 WIN,<ms> MEAS[,<s>] BLE,1 TX,0|1 CFG
//           TARE DUMP,<n> CLR OFF
// ============================================================

#include "Config.h"
#include "Cs1237.h"
#include "LedController.h"
#include "PowerManager.h"
#include "BleService.h"
#include <Arduino.h>
#include <math.h>
#include <string.h>
#include "esp_timer.h"

#define DIAG_VERSION "0.4.0"
#define SERIAL_BAUD 115200
// Stream and DUMP lines are written from the sample pipeline. A large TX
// buffer keeps Serial.print from ever blocking the loop: even the fastest
// window (40 lines/s) needs ~4.5 KB/s of the 11.5 KB/s available, a full
// DUMP is < 8 KB.
#define SERIAL_TX_BUFFER 8192
#define WINDOW_MS_DEFAULT 1000 // one line per second
#define WINDOW_MS_MIN 25       // 40 lines/s, ~32 samples each (the BLE pack)
#define WINDOW_MS_MAX 10000
#define TARE_SAMPLES 256
#define DUMP_MAX_SAMPLES 256
#define DUMP_LINE_BYTES 40 // skip a line (and count it) rather than block
#define MEAS_DEFAULT_S 10
#define MEAS_MAX_S 600
#define ADC_RETRY_MS 3000
#define NOT_RUNNING_NOTICE_MS 1000
#define BOOT_RELEASE_TIMEOUT_MS 5000
// Sanity bands behind the "!" markers on the stream line.
#define SPS_EXPECTED 1287.0 // 1280 Hz nominal, +0.57% internal oscillator
#define SPS_TOLERANCE 30.0
#define GAP_WARN_US 2000 // > ~2.5 sample intervals = a missed DRDY
#define NEWTONS_PER_KG 9.80665

// ---- Labels decoded from the config we WRITE. CFG and MEAS show what is
// actually on the chip. Bit layout: b6 REFO_OFF | b5:4 speed | b3:2 PGA | b1:0 ch
static const char *rateLabel() {
  switch (CS1237_CONFIG_VALUE & 0x30) {
    case 0x00: return "10Hz";
    case 0x10: return "40Hz";
    case 0x20: return "640Hz";
    default: return "1280Hz";
  }
}
static uint16_t nominalRate() {
  switch (CS1237_CONFIG_VALUE & 0x30) {
    case 0x00: return 10;
    case 0x10: return 40;
    case 0x20: return 640;
    default: return 1280;
  }
}
static const char *chanLabel() {
  switch (CS1237_CONFIG_VALUE & 0x03) {
    case 0x00: return "cell";
    case 0x02: return "temp";
    case 0x03: return "short";
    default: return "rsvd";
  }
}
static unsigned pgaLabel() {
  switch (CS1237_CONFIG_VALUE & 0x0C) {
    case 0x00: return 1;
    case 0x04: return 2;
    case 0x08: return 64;
    default: return 128;
  }
}

static float toNewtons(double counts) {
  return (float)(counts * DEFAULT_NEWTONS_PER_COUNT);
}

static uint32_t uptimeS() {
  return (uint32_t)((uint64_t)esp_timer_get_time() / 1000000ULL);
}

// ---- Running statistics over a set of samples (a pack, or a MEAS run).
// Every timestamp comes from the DRDY ISR.
struct Stats {
  uint32_t n = 0;
  int32_t ref = 0; // first sample; deltas keep the sums small
  int64_t sum = 0;
  double sumsq = 0.0;
  int32_t mn = 0, mx = 0;
  uint64_t tFirst = 0, tLast = 0;
  uint32_t gapMax = 0;
  uint32_t rail = 0; // samples pinned at +/- full scale
  uint32_t ones = 0; // samples reading -1 (0xFFFFFF): DOUT stuck high

  void reset() { *this = Stats(); }

  // gapUs = interval from the previous sample (0 for the first ever), so
  // the gap across a pack boundary is still counted.
  void add(int32_t raw, uint64_t t, uint32_t gapUs) {
    if (n == 0) {
      ref = raw;
      tFirst = t;
      mn = 0;
      mx = 0;
    }
    if (gapUs > gapMax) gapMax = gapUs;
    tLast = t;
    const int32_t d = raw - ref;
    sum += d;
    sumsq += (double)d * (double)d;
    if (d < mn) mn = d;
    if (d > mx) mx = d;
    if (raw == 0x7FFFFF || raw == -0x800000) rail++;
    if (raw == -1) ones++;
    n++;
  }
  double mean() const { return n ? (double)ref + (double)sum / n : 0.0; }
  double rms() const {
    if (n == 0) return 0.0;
    const double m = (double)sum / n;
    const double v = sumsq / n - m * m;
    return sqrt(v > 0.0 ? v : 0.0);
  }
  int32_t pkpk() const { return mx - mn; }
  double sps() const {
    return (n < 2) ? 0.0 : (double)(n - 1) * 1e6 / (double)(tLast - tFirst);
  }
};

// ---- Window stream ----
static Stats pack;
static uint32_t packSeq = 0;
static uint64_t windowUs = (uint64_t)WINDOW_MS_DEFAULT * 1000ULL;
static uint32_t packLoopGapMax = 0;
static uint32_t lastDrop = 0, lastRecfg = 0; // change since last pack -> marker
static uint32_t lastMiss = 0, lastStorm = 0;
static bool streamOn = true;

// ---- MEAS,<s>: bounded measurement run, result block like the bench sketch ----
struct Meas {
  bool active = false;
  uint64_t startUs = 0, durationUs = 0;
  Stats acc;
  double firstSum = 0.0, lastSum = 0.0; // first / last 10% of the run -> drift
  uint32_t firstN = 0, lastN = 0;
  uint32_t drop0 = 0, recfg0 = 0, loopMax = 0;
  uint8_t cfgOnChip = 0xFF;
};
static Meas meas;

// Lifetime figures (STATUS; CLR resets the sketch-side ones).
static uint64_t lastSampleUs = 0; // last popped sample: DUMP dt, pack-boundary gap
static uint32_t lifetimeSamples = 0;
static uint32_t lifetimeBad = 0;
static uint32_t orderErrors = 0; // timestamp went backwards: a driver bug, never hardware

// ---- Spike detector: a lone sample that disagrees with BOTH neighbours by
// more than SPIKE_TH_COUNTS while the neighbours agree with each other. The
// cell's mechanical bandwidth is far below 1306 Hz, so no real force does
// that inside one sample: it is a corrupted read. Each one is logged with
// the XOR against the previous sample so the flipped bit is visible. The
// first BLE run produced these only while the radio was active (sign bit
// and bits 10-13), at SCLK half-period 1 us.
#define SPIKE_TH_COUNTS 1500L // ~0.68 N, ~8 sigma of the 190-count noise
#define SPIKE_LOG_PER_S 4
static int32_t spikeA = 0, spikeB = 0;
static uint8_t spikeFill = 0;
static uint32_t spikes = 0, spikesWindow = 0;
static uint32_t spikeLogCount = 0, spikeLogSecond = 0;

static void detectSpike(int32_t c) {
  if (spikeFill >= 2) {
    const int32_t a = spikeA, b = spikeB;
    const long da = (long)b - (long)a, dc = (long)b - (long)c, dac = (long)c - (long)a;
    if (labs(da) > SPIKE_TH_COUNTS && labs(dc) > SPIKE_TH_COUNTS && labs(dac) <= SPIKE_TH_COUNTS) {
      spikes++;
      spikesWindow++;
      const uint32_t sec = uptimeS();
      if (sec != spikeLogSecond) {
        spikeLogSecond = sec;
        spikeLogCount = 0;
      }
      if (spikeLogCount++ < SPIKE_LOG_PER_S) {
        const uint32_t x = ((uint32_t)b ^ (uint32_t)a) & 0xFFFFFFUL;
        char bitInfo[32];
        if (x != 0 && (x & (x - 1)) == 0) snprintf(bitInfo, sizeof(bitInfo), "single bit %d", __builtin_ctz(x));
        else snprintf(bitInfo, sizeof(bitInfo), "xor 0x%06lX", (unsigned long)x);
        Serial.printf("SPIKE #%lu: prev 0x%06lX mid 0x%06lX next 0x%06lX (%+ld cts, %+.2f N) -> %s\n",
                      (unsigned long)spikes, (unsigned long)((uint32_t)a & 0xFFFFFFUL),
                      (unsigned long)((uint32_t)b & 0xFFFFFFUL), (unsigned long)((uint32_t)c & 0xFFFFFFUL),
                      da, toNewtons((double)da), bitInfo);
      }
    }
  }
  spikeA = spikeB;
  spikeB = c;
  if (spikeFill < 2) spikeFill++;
}
static uint32_t maxBurst = 0;
static uint32_t maxLoopGapUs = 0;
static uint64_t lastLoopUs = 0;

// Tare (average of the next TARE_SAMPLES samples, captured non-blocking).
static bool tareValid = false;
static int32_t tareCounts = 0;
static uint32_t tareRemaining = 0;
static int64_t tareSum = 0;

// DUMP,<n>: print the next n raw samples as CSV from the pipeline.
static uint32_t dumpRemaining = 0;
static uint32_t dumpIndex = 0;
static uint32_t dumpSkipped = 0;

static uint32_t lastAdcRetryMs = 0;
static uint32_t lastNotRunningNoticeMs = 0;

// ---- BLE: the production module, started on demand (bench sketch's 'b').
// Once a central is connected the pipeline sends protocol-v1 packets so the
// noise delta is measured under the real radio load. Stopping Bluedroid
// cleanly is unreliable, so like the bench a reset is the way back to
// radio-off.
static bool bleStarted = false;
static bool bleTx = true; // send data packets while connected
static uint8_t packetBuf[12 + 4 * SAMPLES_PER_PACKET];
static uint8_t packetCount = 0;
static uint64_t packetFirstUs = 0;
static uint16_t packetSeq = 0;
static uint32_t packetsSent = 0;
static uint32_t packetsWindow = 0;
static uint32_t lastPacketDropped = 0, lastPacketReconfigs = 0;

static const char *bleLabel() {
  if (!bleStarted) return "off";
  if (!isBleConnected()) return "adv";
  return bleTx ? "stream" : "conn";
}

// Protocol v1 packet, byte for byte what NexoHHD.ino sends (keep in sync):
// 12-byte LE header (version, flags, u16 seq, u32 first-sample us, u16
// dropped saturating, u8 count, u8 reserved) + count x float32 Newtons.
static void sendPacket() {
  const uint32_t dropped = cs1237DroppedCount();
  const uint32_t reconfigs = cs1237ReconfigCount();
  uint8_t flags = 0;
  if (reconfigs != lastPacketReconfigs) flags |= 0x01;
  if (dropped != lastPacketDropped) flags |= 0x02;
  lastPacketReconfigs = reconfigs;
  lastPacketDropped = dropped;
  const uint16_t droppedSat = (dropped > 0xFFFF) ? 0xFFFF : (uint16_t)dropped;
  const uint32_t t32 = (uint32_t)packetFirstUs;
  packetBuf[0] = BLE_PROTOCOL_VERSION;
  packetBuf[1] = flags;
  packetBuf[2] = (uint8_t)(packetSeq & 0xFF);
  packetBuf[3] = (uint8_t)(packetSeq >> 8);
  packetBuf[4] = (uint8_t)(t32 & 0xFF);
  packetBuf[5] = (uint8_t)((t32 >> 8) & 0xFF);
  packetBuf[6] = (uint8_t)((t32 >> 16) & 0xFF);
  packetBuf[7] = (uint8_t)((t32 >> 24) & 0xFF);
  packetBuf[8] = (uint8_t)(droppedSat & 0xFF);
  packetBuf[9] = (uint8_t)(droppedSat >> 8);
  packetBuf[10] = packetCount;
  packetBuf[11] = 0;
  bleNotifyData(packetBuf, 12 + 4 * (size_t)packetCount);
  packetSeq++;
  packetsSent++;
  packetsWindow++;
  packetCount = 0;
}

static void packetAppend(float value, uint64_t tUs) {
  if (packetCount == 0) packetFirstUs = tUs;
  memcpy(&packetBuf[12 + 4 * (size_t)packetCount], &value, 4);
  if (++packetCount >= SAMPLES_PER_PACKET) sendPacket();
}

// ---- Stream line, once per window ----

// nextFirstUs = timestamp of the first sample of the NEXT window, so the
// window spans exactly pack.n intervals and the rate is exact.
static void emitPack(uint64_t nextFirstUs) {
  const double sps = (double)pack.n * 1e6 / (double)(nextFirstUs - pack.tFirst);

  const uint32_t drop = cs1237DroppedCount();
  const uint32_t recfg = cs1237ReconfigCount();
  const uint32_t miss = cs1237EdgeMissCount();
  const uint32_t storm = cs1237StormCount();
  char mark[64] = "";
  if (drop != lastDrop) strcat(mark, " !DROP");
  if (recfg != lastRecfg) strcat(mark, " !RECFG");
  if (miss != lastMiss) strcat(mark, " !MISS");   // loop had to read DRDY itself
  if (storm != lastStorm) strcat(mark, " !STORM"); // DOUT stuck low, interrupt cut
  if (spikesWindow > 0) strcat(mark, " !SPIKE"); // corrupted read(s), see SPIKE lines
  if (pack.rail + pack.ones > 0) strcat(mark, " !BAD");
  else if (fabs(sps - SPS_EXPECTED) > SPS_TOLERANCE) strcat(mark, " !RATE");
  if (pack.gapMax > GAP_WARN_US) strcat(mark, " !GAP");
  lastDrop = drop;
  lastRecfg = recfg;
  lastMiss = miss;
  lastStorm = storm;

  if (streamOn && dumpRemaining == 0) {
    const double meanCounts = pack.mean();
    const double net = meanCounts - (tareValid ? (double)tareCounts : 0.0);
    Serial.printf("%5lu %-6s %-5s %-6s %7.1f SPS | %+9.3f N %s | RMS %.4f N | pkpk %.4f N | "
                  "raw %9.0f | n %4lu | pkt %3lu | gap %4lu us | loop %4lu us%s\n",
                  (unsigned long)packSeq, rateLabel(), chanLabel(), bleLabel(), sps, toNewtons(net),
                  tareValid ? "tared" : "abs  ", toNewtons(pack.rms()), toNewtons(pack.pkpk()),
                  meanCounts, (unsigned long)pack.n, (unsigned long)packetsWindow,
                  (unsigned long)pack.gapMax, (unsigned long)packLoopGapMax, mark);
  }
  packSeq++;
  pack.reset();
  packLoopGapMax = 0;
  packetsWindow = 0;
  spikesWindow = 0;
}

// ---- MEAS ----

static void measStart(uint32_t seconds) {
  // Read the chip's config first, so the one DRDY gap that costs stays out
  // of the run's statistics (the bench sketch did the same).
  const uint8_t cfg = cs1237ReadConfig();
  meas = Meas();
  meas.active = true;
  meas.startUs = (uint64_t)esp_timer_get_time();
  meas.durationUs = (uint64_t)seconds * 1000000ULL;
  meas.cfgOnChip = cfg;
  meas.drop0 = cs1237DroppedCount();
  meas.recfg0 = cs1237ReconfigCount();
  Serial.printf("measuring %lu s at %s %s, BLE %s%s ...\n", (unsigned long)seconds, rateLabel(),
                chanLabel(), bleLabel(), tareValid ? "" : " (no tare set - force shown is absolute)");
}

static void measFinish() {
  meas.active = false;
  const Stats &a = meas.acc;
  Serial.println("=== BEGIN RESULT ===");
  Serial.printf("config      0x%02X requested, 0x%02X on chip%s\n", CS1237_CONFIG_VALUE, meas.cfgOnChip,
                (meas.cfgOnChip == CS1237_CONFIG_VALUE) ? "" : "   <-- MISMATCH");
  Serial.printf("setting     %s  PGA%u  ch=%s\n", rateLabel(), pgaLabel(), chanLabel());
  Serial.printf("ble         %s\n", bleLabel());
  Serial.printf("duration    %lu ms\n", (unsigned long)(meas.durationUs / 1000ULL));
  Serial.printf("samples     %lu\n", (unsigned long)a.n);
  if (a.n < 2) {
    Serial.println("! no samples captured");
    Serial.println("=== END RESULT ===");
    return;
  }
  const double sps = a.sps();
  Serial.printf("eff rate    %.2f SPS (nominal %u)%s\n", sps, nominalRate(),
                (sps < nominalRate() * 0.9) ? "   <-- LOW" : "");
  Serial.printf("max gap     %lu us\n", (unsigned long)a.gapMax);
  Serial.printf("dropped     %lu\n", (unsigned long)(cs1237DroppedCount() - meas.drop0));
  Serial.printf("reverts     %lu\n", (unsigned long)(cs1237ReconfigCount() - meas.recfg0));
  Serial.printf("bad reads   %lu\n", (unsigned long)(a.rail + a.ones));
  Serial.printf("loop max    %lu us\n", (unsigned long)meas.loopMax);
  const double meanCounts = a.mean();
  const double net = meanCounts - (tareValid ? (double)tareCounts : 0.0);
  const double drift = (meas.firstN && meas.lastN)
                           ? (meas.lastSum / meas.lastN - meas.firstSum / meas.firstN)
                           : 0.0;
  Serial.printf("mean        %.0f counts\n", meanCounts);
  Serial.printf("tared       %.0f counts | %.3f N | %.4f kg%s\n", net, toNewtons(net),
                toNewtons(net) / NEWTONS_PER_KG, tareValid ? "" : "   (no tare: absolute)");
  Serial.printf("RMS noise   %.1f counts | %.4f N\n", a.rms(), toNewtons(a.rms()));
  Serial.printf("pk-pk       %ld counts | %.4f N\n", (long)a.pkpk(), toNewtons(a.pkpk()));
  Serial.printf("drift       %.0f counts | %.4f N  (first vs last 10%%)\n", drift, toNewtons(drift));
  Serial.println("=== END RESULT ===");
}

// ---- Sample pipeline ----

static void handleSample(const Cs1237Sample &s) {
  uint32_t gap = 0;
  if (lastSampleUs != 0) {
    if (s.tUs >= lastSampleUs) gap = (uint32_t)(s.tUs - lastSampleUs);
    else orderErrors++; // would print as ~4294967xxx us; count it instead
  }
  // Close the window on the first sample that falls outside it.
  if (pack.n > 0 && (s.tUs - pack.tFirst) >= windowUs) emitPack(s.tUs);
  pack.add(s.raw, s.tUs, gap);
  detectSpike(s.raw);
  lifetimeSamples++;
  if (s.raw == -1 || s.raw == 0x7FFFFF || s.raw == -0x800000) lifetimeBad++;

  if (meas.active) {
    const uint64_t el = s.tUs - meas.startUs; // samples queued before the command wrap huge -> skipped
    if (el < meas.durationUs) {
      meas.acc.add(s.raw, s.tUs, gap);
      if (el < meas.durationUs / 10) {
        meas.firstSum += s.raw;
        meas.firstN++;
      } else if (el > meas.durationUs * 9 / 10) {
        meas.lastSum += s.raw;
        meas.lastN++;
      }
    }
  }

  if (tareRemaining > 0) {
    tareSum += s.raw;
    if (--tareRemaining == 0) {
      tareCounts = (int32_t)(tareSum / TARE_SAMPLES);
      tareValid = true;
      Serial.printf("tare = %ld counts (mean of %d samples)\n", (long)tareCounts, TARE_SAMPLES);
    }
  }

  if (dumpRemaining > 0) {
    if (Serial.availableForWrite() >= DUMP_LINE_BYTES) {
      Serial.printf("%lu,%lu,%ld,0x%06lX\n", (unsigned long)dumpIndex, (unsigned long)gap,
                    (long)s.raw, (unsigned long)((uint32_t)s.raw & 0xFFFFFFUL));
    } else {
      dumpSkipped++;
    }
    dumpIndex++;
    if (--dumpRemaining == 0) {
      Serial.printf("# end (%lu lines skipped because the TX buffer was full)\n",
                    (unsigned long)dumpSkipped);
    }
  }

  lastSampleUs = s.tUs;

  // Real radio load: the production packet stream, only while connected.
  if (bleStarted && bleTx && isBleConnected()) {
    packetAppend(toNewtons((double)s.raw - (tareValid ? (double)tareCounts : 0.0)), s.tUs);
  } else {
    packetCount = 0;
  }
}

// ---- Reporting / commands ----

static void printStatus() {
  Serial.printf("STATUS: diag v%s | uptime %lu s | ADC running %d | stream %d win %lu ms | meas %d | lines %lu | "
                "samples %lu | drop %lu | recfg %lu | maxgap %lu us | maxburst %lu | maxloop %lu us | "
                "bad %lu order %lu spikes %lu | irq %lu spurious %lu miss %lu storm %lu | clk %lu us | "
                "ble %s pkts %lu | tare %ld (%s) | BTN %d USB %d | pins LATCH %d BTN %d USB %d LED %d SCLK %d DOUT %d\n",
                DIAG_VERSION, (unsigned long)uptimeS(), cs1237IsRunning() ? 1 : 0, streamOn ? 1 : 0,
                (unsigned long)(windowUs / 1000ULL), meas.active ? 1 : 0, (unsigned long)packSeq,
                (unsigned long)lifetimeSamples,
                (unsigned long)cs1237DroppedCount(), (unsigned long)cs1237ReconfigCount(),
                (unsigned long)cs1237MaxGapUs(), (unsigned long)maxBurst, (unsigned long)maxLoopGapUs,
                (unsigned long)lifetimeBad, (unsigned long)orderErrors, (unsigned long)spikes,
                (unsigned long)cs1237IsrCalls(), (unsigned long)cs1237IsrSpurious(),
                (unsigned long)cs1237EdgeMissCount(), (unsigned long)cs1237StormCount(),
                (unsigned long)cs1237SclkHalfUs(),
                bleLabel(), (unsigned long)packetsSent,
                (long)tareCounts, tareValid ? "valid" : "not set",
                digitalRead(POWER_BUTTON_PIN), isUsbPresent() ? 1 : 0,
                POWER_LATCH_PIN, POWER_BUTTON_PIN, USB_DETECT_PIN, STATUS_LED_PIN,
                CS1237_SCLK_PIN, CS1237_DATA_PIN);
}

static void printHelp() {
  Serial.println("Commands (newline-terminated):");
  Serial.println("  HELP        this list");
  Serial.println("  STATUS      one-shot summary with lifetime counters");
  Serial.println("  STREAM,0|1  pause / resume the stream");
  Serial.printf("  WIN,<ms>    averaging window per line, %d..%d ms (default %d; 25 = 40 lines/s)\n",
                WINDOW_MS_MIN, WINDOW_MS_MAX, WINDOW_MS_DEFAULT);
  Serial.printf("  MEAS[,<s>]  measurement run of s seconds (default %d, max %d) -> result block\n",
                MEAS_DEFAULT_S, MEAS_MAX_S);
  Serial.printf("  BLE,1       start the radio, advertise as %s (production BLE module); reset to stop\n",
                BLE_DEVICE_NAME);
  Serial.println("  TX,0|1      send protocol-v1 data packets while connected (default 1)");
  Serial.printf("  CLK,<us>    SCLK half-period 1..10 us (default %d); A/B under streaming, watch spikes\n",
                CS1237_SCLK_HALF_US);
  Serial.println("  CFG         read back the CS1237 config register (expect 0x7C)");
  Serial.printf("  TARE        average the next %d samples as zero\n", TARE_SAMPLES);
  Serial.printf("  DUMP,<n>    print the next n raw samples (1..%d) as idx,dt_us,raw,hex\n", DUMP_MAX_SAMPLES);
  Serial.println("  CLR         clear sketch-side maxima (burst, loop gap, bad-read count)");
  Serial.println("  OFF         release the power latch (same as holding the button)");
}

static void processCommand(const char *cmd) {
  notifyActivity();
  if (strcmp(cmd, "HELP") == 0 || strcmp(cmd, "?") == 0) {
    printHelp();
  } else if (strcmp(cmd, "STATUS") == 0) {
    printStatus();
  } else if (strncmp(cmd, "STREAM,", 7) == 0) {
    const char c = cmd[7];
    if (c == '0' || c == '1') {
      streamOn = (c == '1');
      Serial.printf("stream %s\n", streamOn ? "ON" : "OFF");
    } else {
      Serial.println("ERR:STREAM expects 0 or 1");
    }
  } else if (strncmp(cmd, "WIN,", 4) == 0) {
    const long ms = atol(cmd + 4);
    if (ms < WINDOW_MS_MIN || ms > WINDOW_MS_MAX) {
      Serial.printf("ERR:WIN expects %d..%d ms\n", WINDOW_MS_MIN, WINDOW_MS_MAX);
      return;
    }
    windowUs = (uint64_t)ms * 1000ULL;
    pack.reset();
    Serial.printf("window %ld ms (~%.1f lines/s)\n", ms, 1000.0 / ms);
  } else if (strcmp(cmd, "MEAS") == 0 || strncmp(cmd, "MEAS,", 5) == 0) {
    long s = MEAS_DEFAULT_S;
    if (cmd[4] == ',') s = atol(cmd + 5);
    if (s < 1 || s > MEAS_MAX_S) {
      Serial.printf("ERR:MEAS expects 1..%d seconds\n", MEAS_MAX_S);
      return;
    }
    if (!cs1237IsRunning()) { Serial.println("ERR:ADC not running"); return; }
    if (meas.active) { Serial.println("ERR:MEAS already running"); return; }
    measStart((uint32_t)s);
  } else if (strcmp(cmd, "CFG") == 0) {
    const uint8_t v = cs1237ReadConfig();
    const char *verdict = (v == CS1237_CONFIG_VALUE) ? "OK"
                          : (v == 0xFF) ? "*** 0xFF: timeout or chip not driving DOUT ***"
                                        : "*** MISMATCH ***";
    Serial.printf("config = 0x%02X (expected 0x%02X) %s\n", v, CS1237_CONFIG_VALUE, verdict);
  } else if (strcmp(cmd, "TARE") == 0) {
    if (!cs1237IsRunning()) { Serial.println("ERR:ADC not running"); return; }
    tareSum = 0;
    tareRemaining = TARE_SAMPLES;
    Serial.printf("taring over %d samples, keep the cell unloaded and still...\n", TARE_SAMPLES);
  } else if (strncmp(cmd, "DUMP,", 5) == 0) {
    const long n = atol(cmd + 5);
    if (n < 1 || n > DUMP_MAX_SAMPLES) {
      Serial.printf("ERR:DUMP expects 1..%d\n", DUMP_MAX_SAMPLES);
      return;
    }
    if (!cs1237IsRunning()) { Serial.println("ERR:ADC not running"); return; }
    dumpIndex = 0;
    dumpSkipped = 0;
    dumpRemaining = (uint32_t)n;
    Serial.println("# idx,dt_us,raw,hex");
  } else if (strncmp(cmd, "BLE,", 4) == 0) {
    const char c = cmd[4];
    if (c == '1') {
      if (bleStarted) { Serial.println("BLE already running"); return; }
      // Bluedroid init blocks the loop for a few hundred ms and touches
      // flash: expect one !GAP (maybe !DROP / miss) on this line, once.
      initializeBLE();
      bleStarted = true;
      Serial.printf("BLE up, advertising as %s. nRF Connect: connect, request MTU 517, enable "
                    "notifications on ...0002 (packets %u B, ~%u/s). Reset to turn the radio off again.\n",
                    BLE_DEVICE_NAME, (unsigned)(12 + 4 * SAMPLES_PER_PACKET),
                    (unsigned)(nominalRate() / SAMPLES_PER_PACKET));
    } else if (c == '0') {
      Serial.println("ERR:the radio cannot be stopped cleanly (Bluedroid); reset or power-cycle for radio-off");
    } else {
      Serial.println("ERR:BLE expects 1");
    }
  } else if (strncmp(cmd, "TX,", 3) == 0) {
    const char c = cmd[3];
    if (c == '0' || c == '1') {
      bleTx = (c == '1');
      packetCount = 0;
      Serial.printf("packet stream while connected %s\n", bleTx ? "ON" : "OFF");
    } else {
      Serial.println("ERR:TX expects 0 or 1");
    }
  } else if (strncmp(cmd, "CLK,", 4) == 0) {
    const long us = atol(cmd + 4);
    if (us < 1 || us > 10) {
      Serial.println("ERR:CLK expects 1..10 us");
      return;
    }
    cs1237SetSclkHalfUs((uint32_t)us);
    Serial.printf("SCLK half-period %lu us (27-pulse read = %lu us); spikes so far %lu\n",
                  (unsigned long)cs1237SclkHalfUs(), (unsigned long)(27 * 2 * cs1237SclkHalfUs()),
                  (unsigned long)spikes);
  } else if (strcmp(cmd, "CLR") == 0) {
    maxBurst = 0;
    maxLoopGapUs = 0;
    lifetimeBad = 0;
    orderErrors = 0;
    spikes = 0;
    Serial.println("OK:CLR (driver counters drop/recfg/maxgap are lifetime and stay)");
  } else if (strcmp(cmd, "OFF") == 0) {
    Serial.println("OK:OFF");
    Serial.flush();
    powerOffNow();
  } else {
    Serial.println("ERR:unknown command (HELP)");
  }
}

static void handleSerialInput() {
  static char line[48];
  static size_t len = 0;
  while (Serial.available()) {
    const char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (len > 0) {
        line[len] = '\0';
        processCommand(line);
        len = 0;
      }
    } else if (len < sizeof(line) - 1) {
      line[len++] = c;
    }
  }
}

// Logs every debounced change of the raw GPIO36 level so the button
// polarity (an open item: active-HIGH is assumed) can be read off the log.
static void logButtonLevel() {
  static int lastLogged = -1;
  static int pending = -1;
  static uint32_t pendingSinceMs = 0;
  const int level = digitalRead(POWER_BUTTON_PIN);
  const uint32_t now = millis();
  if (level != pending) {
    pending = level;
    pendingSinceMs = now;
  }
  if (level != lastLogged && (now - pendingSinceMs) >= BUTTON_DEBOUNCE_MS) {
    lastLogged = level;
    Serial.printf("BTN raw level -> %s at %lu ms\n", level ? "HIGH" : "LOW", (unsigned long)now);
  }
}

// If the ADC never configured: say so once a second and retry on a timer.
// cs1237Begin() is bounded (about 2.5 s worst case) and nothing is
// streaming while it is down.
static void serviceAdcRetry() {
  if (cs1237IsRunning()) return;
  const uint32_t now = millis();
  if (now - lastNotRunningNoticeMs >= NOT_RUNNING_NOTICE_MS) {
    lastNotRunningNoticeMs = now;
    Serial.printf("ADC NOT RUNNING - config never verified, retrying every %d s | BTN %d USB %d\n",
                  ADC_RETRY_MS / 1000, digitalRead(POWER_BUTTON_PIN), isUsbPresent() ? 1 : 0);
  }
  if (now - lastAdcRetryMs < ADC_RETRY_MS) return;
  lastAdcRetryMs = now;
  Serial.println("CS1237: retrying configuration");
  if (cs1237Begin()) {
    lastSampleUs = 0;
    pack.reset();
  }
}

void setup() {
  powerManagerInit();
  Serial.setTxBufferSize(SERIAL_TX_BUFFER);
  Serial.begin(SERIAL_BAUD);

  setupStatusLed();
  serviceStatusLed(false, false, false); // blue right away: the device is on

  Serial.println();
  Serial.printf("NexoHHD SignalChainTest v%s (main FW %s)\n", DIAG_VERSION, FW_VERSION);
  Serial.printf("pins: LATCH %d  BTN %d  USB %d  LED %d  SCLK %d  DOUT %d\n",
                POWER_LATCH_PIN, POWER_BUTTON_PIN, USB_DETECT_PIN, STATUS_LED_PIN,
                CS1237_SCLK_PIN, CS1237_DATA_PIN);
  Serial.printf("boot levels: BTN %d (active-HIGH assumed)  USB %d\n",
                digitalRead(POWER_BUTTON_PIN), isUsbPresent() ? 1 : 0);

  // Wait for the boot press to be released, BOUNDED. Done before the ADC
  // starts so a held button cannot overflow the ring and fake a drop count.
  const uint32_t releaseDeadline = millis() + BOOT_RELEASE_TIMEOUT_MS;
  while (digitalRead(POWER_BUTTON_PIN) == HIGH && millis() < releaseDeadline) {
    serviceStatusLed(false, false, false);
    delay(10);
  }
  if (digitalRead(POWER_BUTTON_PIN) == HIGH) {
    Serial.println("WARN: BTN still HIGH 5 s after boot: either held down, or GPIO36 is active-LOW (see README)");
  }

  if (!cs1237Begin()) {
    Serial.println("FATAL: CS1237 did not configure - check SCLK (13), DOUT (14), AVDD/DVDD; retrying every 3 s");
  }

  restoreBrownoutDetector();
  lastLoopUs = (uint64_t)esp_timer_get_time();
  Serial.printf("streaming one line per %d ms window (~%u samples at %u Hz). WIN,<ms> changes it; HELP for commands.\n",
                WINDOW_MS_DEFAULT, (unsigned)((uint32_t)nominalRate() * WINDOW_MS_DEFAULT / 1000UL),
                (unsigned)nominalRate());
  Serial.println("BLE is OFF (radio-off baseline). BLE,1 starts advertising; connect to measure under streaming load.");
}

void loop() {
  // Loop stall watchdog: worst gap between passes, per pack / per run / lifetime.
  const uint64_t nowUs = (uint64_t)esp_timer_get_time();
  const uint32_t loopGap = (uint32_t)(nowUs - lastLoopUs);
  lastLoopUs = nowUs;
  if (loopGap > packLoopGapMax) packLoopGapMax = loopGap;
  if (loopGap > maxLoopGapUs) maxLoopGapUs = loopGap;
  if (meas.active && loopGap > meas.loopMax) meas.loopMax = loopGap;

  servicePowerManager(false); // latch, button long-press, inactivity auto-off
  cs1237Service();            // config-revert watchdog
  serviceStatusLed(false, false, false); // blue; red override comes from PowerManager
  logButtonLevel();
  serviceAdcRetry();

  // Drain the ring completely every pass. The burst size is a direct read
  // of ring occupancy, i.e. of how late this loop pass was.
  Cs1237Sample s;
  uint32_t burst = 0;
  while (cs1237PopSample(s)) {
    handleSample(s);
    burst++;
  }
  if (burst > maxBurst) maxBurst = burst;

  if (meas.active && (nowUs - meas.startUs) >= meas.durationUs) measFinish();

  if (bleStarted) {
    serviceBLE(); // deferred advertising restart after a disconnect
    char cmd[48];
    while (getBleCommand(cmd, sizeof(cmd))) processCommand(cmd); // control char -> same handler
  }
  handleSerialInput();
}
