#include "Cs1237.h"
#include "Config.h"
#include <Arduino.h>
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include "esp_timer.h"
#include "esp_rom_sys.h"

// ============================================================
// CS1237 two-wire driver.
//
// Design rules carried over from the ErgoJump post-mortem:
//  - Timestamp in the ISR, decide in the loop. The ISR only reads the
//    conversion, timestamps it and enqueues it.
//  - Queue every sample; never keep just the latest. Ring overflow drops
//    the oldest and increments a counter that is REPORTED over BLE.
//  - No delay() on any steady-state path. The only bounded waits live in
//    begin() and in the rare recoveries.
//
// CS1237 electrical constraints honored here:
//  - SCLK min pulse 455 ns; SCLK must NEVER stay high > 100 us or the
//    chip powers down. Every high phase is wrapped in a critical section
//    so it cannot be stretched by preemption.
//  - DOUT is valid while SCLK is high (sample during the high phase, not
//    after the falling edge).
//  - Register ops are a 46-pulse sequence: 24 data bits, pulses 25-27
//    (DRDY reset / DOUT forced high), 28-29 (direction switch), 30-36
//    (7-bit command, MSB first: 0x65 write / 0x56 read), 37 (turnaround),
//    38-45 (register byte), 46 (finish, DOUT back to the chip).
//
// DRDY handling (revised after the first hardware run, 2026-09-03):
//  - DRDY is a LEVEL: DOUT goes low when a result is ready and stays low
//    until it is clocked out. A falling-EDGE interrupt therefore has a
//    failure mode with no recovery: miss one edge (interrupt not yet armed,
//    masked during a flash write, or whatever the first run hit) and DOUT
//    sits low forever with no further edges. On the bench that showed up
//    as polling working perfectly while the ISR never delivered a sample.
//    The interrupt is now LOW-LEVEL triggered: as long as DOUT is low the
//    ISR runs, reads the result (which raises DOUT) and the condition
//    clears itself. Nothing can be "missed".
//  - Storm guard: a level interrupt on a line stuck low (unpowered or
//    shorted chip) would re-fire forever and starve the loop. If DOUT is
//    still low right after the DRDY-reset pulses a few times in a row, the
//    ISR cuts the pin's interrupt routing with a plain register write and
//    the service loop runs a bounded, rate-limited recovery.
//  - Loop-side fallback: if DOUT has been low for longer than ~1.5 sample
//    intervals without an ISR sample, the service loop reads it itself so
//    data keeps flowing even with no working interrupt. Every such read is
//    COUNTED (edge-miss) and reported: it is a fault signature, not a
//    normal path.
//  - Diagnostics for all of this ride in STATUS: ISR entries, spurious
//    entries, loop-side reads, storm recoveries.
//
// Register-op pulse grouping cross-checked (2026-09-03) against the bench
// bring-up sketch that produced the 0.096 N RMS measurement
// (Downloads/cs1237_bringup/cs1237_bringup.ino, cs1237_xfer()):
// 24 + 3 + 2 + 7 + 1 + 8 + 1 = 46, identical. Deliberate differences:
//  - DOUT idles as INPUT_PULLUP, same as the bench (a chip that is not
//    driving the line then reads 0xFF, which the readback check catches).
//  - The bench switches DOUT to OUTPUT before pulses 28-29 and keeps
//    driving it through pulse 37; this driver switches after pulse 29 and,
//    for a read, releases before pulse 37 (the datasheet's turnaround
//    pulse). Both keep the command on pulses 30-36 and the register byte
//    on 38-45, which is what the chip decodes.
//  - Command/data bits are set up before the rising edge and held through
//    the high phase, so they are valid at the falling edge exactly as the
//    bench's "drive while SCLK high" was.
// First hardware run confirmed the register path: write, readback 0x7C and
// polled conversions all worked on the production pins.
// ============================================================

#define CS1237_CMD_WRITE 0x65
#define CS1237_CMD_READ 0x56

// Direct register access: both pins are < 32, and these are the only
// IRAM-safe, single-instruction ways to toggle/read them from the ISR.
#define SCLK_BIT (1UL << CS1237_SCLK_PIN)
#define DATA_BIT (1UL << CS1237_DATA_PIN)
#define SCLK_HIGH() (GPIO.out_w1ts = SCLK_BIT)
#define SCLK_LOW() (GPIO.out_w1tc = SCLK_BIT)
#define DATA_READ() ((GPIO.in >> CS1237_DATA_PIN) & 1U)
#define DATA_HIGH() (GPIO.out_w1ts = DATA_BIT)
#define DATA_LOW() (GPIO.out_w1tc = DATA_BIT)

static portMUX_TYPE ringMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE sclkMux = portMUX_INITIALIZER_UNLOCKED;
static portMUX_TYPE fallbackMux = portMUX_INITIALIZER_UNLOCKED;

static volatile Cs1237Sample ring[SAMPLE_RING_SIZE];
static volatile uint16_t ringHead = 0; // next write (ISR)
static volatile uint16_t ringTail = 0; // next read (loop)
static volatile uint32_t droppedCount = 0;
static volatile uint64_t lastSampleUs = 0;
static volatile uint32_t maxGapUs = 0;

// DRDY path diagnostics.
static volatile uint32_t isrCalls = 0;
static volatile uint32_t isrSpurious = 0;
static volatile uint32_t stuckLowStreak = 0;
static volatile bool isrStormCut = false;
static uint32_t edgeMissCount = 0;
static uint32_t stormCount = 0;
static uint64_t lastStormRecoveryUs = 0;

static uint32_t reconfigCount = 0;
static bool running = false;
static uint64_t lastWatchdogCheckUs = 0;

// SCLK half-period (high time = low time). At 1 us the first BLE run
// showed rare single-bit read errors (sign bit and bits 10-13) only while
// the radio was active: on-time reads, no timing gaps, just a bit read on
// the wrong side of the threshold. The bench sketch clocked at ~3-4 us per
// phase and never did. Runtime-settable so the diagnostic sketch can A/B
// it under streaming load; see CS1237_SCLK_HALF_US.
static volatile uint32_t sclkHalfUs = CS1237_SCLK_HALF_US;

// --- Task-context bit-banging (config ops; DRDY interrupt detached) ---

// The critical section bounds the SCLK-high window so a context switch
// can never stretch it past the 100 us power-down threshold.
static int pulseReadBit() {
  portENTER_CRITICAL(&sclkMux);
  SCLK_HIGH();
  esp_rom_delay_us(sclkHalfUs);
  const int bit = (int)DATA_READ();
  SCLK_LOW();
  portEXIT_CRITICAL(&sclkMux);
  esp_rom_delay_us(sclkHalfUs);
  return bit;
}

static void pulseWriteBit(int bit) {
  portENTER_CRITICAL(&sclkMux);
  if (bit) DATA_HIGH();
  else DATA_LOW();
  esp_rom_delay_us(sclkHalfUs); // data setup before rising edge
  SCLK_HIGH();
  esp_rom_delay_us(sclkHalfUs);
  SCLK_LOW();
  portEXIT_CRITICAL(&sclkMux);
  esp_rom_delay_us(sclkHalfUs);
}

// Waits for DOUT low (conversion ready). Only used in begin()/recovery,
// never on the steady-state path. Returns false on timeout.
static bool waitForReady(uint32_t timeoutMs) {
  const uint64_t deadline = (uint64_t)esp_timer_get_time() + (uint64_t)timeoutMs * 1000ULL;
  while (DATA_READ() != 0) {
    if ((uint64_t)esp_timer_get_time() > deadline) return false;
    esp_rom_delay_us(50);
  }
  return true;
}

// Full 46-pulse register operation. Caller must have the DRDY interrupt
// detached and a conversion ready (DOUT low). Returns the byte read
// (meaningful for CMD_READ; for CMD_WRITE returns `value` back).
static uint8_t registerOp(uint8_t command, uint8_t value) {
  // Pulses 1-24: discard the pending conversion.
  for (int i = 0; i < 24; i++) (void)pulseReadBit();
  // Pulses 25-27: DRDY reset, DOUT forced high.
  for (int i = 0; i < 3; i++) (void)pulseReadBit();
  // Pulses 28-29: the chip prepares for a direction switch.
  (void)pulseReadBit();
  (void)pulseReadBit();
  // Pulses 30-36: 7-bit command, MSB first, MCU drives the line.
  pinMode(CS1237_DATA_PIN, OUTPUT);
  for (int i = 6; i >= 0; i--) pulseWriteBit((command >> i) & 1);
  uint8_t result = value;
  if (command == CS1237_CMD_WRITE) {
    // Pulse 37: turnaround, MCU keeps the line.
    pulseWriteBit(1);
    // Pulses 38-45: register byte, MSB first.
    for (int i = 7; i >= 0; i--) pulseWriteBit((value >> i) & 1);
    // Pulse 46: finish; release the line back to the chip.
    pulseWriteBit(1);
    pinMode(CS1237_DATA_PIN, INPUT_PULLUP);
  } else {
    // Pulse 37: turnaround, chip takes the line back.
    pinMode(CS1237_DATA_PIN, INPUT_PULLUP);
    (void)pulseReadBit();
    // Pulses 38-45: register byte from the chip.
    result = 0;
    for (int i = 0; i < 8; i++) result = (uint8_t)((result << 1) | pulseReadBit());
    // Pulse 46: finish.
    (void)pulseReadBit();
  }
  return result;
}

// --- Shared conversion read (ISR and loop-side fallback) ---

// Clocks one conversion out: 24 data pulses, then pulses 25-27 which reset
// DRDY and force DOUT high until the next result. Per-pulse critical
// sections bound SCLK-high while still letting higher-priority interrupts
// run between pulses. Safe from ISR and task context.
static int32_t IRAM_ATTR clockOutConversion() {
  uint32_t bits = 0;
  for (int i = 0; i < 27; i++) {
    portENTER_CRITICAL_SAFE(&sclkMux);
    SCLK_HIGH();
    esp_rom_delay_us(sclkHalfUs);
    if (i < 24) bits = (bits << 1) | DATA_READ();
    SCLK_LOW();
    portEXIT_CRITICAL_SAFE(&sclkMux);
    esp_rom_delay_us(sclkHalfUs);
  }
  int32_t raw = (int32_t)bits;
  if (raw & 0x800000) raw |= 0xFF000000; // sign-extend 24 -> 32
  return raw;
}

static void IRAM_ATTR enqueueSample(int32_t raw, uint64_t t) {
  portENTER_CRITICAL_SAFE(&ringMux);
  if (lastSampleUs != 0) {
    const uint32_t gap = (uint32_t)(t - lastSampleUs);
    if (gap > maxGapUs) maxGapUs = gap;
  }
  lastSampleUs = t;
  const uint16_t next = (uint16_t)((ringHead + 1) % SAMPLE_RING_SIZE);
  if (next == ringTail) {
    // Overflow: drop the oldest so the newest data always survives.
    ringTail = (uint16_t)((ringTail + 1) % SAMPLE_RING_SIZE);
    droppedCount = droppedCount + 1;
  }
  ring[ringHead].raw = raw;
  ring[ringHead].tUs = t;
  ringHead = next;
  portEXIT_CRITICAL_SAFE(&ringMux);
}

// --- DRDY interrupt (low level): timestamp, read, enqueue ---

static void IRAM_ATTR drdyIsr() {
  isrCalls = isrCalls + 1;
  // The dispatcher clears the pin status after we return; our own clocking
  // toggles DOUT, so a re-entry with the line already high can happen.
  // Count it and ignore it.
  if (DATA_READ() != 0) {
    isrSpurious = isrSpurious + 1;
    return;
  }
  const uint64_t t = (uint64_t)esp_timer_get_time();
  const int32_t raw = clockOutConversion();
  if (DATA_READ() == 0) {
    // Still asserted right after the DRDY-reset pulses: the line is stuck
    // low. Cut this pin's interrupt routing (plain register write,
    // IRAM-safe) before the level trigger starves the loop; the service
    // loop recovers.
    stuckLowStreak = stuckLowStreak + 1;
    if (stuckLowStreak >= CS1237_STORM_LIMIT) {
      GPIO.pin[CS1237_DATA_PIN].int_ena = 0;
      isrStormCut = true;
      return;
    }
  } else {
    stuckLowStreak = 0;
  }
  enqueueSample(raw, t);
}

// --- Configuration / recovery ---

// Writes the config, verifies it by readback, then settles and discards
// the required conversions. Interrupt must be detached. Bounded: worst
// case ~= timeouts below, only ever runs at boot or after a fault.
static bool configureChip() {
  // A chip that powered down (SCLK held high) wakes when SCLK goes low.
  SCLK_LOW();
  delayMicroseconds(20);

  if (!waitForReady(300)) return false; // 10 Hz default rate: first result can take >100 ms
  registerOp(CS1237_CMD_WRITE, CS1237_CONFIG_VALUE);

  if (!waitForReady(300)) return false;
  const uint8_t readback = registerOp(CS1237_CMD_READ, 0);
  if (readback != CS1237_CONFIG_VALUE) {
    Serial.printf("CS1237: config readback 0x%02X, expected 0x%02X\n", readback, CS1237_CONFIG_VALUE);
    return false;
  }

  delay(CS1237_SETTLE_MS); // analog settling after config change
  for (int i = 0; i < CS1237_DISCARD_AFTER_CONFIG; i++) {
    if (!waitForReady(50)) return false;
    (void)clockOutConversion(); // discard
  }
  return true;
}

static void attachDrdy() {
  portENTER_CRITICAL(&ringMux);
  lastSampleUs = (uint64_t)esp_timer_get_time(); // don't trip the watchdog on attach
  portEXIT_CRITICAL(&ringMux);
  stuckLowStreak = 0;
  isrStormCut = false;
  // Low-level trigger: if a result is already waiting when this arms, the
  // ISR runs immediately instead of waiting for an edge that never comes.
  attachInterrupt(digitalPinToInterrupt(CS1237_DATA_PIN), drdyIsr, ONLOW);
}

bool cs1237Begin() {
  pinMode(CS1237_SCLK_PIN, OUTPUT);
  SCLK_LOW();
  pinMode(CS1237_DATA_PIN, INPUT_PULLUP);

  // attachInterrupt() swallows a failure to start the GPIO ISR service
  // (log_e only, silent at the default debug level). Install it here so a
  // failure is loud; the loop-side fallback keeps data flowing regardless.
  const esp_err_t isrErr = gpio_install_isr_service((int)ARDUINO_ISR_FLAG);
  if (isrErr != ESP_OK && isrErr != ESP_ERR_INVALID_STATE) {
    Serial.printf("CS1237: FATAL gpio_install_isr_service failed (err %d): DRDY interrupt unavailable, loop-side reads only\n",
                  (int)isrErr);
  }

  bool ok = false;
  for (int attempt = 0; attempt < 3 && !ok; attempt++) {
    ok = configureChip();
    if (!ok) Serial.printf("CS1237: configure attempt %d failed\n", attempt + 1);
  }
  if (!ok) {
    running = false;
    return false;
  }
  attachDrdy();
  running = true;
  Serial.printf("CS1237: configured 0x7C (1280 Hz, PGA 128, ext ref); DRDY irq type=%u ena=0x%02X (expect 4 / non-zero)\n",
                (unsigned)GPIO.pin[CS1237_DATA_PIN].int_type, (unsigned)GPIO.pin[CS1237_DATA_PIN].int_ena);
  return true;
}

void cs1237Service() {
  if (!running) return;

  // ORDERING RULE for every staleness test in here: read lastSampleUs
  // BEFORE reading the clock. The ISR only ever moves lastSampleUs
  // forward, so "last then now" guarantees now >= last and the unsigned
  // subtraction cannot wrap. The first bench run had it the other way
  // round: the ISR landed between the two reads, now - last wrapped to
  // ~2^64, the fallback clocked a phantom conversion out of a chip with
  // nothing ready (stamped a few microseconds in the past, value garbage),
  // and the watchdog fired reconfigures every ~20 s for no reason.

  // 1. Loop-side fallback, every pass. DOUT low means a result is waiting;
  // if the ISR has not taken it within ~1.5 intervals the interrupt did
  // not run (not attached, cut by the storm guard, masked during a flash
  // write). The whole decision and the read run with interrupts off, so
  // the ISR cannot take the conversion between our check and our clocking.
  // Every such read is COUNTED: it is a fault signature.
  int32_t raw = 0;
  uint64_t t = 0;
  bool tookOne = false;
  portENTER_CRITICAL(&fallbackMux);
  if (DATA_READ() == 0) {
    const uint64_t last = lastSampleUs; // interrupts off: consistent 64-bit read
    t = (uint64_t)esp_timer_get_time();  // after `last`, so t >= last
    if (t - last > CS1237_MISSED_DRDY_US) {
      raw = clockOutConversion();
      tookOne = true;
    }
  }
  portEXIT_CRITICAL(&fallbackMux);
  if (tookOne) {
    enqueueSample(raw, t);
    edgeMissCount++;
  }

  portENTER_CRITICAL(&ringMux);
  const uint64_t last = lastSampleUs;
  portEXIT_CRITICAL(&ringMux);
  const uint64_t now = (uint64_t)esp_timer_get_time(); // after `last`

  // 2. Storm guard tripped: bounded recovery, rate-limited so a dead line
  // cannot turn the loop into a reconfigure treadmill.
  if (isrStormCut) {
    if (now - lastStormRecoveryUs < 250000ULL) return;
    lastStormRecoveryUs = now;
    stormCount++;
    detachInterrupt(digitalPinToInterrupt(CS1237_DATA_PIN));
    const bool ok = configureChip();
    attachDrdy();
    Serial.printf("CS1237: DOUT stuck low, interrupt cut; recovery %s (storms=%lu)\n",
                  ok ? "ok" : "FAILED", (unsigned long)stormCount);
    return;
  }

  // 3. Config-revert watchdog, checked every 10 ms.
  if (now - lastWatchdogCheckUs < 10000) return;
  lastWatchdogCheckUs = now;

  if (now - last <= CS1237_WATCHDOG_US) return;

  // No sample for 50 ms at a nominal 0.78 ms interval: the config register
  // reverted on a supply dip (now converting at 10 Hz) or the chip powered
  // down. Rewrite the config and resume. This must exist in production:
  // without it a fielded unit reports plausible force values at 1/128th of
  // the intended rate and nobody can tell from the numbers alone.
  detachInterrupt(digitalPinToInterrupt(CS1237_DATA_PIN));
  const bool ok = configureChip();
  attachDrdy();
  if (ok) {
    reconfigCount++;
    Serial.printf("CS1237: config revert recovered (count=%lu)\n", (unsigned long)reconfigCount);
  }
  // On failure the watchdog simply fires again next window.
}

uint8_t cs1237ReadConfig() {
  // One 46-pulse read with DRDY detached. Bounded by the 300 ms ready
  // wait; the conversion in flight is consumed by the op, so expect one
  // inter-sample gap of roughly two intervals. Not a steady-state path.
  const bool wasRunning = running;
  if (wasRunning) detachInterrupt(digitalPinToInterrupt(CS1237_DATA_PIN));
  uint8_t value = 0xFF;
  if (waitForReady(300)) value = registerOp(CS1237_CMD_READ, 0);
  if (wasRunning) attachDrdy();
  return value;
}

bool cs1237PopSample(Cs1237Sample &s) {
  bool have = false;
  portENTER_CRITICAL(&ringMux);
  if (ringTail != ringHead) {
    s.raw = ring[ringTail].raw;
    s.tUs = ring[ringTail].tUs;
    ringTail = (uint16_t)((ringTail + 1) % SAMPLE_RING_SIZE);
    have = true;
  }
  portEXIT_CRITICAL(&ringMux);
  return have;
}

uint32_t cs1237DroppedCount() {
  portENTER_CRITICAL(&ringMux);
  const uint32_t v = droppedCount;
  portEXIT_CRITICAL(&ringMux);
  return v;
}

uint32_t cs1237ReconfigCount() { return reconfigCount; }

uint32_t cs1237MaxGapUs() {
  portENTER_CRITICAL(&ringMux);
  const uint32_t v = maxGapUs;
  portEXIT_CRITICAL(&ringMux);
  return v;
}

void cs1237SetSclkHalfUs(uint32_t us) {
  if (us < 1) us = 1;
  if (us > 10) us = 10; // 27 pulses x 20 us = 540 us, still inside one interval
  sclkHalfUs = us;
}

uint32_t cs1237SclkHalfUs() { return sclkHalfUs; }

uint32_t cs1237IsrCalls() { return isrCalls; }
uint32_t cs1237IsrSpurious() { return isrSpurious; }
uint32_t cs1237EdgeMissCount() { return edgeMissCount; }
uint32_t cs1237StormCount() { return stormCount; }

bool cs1237IsRunning() { return running; }
