// ============================================================
// NexoHHD - handheld dynamometer firmware
//
// Load cell -> CS1237 (1280 Hz, PGA 128) -> Newtons -> BLE stream.
//
// Ground rules (from the ErgoJump post-mortem):
//  - No delay() on any steady-state path. Housekeeping is state machines;
//    nothing is "paused during measurement" because nothing ever blocks.
//  - Timestamps come from the DRDY ISR via esp_timer_get_time() (64-bit),
//    never from loop-observation time, never micros().
//  - Dropped samples, config-revert recoveries and worst gaps are part of
//    the data contract: they ride in every packet / status report.
//  - Commands execute on the loop task only; BLE callbacks just enqueue.
//  - Out-of-range inputs fail loudly (ERR:...), never silently.
// ============================================================

#include "Config.h"
#include "Cs1237.h"
#include "ForceSensor.h"
#include "BleService.h"
#include "BatteryMonitor.h"
#include "LedController.h"
#include "PowerManager.h"
#include <Arduino.h>
#include "esp_timer.h"

// --- Streaming state ---
static bool streaming = false;
static bool rawMode = false; // stream raw counts instead of Newtons (calibration aid)
static uint8_t decimation = DEFAULT_DECIMATION;
static uint8_t decimCounter = 0;

// --- Packet assembly (protocol v1, PROVISIONAL until app team confirms) ---
// Header (12 bytes, little-endian):
//   0     uint8  protocol version
//   1     uint8  flags: bit0 = config-revert recovery since last packet
//                       bit1 = samples dropped since last packet
//                       bit2 = payload is raw counts, not Newtons
//   2-3   uint16 sequence number (app detects gaps itself)
//   4-7   uint32 timestamp of first sample, us (low 32 bits of 64-bit clock)
//   8-9   uint16 total dropped samples (saturating)
//   10    uint8  sample count
//   11    uint8  reserved (0)
// Payload: count x float32 (Newtons, or raw counts when flag bit2 is set).
static uint8_t packetBuf[12 + 4 * SAMPLES_PER_PACKET];
static uint8_t packetCount = 0;
static uint64_t packetFirstUs = 0;
static uint16_t packetSeq = 0;
static uint32_t lastPacketDropped = 0;
static uint32_t lastPacketReconfigs = 0;

// --- Loop stall watchdog (diagnostic guardrail, in from the first commit:
// this is what would have caught ErgoJump's battery-read stall the day it
// was introduced instead of two commits later) ---
static uint64_t lastLoopUs = 0;
static uint32_t maxLoopGapUs = 0;

static void sendPacket() {
  const uint32_t dropped = cs1237DroppedCount();
  const uint32_t reconfigs = cs1237ReconfigCount();
  uint8_t flags = 0;
  if (reconfigs != lastPacketReconfigs) flags |= 0x01;
  if (dropped != lastPacketDropped) flags |= 0x02;
  if (rawMode) flags |= 0x04;
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
  packetCount = 0;
}

static void packetAppend(float value, uint64_t tUs) {
  if (packetCount == 0) packetFirstUs = tUs;
  memcpy(&packetBuf[12 + 4 * (size_t)packetCount], &value, 4);
  if (++packetCount >= SAMPLES_PER_PACKET) sendPacket();
}

static void startStreaming() {
  decimCounter = 0;
  packetCount = 0;
  streaming = true;
}

static void stopStreaming() {
  if (packetCount > 0) sendPacket(); // flush the partial packet
  streaming = false;
}

static void sendStatus() {
  char buf[300];
  snprintf(buf, sizeof(buf),
           "FW=%s ADC=%d STREAM=%d RAW=%d DEC=%u N=%.2f BAT=%.2f USB=%d "
           "DROP=%lu RECFG=%lu GAP=%lu LOOP=%lu IRQ=%lu SPUR=%lu MISS=%lu STORM=%lu "
           "CALOFF=%ld CALSC=%.4e",
           FW_VERSION, cs1237IsRunning() ? 1 : 0, streaming ? 1 : 0,
           rawMode ? 1 : 0, (unsigned)decimation, forceLastNewtons(),
           batteryVoltage(), isUsbPresent() ? 1 : 0,
           (unsigned long)cs1237DroppedCount(),
           (unsigned long)cs1237ReconfigCount(),
           (unsigned long)cs1237MaxGapUs(), (unsigned long)maxLoopGapUs,
           (unsigned long)cs1237IsrCalls(), (unsigned long)cs1237IsrSpurious(),
           (unsigned long)cs1237EdgeMissCount(), (unsigned long)cs1237StormCount(),
           (long)forceCalOffset(), forceCalScale());
  bleNotifyStatus(buf);
  Serial.println(buf);
}

static void reply(const char *text) {
  bleNotifyStatus(text);
  Serial.println(text);
}

static void processCommand(const char *cmd) {
  notifyActivity();
  if (strcmp(cmd, "START") == 0) {
    if (!cs1237IsRunning()) { reply("ERR:ADC not running"); return; }
    startStreaming();
    reply("OK:START");
  } else if (strcmp(cmd, "STOP") == 0) {
    stopStreaming();
    reply("OK:STOP");
  } else if (strcmp(cmd, "TARE") == 0) {
    if (forceStartCapture(FORCE_CAPTURE_TARE, 0)) reply("OK:TARE capturing");
    else reply(forceTakeCaptureResult());
  } else if (strcmp(cmd, "CAL0") == 0) {
    if (forceStartCapture(FORCE_CAPTURE_CAL_ZERO, 0)) reply("OK:CAL0 capturing");
    else reply(forceTakeCaptureResult());
  } else if (strncmp(cmd, "CALW,", 5) == 0) {
    const float ref = atof(cmd + 5);
    if (forceStartCapture(FORCE_CAPTURE_CAL_SCALE, ref)) reply("OK:CALW capturing");
    else reply(forceTakeCaptureResult());
  } else if (strncmp(cmd, "DEC,", 4) == 0) {
    const long d = atol(cmd + 4);
    if (d >= 1 && d <= MAX_DECIMATION) {
      decimation = (uint8_t)d;
      decimCounter = 0;
      reply("OK:DEC");
    } else {
      reply("ERR:DEC out of range (1..32)");
    }
  } else if (strncmp(cmd, "RAW,", 4) == 0) {
    const char c = cmd[4];
    if (c == '0' || c == '1') { rawMode = (c == '1'); reply("OK:RAW"); }
    else reply("ERR:RAW expects 0 or 1");
  } else if (strcmp(cmd, "STATUS") == 0) {
    sendStatus();
  } else if (strcmp(cmd, "OFF") == 0) {
    reply("OK:OFF");
    powerOffNow();
  } else {
    reply("ERR:unknown command");
  }
}

// Non-blocking serial line reader feeding the same command handler.
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

void setup() {
  powerManagerInit();
  Serial.begin(115200);

  forceSensorInit();
  setupBatteryMonitor();
  setupStatusLed();

  if (!cs1237Begin()) {
    Serial.println("FATAL: CS1237 did not configure - check wiring");
    // Device stays up (LED/BLE/power still work) so the fault is visible
    // and reportable; STATUS shows ADC=0.
  }

  initializeBLE();

  // Wait for the boot button press to be released, BOUNDED (ErgoJump's
  // unbounded wait hung setup forever on a stuck button).
  const uint32_t releaseDeadline = millis() + 5000;
  while (digitalRead(POWER_BUTTON_PIN) == HIGH && millis() < releaseDeadline) {
    delay(10);
  }

  restoreBrownoutDetector();
  lastLoopUs = (uint64_t)esp_timer_get_time();
  Serial.println("NexoHHD ready");
}

void loop() {
  // Loop stall watchdog: record the worst gap between passes. Reported in
  // STATUS; any future "harmless" blocking addition shows up here at once.
  const uint64_t nowUs = (uint64_t)esp_timer_get_time();
  const uint32_t gap = (uint32_t)(nowUs - lastLoopUs);
  if (gap > maxLoopGapUs) maxLoopGapUs = gap;
  lastLoopUs = nowUs;

  const bool connected = isBleConnected();

  servicePowerManager(connected || streaming);
  serviceBatteryMonitor();
  serviceBLE();
  cs1237Service();
  serviceStatusLed(isUsbPresent(), connected, batteryCriticallyLow());

  if (batteryCriticallyLow()) {
    // Never deferred during streaming: the monitor is non-blocking, so
    // there is no reason to fly blind mid-measurement like ErgoJump did.
    stopStreaming();
    powerOffNow();
  }

  // --- Sample pipeline: drain ring -> calibrate -> decimate -> packetize ---
  Cs1237Sample s;
  while (cs1237PopSample(s)) {
    forceFeedSample(s.raw);
    if (!streaming) continue;
    if (++decimCounter < decimation) continue;
    decimCounter = 0;
    packetAppend(rawMode ? (float)s.raw : forceToNewtons(s.raw), s.tUs);
  }

  // Report finished tare/calibration captures.
  const char *capResult = forceTakeCaptureResult();
  if (capResult[0] != '\0') reply(capResult);

  // --- Commands (loop task only) ---
  char cmd[48];
  while (getBleCommand(cmd, sizeof(cmd))) processCommand(cmd);
  handleSerialInput();

  // Periodic status while connected and not streaming.
  static uint32_t lastStatusMs = 0;
  if (connected && !streaming && millis() - lastStatusMs >= 2000) {
    lastStatusMs = millis();
    sendStatus();
  }

  // Disconnect during a stream: stop cleanly so state is consistent on reconnect.
  if (streaming && !connected) stopStreaming();
}
