#include "BatteryMonitor.h"
#include "Config.h"

// One ADC sample every SAMPLE_INTERVAL_MS, averaged in groups of
// SAMPLES_PER_AVERAGE. Continuous sampling keeps the ADC sampling cap
// charged, so no throw-away read is needed.
static const uint32_t SAMPLE_INTERVAL_MS = 4;
static const int SAMPLES_PER_AVERAGE = 32;

static uint32_t lastSampleMs = 0;
static uint32_t accMv = 0;
static int accCount = 0;
static float voltage = 0.0f;

static int consecutiveLow = 0;
static uint32_t armAtMs = 0;

void setupBatteryMonitor() {
  analogReadResolution(12);
  // 0 dB attenuation: full-scale ~950 mV; 4.2 V / 5.68 divider = 739 mV.
  analogSetPinAttenuation(BATTERY_PIN, ADC_0db);
  armAtMs = millis() + BATTERY_SAFETY_ARM_MS;
}

void serviceBatteryMonitor() {
  const uint32_t now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  accMv += analogReadMilliVolts(BATTERY_PIN);
  if (++accCount < SAMPLES_PER_AVERAGE) return;

  voltage = ((float)accMv / SAMPLES_PER_AVERAGE / 1000.0f) * BATTERY_DIVIDER_RATIO;
  accMv = 0;
  accCount = 0;

  if ((int32_t)(now - armAtMs) < 0) return; // post-boot stabilization
  if (voltage < BATTERY_SHUTDOWN_VOLTS) {
    if (consecutiveLow < BATTERY_SAFETY_CONSECUTIVE) consecutiveLow++;
  } else {
    consecutiveLow = 0;
  }
}

float batteryVoltage() { return voltage; }

bool batteryCriticallyLow() { return consecutiveLow >= BATTERY_SAFETY_CONSECUTIVE; }
