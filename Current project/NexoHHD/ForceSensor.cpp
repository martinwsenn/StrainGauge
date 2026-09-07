#include "ForceSensor.h"
#include "Config.h"
#include <Preferences.h>

static Preferences prefs;

static int32_t calOffset = 0;
static float calScale = DEFAULT_NEWTONS_PER_COUNT;
static int32_t tareCounts = 0;

static float lastNewtons = 0.0f;

// Capture state (single capture at a time, driven from the loop pipeline)
static bool captureActive = false;
static ForceCaptureType captureType = FORCE_CAPTURE_TARE;
static float captureRefNewtons = 0.0f;
static int64_t captureAcc = 0;
static uint16_t captureCount = 0;
static char captureResult[96] = "";

void forceSensorInit() {
  prefs.begin("nexo", false);
  calOffset = prefs.getInt("caloff", 0);
  calScale = prefs.getFloat("calscale", DEFAULT_NEWTONS_PER_COUNT);
  Serial.printf("ForceSensor: calOffset=%ld calScale=%.6e%s\n",
                (long)calOffset, calScale,
                prefs.isKey("calscale") ? "" : " (defaults - unit NOT calibrated)");
}

float forceToNewtons(int32_t raw) {
  return (float)((raw - calOffset) - tareCounts) * calScale;
}

float forceLastNewtons() { return lastNewtons; }

static void finishCapture(int32_t avg) {
  switch (captureType) {
    case FORCE_CAPTURE_TARE:
      tareCounts = avg - calOffset;
      snprintf(captureResult, sizeof(captureResult), "OK:TARE counts=%ld", (long)tareCounts);
      break;
    case FORCE_CAPTURE_CAL_ZERO:
      calOffset = avg;
      tareCounts = 0;
      prefs.putInt("caloff", calOffset);
      snprintf(captureResult, sizeof(captureResult), "OK:CAL0 offset=%ld", (long)calOffset);
      break;
    case FORCE_CAPTURE_CAL_SCALE: {
      const int32_t delta = avg - calOffset;
      if (delta == 0) {
        snprintf(captureResult, sizeof(captureResult), "ERR:CALW no signal above zero offset");
        return;
      }
      calScale = captureRefNewtons / (float)delta;
      prefs.putFloat("calscale", calScale);
      snprintf(captureResult, sizeof(captureResult), "OK:CALW scale=%.6e (delta=%ld)", calScale, (long)delta);
      break;
    }
  }
}

void forceFeedSample(int32_t raw) {
  lastNewtons = forceToNewtons(raw);
  if (!captureActive) return;
  captureAcc += raw;
  if (++captureCount >= CAL_CAPTURE_SAMPLES) {
    captureActive = false;
    finishCapture((int32_t)(captureAcc / CAL_CAPTURE_SAMPLES));
  }
}

bool forceStartCapture(ForceCaptureType type, float refNewtons) {
  if (captureActive) {
    snprintf(captureResult, sizeof(captureResult), "ERR:capture already running");
    return false;
  }
  if (type == FORCE_CAPTURE_CAL_SCALE &&
      (refNewtons <= 0.0f || refNewtons > CAL_MAX_REF_NEWTONS)) {
    snprintf(captureResult, sizeof(captureResult),
             "ERR:CALW ref out of range (0..%.0f N)", (double)CAL_MAX_REF_NEWTONS);
    return false;
  }
  captureType = type;
  captureRefNewtons = refNewtons;
  captureAcc = 0;
  captureCount = 0;
  captureActive = true;
  return true;
}

bool forceCaptureBusy() { return captureActive; }

const char *forceTakeCaptureResult() {
  static char out[sizeof(captureResult)];
  if (captureResult[0] == '\0') return "";
  strncpy(out, captureResult, sizeof(out));
  out[sizeof(out) - 1] = '\0';
  captureResult[0] = '\0';
  return out;
}

int32_t forceCalOffset() { return calOffset; }
float forceCalScale() { return calScale; }
