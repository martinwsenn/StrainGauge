#ifndef FORCE_SENSOR_H
#define FORCE_SENSOR_H

#include <Arduino.h>

// Raw counts -> Newtons, with per-unit calibration persisted in NVS.
//   newtons = ((raw - calOffset) - tareCounts) * calScale
// calOffset/calScale come from the factory calibration procedure (README);
// tareCounts is a runtime rebase and is never persisted.

void forceSensorInit(); // load calibration from NVS (defaults if absent)

float forceToNewtons(int32_t raw);

// Feed every sample through this from the loop pipeline; it powers the
// capture averaging below and keeps the "live force" value fresh.
void forceFeedSample(int32_t raw);
float forceLastNewtons();

// Capture requests (each averages CAL_CAPTURE_SAMPLES samples ~0.2 s):
//   TARE      -> runtime zero at current load
//   CAL_ZERO  -> store zero-load offset to NVS (clears tare)
//   CAL_SCALE -> store scale from a known reference load to NVS
// startCapture returns false if a capture is already running or (for
// CAL_SCALE) the reference is out of range — failures are loud, never
// silent, per the ErgoJump sensitivity-range lesson.
enum ForceCaptureType { FORCE_CAPTURE_TARE, FORCE_CAPTURE_CAL_ZERO, FORCE_CAPTURE_CAL_SCALE };
bool forceStartCapture(ForceCaptureType type, float refNewtons);
bool forceCaptureBusy();
// Non-empty once a capture finishes (result or error text); cleared on read.
const char *forceTakeCaptureResult();

int32_t forceCalOffset();
float forceCalScale();

#endif // FORCE_SENSOR_H
