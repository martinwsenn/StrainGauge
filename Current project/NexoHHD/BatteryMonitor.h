#ifndef BATTERY_MONITOR_H
#define BATTERY_MONITOR_H

#include <Arduino.h>

void setupBatteryMonitor();

// Non-blocking: takes at most ONE ADC sample per call, spread over time
// (~130 ms per completed 32-sample average). ErgoJump's equivalent held
// the loop for ~110 ms of delay() per read and had to be disabled during
// tests, leaving the safety cutoff blind; this one runs at full sampling
// rate with no measurable stall, so it is NEVER deferred.
void serviceBatteryMonitor();

float batteryVoltage(); // latest completed average; 0 until the first one

// True when the safety cutoff should fire: armed (3 s post-boot) and
// BATTERY_SAFETY_CONSECUTIVE consecutive averages under the threshold.
// Caller (PowerManager) performs the actual power-off.
bool batteryCriticallyLow();

#endif // BATTERY_MONITOR_H
