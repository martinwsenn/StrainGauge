#ifndef CS1237_H
#define CS1237_H

#include <Arduino.h>

// One conversion result. tUs is esp_timer_get_time() captured in the DRDY
// ISR — 64-bit, wrap-free, and immune to the chip's +0.57% oscillator
// offset because we never assume the nominal sample interval.
struct Cs1237Sample {
  int32_t raw;  // sign-extended 24-bit conversion
  uint64_t tUs; // microseconds since boot at DRDY
};

// Configures pins, writes the config register (verified by readback),
// discards settling conversions and attaches the DRDY interrupt.
// Returns false if the chip never acknowledged the config (wiring fault);
// sampling is left disabled in that case.
bool cs1237Begin();

// Call every loop pass. Runs the config-revert watchdog: if the register
// dropped back to 10 Hz after a supply dip (or the chip powered down),
// this rewrites the config and resumes. Cheap when healthy.
void cs1237Service();

// Pops the oldest sample from the ring buffer. Returns false when empty.
bool cs1237PopSample(Cs1237Sample &s);

// --- Diagnostics: part of the data contract, not debug-only ---
uint32_t cs1237DroppedCount();  // ring overflows (oldest dropped)
uint32_t cs1237ReconfigCount(); // watchdog-triggered config rewrites
uint32_t cs1237MaxGapUs();      // worst observed inter-sample gap
// DRDY path health. Healthy: calls ~= samples, the other three stay at 0.
uint32_t cs1237IsrCalls();      // DRDY interrupt entries
uint32_t cs1237IsrSpurious();   // entries with DOUT already high (ignored)
uint32_t cs1237EdgeMissCount(); // samples the loop had to read because the interrupt did not
uint32_t cs1237StormCount();    // DOUT-stuck-low interrupt cuts + recoveries
bool cs1237IsRunning();

// SCLK half-period in microseconds (1..10; default CS1237_SCLK_HALF_US).
// Exposed for the diagnostic sketch's timing-margin A/B under radio load.
void cs1237SetSclkHalfUs(uint32_t us);
uint32_t cs1237SclkHalfUs();

// On-demand config register readback for bring-up/diagnostics. Detaches
// DRDY for one bounded 46-pulse op and re-attaches. 0xFF = timeout or a
// chip that is not driving DOUT. Call only after cs1237Begin().
uint8_t cs1237ReadConfig();

#endif // CS1237_H
