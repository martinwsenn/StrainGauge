#ifndef CONFIG_H
#define CONFIG_H

// ============================================================
// NexoHHD hardware configuration
// Pin map from "Sch celda de carga" (KiCad, ESP32-WROOM-32)
// ============================================================

// --- Power ---
// EN of both AP2112K-3.3 regulators. Driven HIGH immediately at boot
// (constructor in PowerManager.cpp) and re-asserted every loop pass;
// driving it LOW powers the device off.
#define POWER_LATCH_PIN 25
// Momentary power button. GPIO36 is input-only with no internal pulls;
// the board provides the biasing. Assumed ACTIVE HIGH (same as ErgoJump's
// button circuit) — VERIFY on first bring-up.
#define POWER_BUTTON_PIN 36
#define BUTTON_DEBOUNCE_MS 40
#define POWER_OFF_HOLD_MS 800
// USB VBUS divider on GPIO34 (input-only, digital): HIGH = USB present.
// Charger is a TP4056; its CHRG/STDBY pins are not wired to the MCU, so
// "charging" == "USB present" as far as firmware can tell.
#define USB_DETECT_PIN 34

// --- Battery sense ---
#define BATTERY_PIN 39 // ADC1 (input-only)
// R17 220k + R16 47k divider => (220+47)/47 nominal. Calibrate per unit.
#define BATTERY_DIVIDER_RATIO 5.68f
#define BATTERY_SHUTDOWN_VOLTS 3.25f
#define BATTERY_SAFETY_ARM_MS 3000
#define BATTERY_SAFETY_CONSECUTIVE 5

// --- LEDs (WS2812B) ---
#define STATUS_LED_PIN 26
#define STATUS_LED_BRIGHTNESS 51 // ~20%
#define LED_STRIP_PIN 23  // present on the board; not used by firmware yet
#define LED_STRIP_COUNT 8 // TODO: confirm actual LED count before using

// --- CS1237 ADC ---
#define CS1237_SCLK_PIN 13
#define CS1237_DATA_PIN 14 // DOUT/DRDY; bidirectional during register ops
// 0x7C = external ref (REFO off), 1280 Hz, PGA 128, channel A.
#define CS1237_CONFIG_VALUE 0x7C
// The config register is volatile and reverts to 0x0C (10 Hz) on a supply
// dip shallow enough that the ESP32 keeps running. If no sample arrives
// for this long while sampling is supposed to be active, the driver
// rewrites the config (revert watchdog). 50 ms sits far above the 1280 Hz
// interval (~0.78 ms) and far below the reverted 10 Hz interval (100 ms).
#define CS1237_WATCHDOG_US 50000ULL
// DRDY is low-level triggered. If DOUT has been low this long with no ISR
// sample (~1.5 intervals at 1280 Hz), the loop reads the result itself and
// counts an edge-miss: the interrupt did not run. Fault signature, reported.
#define CS1237_MISSED_DRDY_US 1200ULL
// DOUT still low right after the DRDY-reset pulses this many times in a
// row = line stuck low; the ISR cuts its own interrupt so a level trigger
// cannot starve the loop, and the service loop recovers.
#define CS1237_STORM_LIMIT 4
// SCLK half-period (high = low), us. Datasheet minimum is 0.455 us. At 1 us
// the first BLE test showed rare single-bit read errors (sign bit, bits
// 10-13) only while the radio was active; the bench sketch's ~3-4 us never
// did. 2 us = 108 us per 27-pulse read, 14% of the 765 us interval.
#define CS1237_SCLK_HALF_US 2
#define CS1237_SETTLE_MS 2            // analog settling after config write
#define CS1237_DISCARD_AFTER_CONFIG 4 // conversions discarded after write

// --- Sampling / streaming ---
#define SAMPLE_RING_SIZE 512  // ~400 ms of headroom at 1280 Hz
#define SAMPLES_PER_PACKET 32 // 40 packets/s at 1280 Hz (bench-validated)
#define DEFAULT_DECIMATION 1  // output-rate decimation, set per SKU
#define MAX_DECIMATION 32

// --- Force calibration defaults (overridden by per-unit NVS values) ---
// ZELL TSA 300 kg, 3.0093 mV/V, PGA 128, REFIN tied to excitation:
// full scale = Vref/256, so 2942 N (300 kg) lands at 77.0% of FS
// => 2942.1 N / 6.462e6 counts. Nominal only; every unit must run the
// calibration procedure (see README) which stores its values in NVS.
#define DEFAULT_NEWTONS_PER_COUNT 4.553e-4f
#define CAL_CAPTURE_SAMPLES 256 // averaged per tare/calibration capture
#define CAL_MAX_REF_NEWTONS 5000.0f

// --- Auto-off ---
#define INACTIVITY_AUTO_OFF_MS (10UL * 60UL * 1000UL)

// --- BLE ---
#define BLE_DEVICE_NAME "NexoHHD"
#define FW_VERSION "0.1.0"
// Data packet layout version. v1 is PROVISIONAL until the app team
// confirms what they expect; bump this on any layout change.
#define BLE_PROTOCOL_VERSION 1
#define SERVICE_UUID      "8e7c1237-a5b4-46f1-93f5-1b0c5a6e0001"
#define DATA_CHAR_UUID    "8e7c1237-a5b4-46f1-93f5-1b0c5a6e0002"
#define CONTROL_CHAR_UUID "8e7c1237-a5b4-46f1-93f5-1b0c5a6e0003"
#define STATUS_CHAR_UUID  "8e7c1237-a5b4-46f1-93f5-1b0c5a6e0004"

#endif // CONFIG_H
