#include "LedController.h"
#include "Config.h"
#include <Adafruit_NeoPixel.h>

static Adafruit_NeoPixel statusLed(1, STATUS_LED_PIN, NEO_GRB + NEO_KHZ800);

static bool overrideActive = false;
static uint8_t ovrR = 0, ovrG = 0, ovrB = 0;

void setupStatusLed() {
  statusLed.begin();
  statusLed.setBrightness(STATUS_LED_BRIGHTNESS);
  statusLed.clear();
  statusLed.show();
}

// Triangle wave 0..255 for the breathing effect, period ~2 s.
static uint8_t breathe(uint32_t nowMs) {
  const uint32_t phase = nowMs % 2000;
  const uint32_t half = (phase < 1000) ? phase : (2000 - phase);
  return (uint8_t)((half * 255) / 1000);
}

void serviceStatusLed(bool usbPresent, bool connected, bool lowBattery) {
  static uint32_t lastRender = 0;
  const uint32_t now = millis();
  if (now - lastRender < 25) return;
  lastRender = now;

  uint8_t r = 0, g = 0, b = 0;
  if (overrideActive) {
    r = ovrR; g = ovrG; b = ovrB;
  } else if (lowBattery) {
    const bool on = (now % 500) < 250;
    r = on ? 255 : 0;
  } else if (usbPresent) {
    const uint8_t v = breathe(now);
    r = v; g = (uint8_t)(v / 3); // amber
  } else if (connected) {
    g = 128; b = 255; // cyan
  } else {
    b = 255; // idle on battery
  }
  statusLed.setPixelColor(0, statusLed.Color(r, g, b));
  statusLed.show();
}

void setStatusLedOverride(uint8_t r, uint8_t g, uint8_t b) {
  overrideActive = true;
  ovrR = r; ovrG = g; ovrB = b;
}

void clearStatusLedOverride() { overrideActive = false; }
