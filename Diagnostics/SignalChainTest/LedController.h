#ifndef LED_CONTROLLER_H
#define LED_CONTROLLER_H

#include <Arduino.h>

void setupStatusLed();

// Non-blocking; renders at ~40 Hz. State priority (highest first):
// override > low battery (red blink) > charging/USB (amber breathing)
// > BLE connected (cyan) > idle on battery (blue).
void serviceStatusLed(bool usbPresent, bool connected, bool lowBattery);

// Temporary full-priority color (e.g. solid red when the power-off hold
// threshold is reached, so the user knows releasing will shut down).
void setStatusLedOverride(uint8_t r, uint8_t g, uint8_t b);
void clearStatusLedOverride();

#endif // LED_CONTROLLER_H
