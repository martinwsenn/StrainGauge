#ifndef POWER_MANAGER_H
#define POWER_MANAGER_H

#include <Arduino.h>

void powerManagerInit();

// Call every loop pass: re-asserts the latch, debounces the power button
// (long press = power off, with LED feedback at the threshold), and runs
// the inactivity auto-off. `busy` = BLE connected or streaming.
void servicePowerManager(bool busy);

void notifyActivity(); // any user/app interaction resets the auto-off timer

bool isUsbPresent(); // digital read of the VBUS divider (GPIO34)

void powerOffNow(); // releases the latch; does not return

// Re-enables the brown-out detector that the boot constructor disabled to
// survive latch inrush. Call at the end of setup(). ErgoJump left it
// disabled forever, trading reset-protection for silent corruption when
// the cell sags — here the window is boot-only.
void restoreBrownoutDetector();

#endif // POWER_MANAGER_H
