#ifndef BLE_SERVICE_H
#define BLE_SERVICE_H

#include <Arduino.h>

void initializeBLE();

// Call every loop pass. Handles the deferred advertising restart after a
// disconnect — BLE callbacks themselves never block or mutate app state.
void serviceBLE();

bool isBleConnected();

// Dequeues one pending command line written to the control characteristic.
// Commands are executed on the loop task only; the BLE onWrite callback
// just enqueues (ErgoJump lesson: test logic on the BLE stack task raced
// the loop task with zero synchronization).
bool getBleCommand(char *buf, size_t len);

void bleNotifyData(const uint8_t *data, size_t len);
void bleNotifyStatus(const char *text);

#endif // BLE_SERVICE_H
