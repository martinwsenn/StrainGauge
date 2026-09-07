#include "BleService.h"
#include "Config.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

static BLEServer *gServer = nullptr;
static BLECharacteristic *gDataChar = nullptr;
static BLECharacteristic *gControlChar = nullptr;
static BLECharacteristic *gStatusChar = nullptr;

static volatile bool gConnected = false;
// Deferred advertising restart: set in the disconnect callback, executed
// from serviceBLE() on the loop task. No delay() inside BLE callbacks.
static volatile bool gAdvRestartPending = false;
static uint32_t gAdvRestartAtMs = 0;

#define CMD_MSG_LEN 48
static QueueHandle_t gCmdQueue = nullptr;

class ServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer *) override { gConnected = true; }
  void onDisconnect(BLEServer *) override {
    gConnected = false;
    gAdvRestartPending = true; // serviceBLE() restarts after a short grace
  }
};

class ControlCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *chr) override {
    // BLE stack task context: copy and enqueue, nothing else.
    // auto: getValue() returns std::string on esp32 core 2.x, String on 3.x.
    auto v = chr->getValue();
    char msg[CMD_MSG_LEN];
    strncpy(msg, v.c_str(), sizeof(msg) - 1);
    msg[sizeof(msg) - 1] = '\0';
    if (gCmdQueue) xQueueSend(gCmdQueue, msg, 0); // full queue: command dropped, app can retry
  }
};

void initializeBLE() {
  gCmdQueue = xQueueCreate(4, CMD_MSG_LEN);

  BLEDevice::init(BLE_DEVICE_NAME);
  // Data packets are 12 + 4*SAMPLES_PER_PACKET bytes (140 with 32 samples);
  // the app must negotiate MTU >= payload+3. We allow up to the max.
  BLEDevice::setMTU(517);

  gServer = BLEDevice::createServer();
  gServer->setCallbacks(new ServerCallbacks());

  BLEService *svc = gServer->createService(SERVICE_UUID);

  gDataChar = svc->createCharacteristic(DATA_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY);
  gDataChar->addDescriptor(new BLE2902());

  gControlChar = svc->createCharacteristic(CONTROL_CHAR_UUID, BLECharacteristic::PROPERTY_WRITE);
  gControlChar->setCallbacks(new ControlCallbacks());

  gStatusChar = svc->createCharacteristic(
      STATUS_CHAR_UUID, BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_READ);
  gStatusChar->addDescriptor(new BLE2902());
  gStatusChar->setValue("boot");

  svc->start();
  BLEAdvertising *adv = BLEDevice::getAdvertising();
  adv->addServiceUUID(SERVICE_UUID);
  adv->setScanResponse(true);
  adv->setMinPreferred(0x06);
  adv->setMaxPreferred(0x12);
  adv->start();
}

void serviceBLE() {
  if (gAdvRestartPending) {
    if (gAdvRestartAtMs == 0) {
      gAdvRestartAtMs = millis() + 200; // grace for the stack to tear down
    } else if ((int32_t)(millis() - gAdvRestartAtMs) >= 0) {
      gAdvRestartPending = false;
      gAdvRestartAtMs = 0;
      gServer->getAdvertising()->start();
    }
  }
}

bool isBleConnected() { return gConnected; }

bool getBleCommand(char *buf, size_t len) {
  if (!gCmdQueue) return false;
  char msg[CMD_MSG_LEN];
  if (xQueueReceive(gCmdQueue, msg, 0) != pdTRUE) return false;
  strncpy(buf, msg, len - 1);
  buf[len - 1] = '\0';
  return true;
}

void bleNotifyData(const uint8_t *data, size_t len) {
  if (!gConnected || !gDataChar) return;
  gDataChar->setValue(const_cast<uint8_t *>(data), len);
  gDataChar->notify();
}

void bleNotifyStatus(const char *text) {
  if (!gStatusChar) return;
  gStatusChar->setValue((uint8_t *)text, strlen(text));
  if (gConnected) gStatusChar->notify();
}
