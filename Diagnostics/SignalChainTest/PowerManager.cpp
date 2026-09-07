#include "PowerManager.h"
#include "Config.h"
#include "LedController.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "driver/gpio.h"

// .bss is zeroed before constructors run, so this is safe to fill there.
static uint32_t savedBrownoutReg = 0;

// Runs before setup(): the momentary button is still the only thing
// powering the board, so the latch (AP2112 EN) must go HIGH before the
// user lets go. Brown-out is disabled only for the boot window (latch
// inrush dips the rail) and restored at the end of setup().
void __attribute__((constructor)) immediateLatch() {
  savedBrownoutReg = READ_PERI_REG(RTC_CNTL_BROWN_OUT_REG);
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);

  gpio_config_t io_conf = {};
  io_conf.pin_bit_mask = (1ULL << POWER_LATCH_PIN);
  io_conf.mode = GPIO_MODE_OUTPUT;
  io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
  io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
  io_conf.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&io_conf);
  gpio_set_level((gpio_num_t)POWER_LATCH_PIN, 1);
}

void restoreBrownoutDetector() {
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, savedBrownoutReg);
}

static uint32_t lastActivityMs = 0;

void powerManagerInit() {
  pinMode(POWER_LATCH_PIN, OUTPUT);
  digitalWrite(POWER_LATCH_PIN, HIGH);
  pinMode(POWER_BUTTON_PIN, INPUT); // input-only pin, board provides bias
  pinMode(USB_DETECT_PIN, INPUT);
  lastActivityMs = millis();
}

void notifyActivity() { lastActivityMs = millis(); }

bool isUsbPresent() { return digitalRead(USB_DETECT_PIN) == HIGH; }

void powerOffNow() {
  digitalWrite(POWER_LATCH_PIN, LOW);
  while (true) delay(1000); // rail collapses; nothing after this runs
}

void servicePowerManager(bool busy) {
  // Assert critical hardware state every pass, not just at boot.
  digitalWrite(POWER_LATCH_PIN, HIGH);

  const uint32_t now = millis();

  // --- Power button: debounce + long-press power-off ---
  static bool lastStableHigh = false;
  static bool lastReadingHigh = false;
  static uint32_t lastDebounceMs = 0;
  static uint32_t pressStartMs = 0;
  static bool holdIndicated = false;

  const bool readingHigh = (digitalRead(POWER_BUTTON_PIN) == HIGH);
  if (readingHigh != lastReadingHigh) {
    lastDebounceMs = now;
    lastReadingHigh = readingHigh;
  }
  if ((now - lastDebounceMs) >= BUTTON_DEBOUNCE_MS && readingHigh != lastStableHigh) {
    lastStableHigh = readingHigh;
    if (readingHigh) {
      pressStartMs = now;
      holdIndicated = false;
      notifyActivity();
    } else {
      const uint32_t held = (pressStartMs > 0) ? (now - pressStartMs) : 0;
      pressStartMs = 0;
      if (holdIndicated) clearStatusLedOverride();
      if (held >= POWER_OFF_HOLD_MS) powerOffNow();
    }
  }
  // Solid red once the hold threshold is reached: releasing now powers off.
  if (lastStableHigh && pressStartMs > 0 && !holdIndicated &&
      (now - pressStartMs) >= POWER_OFF_HOLD_MS) {
    holdIndicated = true;
    setStatusLedOverride(255, 0, 0);
  }

  // --- Inactivity auto-off ---
  // Cheap timer check first; the pin read only happens after expiry.
  if (busy) {
    lastActivityMs = now;
    return;
  }
  if (now - lastActivityMs < INACTIVITY_AUTO_OFF_MS) return;
  if (isUsbPresent()) {
    // Charging counts as activity so we don't re-check every pass on the charger.
    lastActivityMs = now;
    return;
  }
  powerOffNow();
}
