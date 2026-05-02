#include "button.h"

Button::Button(uint8_t pin): buttonPin(pin) {
  pinMode(buttonPin, INPUT_PULLUP);
}

void Button::onAction(void (*cb)(Button*, ButtonAction)) {
  callback = cb;
}

void Button::update() {
  uint8_t reading = digitalRead(buttonPin);

  if (reading != lastReading) {
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    if (reading != stableState) {
      stableState = reading;

      if (stableState == LOW) {
        pressedTime = millis();
        heldFired = false;
        heldLongFired = false;
        if (callback) callback(this, ButtonAction::Down);
      } else {
        if (callback) callback(this, ButtonAction::Up);
      }
    }
  }

  if (stableState == LOW && !heldFired && (millis() - pressedTime) >= holdTime) {
    heldFired = true;
    if (callback) callback(this, ButtonAction::Held);
  }

  if (stableState == LOW && !heldLongFired && (millis() - pressedTime) >= holdLongTime) {
    heldLongFired = true;
    if (callback) callback(this, ButtonAction::HeldLong);
  }
  
  lastReading = reading;
}