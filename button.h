#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

enum class ButtonAction : uint8_t {
  Down,
  Up,
  Held,
  HeldLong
};

class Button {
private:
  uint8_t buttonPin;

  uint8_t stableState = HIGH;
  uint8_t lastReading = HIGH;

  unsigned long lastDebounceTime = 0;
  const unsigned long debounceDelay = 30;

  unsigned long pressedTime = 0;
  const unsigned long holdTime = 500;
  const unsigned long holdLongTime = 5000;

  bool heldFired = false;
  bool heldLongFired = false;

  void (*callback)(Button* btn, ButtonAction action) = nullptr;
public:
  Button(uint8_t pin);
  ~Button() = default;

  void update();
  void onAction(void (*cb)(Button*, ButtonAction));

  uint8_t pin() const { return buttonPin; }
};

#endif