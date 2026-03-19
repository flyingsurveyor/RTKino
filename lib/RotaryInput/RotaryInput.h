// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef ROTARYINPUT_H
#define ROTARYINPUT_H

#include <Arduino.h>

// Lightweight rotary encoder driver with ISR-based CLK detection
// and software debounce. All volatile state is ISR-safe.
namespace RotaryInput {
  // Call once in setup() with the three encoder pins.
  // Attaches an ISR on CLK (FALLING edge).
  void init(uint8_t pinCLK, uint8_t pinDT, uint8_t pinSW);

  // Call every loop() iteration to process ISR flags and apply debounce.
  void update();

  // Returns +1 (CW), -1 (CCW), or 0 (no movement).
  // Value is consumed (reset to 0) after the first read.
  int  getDirection();

  // Returns true once when a single click (press + release < 500 ms) is detected.
  // Consumed after read.
  bool isClicked();

  // Returns true once when a long press (> 800 ms held) is detected.
  // Consumed after read.
  bool isLongPress();

  // Returns true once when a double click (two presses within 400 ms) is detected.
  // Consuming this also cancels the pending single click for the first press.
  // Consumed after read.
  bool isDoubleClick();
}

#endif // ROTARYINPUT_H
