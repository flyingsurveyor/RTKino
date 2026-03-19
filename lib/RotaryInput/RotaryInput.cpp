// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "RotaryInput.h"

// ---------------------------------------------------------------------------
// Timing constants
// ---------------------------------------------------------------------------
static const uint32_t ROT_STEP_INTERVAL_MS  = 80;   // minimum gap between accepted steps
static const uint32_t BTN_DEBOUNCE_MS   = 50;   // button debounce window
static const uint32_t CLICK_MAX_MS      = 500;  // max press duration for a click
static const uint32_t LONGPRESS_MIN_MS  = 800;  // min press duration for long press
static const uint32_t DOUBLE_CLICK_MS   = 400;  // max interval between two presses for double click

// ---------------------------------------------------------------------------
// Pin storage
// ---------------------------------------------------------------------------
static uint8_t s_pinCLK = 0;
static uint8_t s_pinDT  = 0;
static uint8_t s_pinSW  = 0;

// ---------------------------------------------------------------------------
// Gray code transition table
// Index = (previousState << 2) | currentState  (states are 2-bit: CLK<<1 | DT)
// Values: 0 = invalid/noise, +1 = CW, -1 = CCW
// ---------------------------------------------------------------------------
static const int8_t ENCODER_TABLE[16] = {
     0, -1,  1,  0,
     1,  0,  0, -1,
    -1,  0,  0,  1,
     0,  1, -1,  0
};

// Counts threshold to emit one step.
// Most detented encoders produce 4 ISR transitions per detent (full quadrature
// cycle).  A threshold of 4 ensures exactly one step per physical click.
static const int32_t ENCODER_STEP_THRESHOLD = 4;

// ---------------------------------------------------------------------------
// ISR-side volatile state
// ---------------------------------------------------------------------------
static volatile int32_t s_isrCount  = 0;   // accumulated quadrature count
static volatile uint8_t s_prevState = 0;   // previous 2-bit encoder state

// ---------------------------------------------------------------------------
// Button volatile state (read in both ISR context and update())
// ---------------------------------------------------------------------------
static volatile bool     s_btnRaw        = false;  // raw (debounced) button level
static volatile uint32_t s_btnChangeTime = 0;      // millis() of last stable change

// ---------------------------------------------------------------------------
// Processed state (written only by update(), read by getters)
// ---------------------------------------------------------------------------
static volatile int      s_direction     = 0;
static volatile bool     s_clicked       = false;
static volatile bool     s_longPressed   = false;
static volatile bool     s_doubleClicked = false;

// ---------------------------------------------------------------------------
// Local helpers
// ---------------------------------------------------------------------------
static uint32_t          s_lastRotTime   = 0;      // last accepted rotation time
static int32_t           s_accumCount    = 0;      // accumulated ISR counts
static bool              s_btnPrev       = false;  // previous stable button state
static uint32_t          s_btnPressTime  = 0;      // millis() when button went LOW
static bool              s_longFired     = false;  // long-press event already sent
static uint32_t          s_lastPressTime = 0;      // millis() of previous button press (for double click)
static bool              s_suppressClick = false;  // suppress next click release (after double click)

// ---------------------------------------------------------------------------
// ISR — read both pins, look up Gray code table, accumulate direction
// ---------------------------------------------------------------------------
static void IRAM_ATTR encoderISR() {
  uint8_t clk = digitalRead(s_pinCLK);
  uint8_t dt  = digitalRead(s_pinDT);
  uint8_t newState = (clk << 1) | dt;
  uint8_t idx = (s_prevState << 2) | newState;
  s_isrCount += ENCODER_TABLE[idx];
  s_prevState = newState;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
namespace RotaryInput {

void init(uint8_t pinCLK, uint8_t pinDT, uint8_t pinSW) {
  s_pinCLK = pinCLK;
  s_pinDT  = pinDT;
  s_pinSW  = pinSW;

  pinMode(pinCLK, INPUT_PULLUP);
  pinMode(pinDT,  INPUT_PULLUP);
  pinMode(pinSW,  INPUT_PULLUP);

  // Initial button state (HIGH = released)
  s_btnRaw    = (digitalRead(pinSW) == HIGH);
  s_btnPrev   = s_btnRaw;

  // Read initial encoder state so the first ISR transition is valid
  s_prevState = (digitalRead(pinCLK) << 1) | digitalRead(pinDT);

  // Attach ISR on BOTH pins to capture all four quadrature transitions
  attachInterrupt(digitalPinToInterrupt(pinCLK), encoderISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pinDT),  encoderISR, CHANGE);
}

void update() {
  uint32_t now = millis();

  // --- Rotation processing (Gray code state machine) ---
  // Atomically snapshot and reset the ISR accumulator
  noInterrupts();
  int32_t delta = s_isrCount;
  s_isrCount = 0;
  interrupts();

  s_accumCount += delta;

  if (s_accumCount >= ENCODER_STEP_THRESHOLD || s_accumCount <= -ENCODER_STEP_THRESHOLD) {
    if ((now - s_lastRotTime) >= ROT_STEP_INTERVAL_MS) {
      s_lastRotTime = now;
      s_direction   = (s_accumCount > 0) ? 1 : -1;
    }
    s_accumCount = 0;
  }

  // --- Button debounce & classification ---
  bool btnNow = (digitalRead(s_pinSW) == HIGH);  // HIGH = released
  if (btnNow != s_btnPrev) {
    if ((now - s_btnChangeTime) >= BTN_DEBOUNCE_MS) {
      s_btnChangeTime = now;
      s_btnPrev = btnNow;

      if (!btnNow) {
        // Button just pressed (LOW)
        s_btnPressTime = now;
        s_longFired    = false;

        // Double click detection: second press within DOUBLE_CLICK_MS of the previous press
        if ((now - s_lastPressTime) < DOUBLE_CLICK_MS) {
          s_doubleClicked = true;
          s_clicked       = false;  // cancel the single click from the first press
          s_suppressClick = true;   // suppress click on this second release
        }
        s_lastPressTime = now;
      } else {
        // Button just released (HIGH)
        uint32_t held = now - s_btnPressTime;
        if (!s_longFired && held < CLICK_MAX_MS) {
          if (!s_suppressClick) {
            s_clicked = true;
          }
          s_suppressClick = false;
        }
        s_longFired = false;
      }
    }
  }

  // --- Detect long press while still held ---
  if (!s_btnPrev && !s_longFired) {
    if ((now - s_btnPressTime) >= LONGPRESS_MIN_MS) {
      s_longPressed = true;
      s_longFired   = true;
    }
  }
}

int getDirection() {
  int d = s_direction;
  s_direction = 0;
  return d;
}

bool isClicked() {
  bool c = s_clicked;
  s_clicked = false;
  return c;
}

bool isLongPress() {
  bool lp = s_longPressed;
  s_longPressed = false;
  return lp;
}

bool isDoubleClick() {
  bool dc = s_doubleClicked;
  s_doubleClicked = false;
  return dc;
}

} // namespace RotaryInput
