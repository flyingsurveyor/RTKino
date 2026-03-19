// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef OLEDMENU_H
#define OLEDMENU_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// OledMenu — navigable Settings menu for RTKino OLED display
// Driven by RotaryInput events; coexists with normal oledUpdate() display.
// ---------------------------------------------------------------------------

enum OledMenuState {
  OLED_NORMAL,              // Standard RTKino display (oledUpdate() handles it)
  OLED_PAGE2,               // Second home page (oledUpdate() handles it)
  OLED_PAGE3,               // Third home page: system info (oledUpdate() handles it)
  OLED_MENU_MAIN,           // Main menu: [Surveys] [Settings] [Raw Log] [Back]
  OLED_MENU_SETTINGS,       // Settings list (toggles, sub-menus, actions)
  OLED_SUBMENU_RATE,        // ZED measurement rate selector
  OLED_SUBMENU_BRIGHT,      // Brightness selector
  OLED_INFO_SCREEN,         // Read-only info screen
  OLED_CONFIRM_REBOOT,      // "Reboot? ●=Yes ◄=No" confirmation
  // ---- Survey states ----
  OLED_SURVEY_MENU,         // Survey sub-menu: [Measure point / Surveys / New survey / Back]
  OLED_SURVEY_LIST,         // List of surveys (scrollable, click = set active)
  OLED_SURVEY_CODE_CAT,     // Step 1: pick a category (scrollable list)
  OLED_SURVEY_CODE_CODE,    // Step 2: pick a code inside the chosen category
  OLED_SURVEY_MEASURE,      // Measuring screen (progress bar + live stats)
  OLED_SURVEY_RESULT,       // Result screen after measurement
  OLED_SURVEY_QUALITY_WARN, // Quality warning — click to force, long-press to cancel
  // ---- Stakeout states ----
  OLED_STAKEOUT_MENU,       // Stakeout sub-menu: [Navigate / Files / Back]
  OLED_STAKEOUT_FILE_LIST,  // List of stakeout files (scrollable, click = browse)
  OLED_STAKEOUT_POINT_LIST, // List of points in selected file (click = set active)
  OLED_STAKEOUT_NAV         // Live navigation: D2D / dH / Azimuth
};

namespace OledMenu {

  // ---- Lifecycle ----------------------------------------------------------
  void init();
  void handleInput(int direction, bool click, bool doubleClick, bool longPress);
  void draw(Adafruit_SSD1306& display);

  // Returns true when the menu is active (not OLED_NORMAL).
  // While true the caller should suppress oledUpdate() and call draw() instead.
  bool          isActive();
  OledMenuState getState();

  // ---- Action callbacks (set from main.cpp) --------------------------------
  extern void (*onToggleBLE)      (bool enable);
  extern void (*onToggleNtripIn)  (bool enable);
  extern void (*onToggleTcpIn)    (bool enable);
  extern void (*onToggleNtripOut) (bool enable);
  extern void (*onToggleTcpOutSrv)(bool enable);
  extern void (*onToggleTcpOutCli)(bool enable);
  extern void (*onToggleBuzzer)   (bool enable);
  extern void (*onSetRate)        (uint16_t measRateMs);
  extern void (*onReboot)         ();
  extern void (*onSyncToSD)       ();
  extern void (*onToggleRawLog)   (bool enable);

  // ---- Survey action callbacks (set from main.cpp) -----------------------
  extern void   (*onMeasurePoint)      ();            // trigger measurement on active survey
  extern void   (*onForceMeasurePoint) ();            // trigger measurement ignoring quality gate
  extern void   (*onCreateSurvey)      ();            // create new auto-named survey
  extern void   (*onSetActiveSurvey)   (int idx);     // set active survey by list index

  // Point-code readers (set from main.cpp)
  extern int    (*getCodeCatCount) ();
  extern String (*getCodeCatLabel) (int catIdx);
  extern int    (*getCodeCount)    (int catIdx);
  extern String (*getCodeCod)      (int catIdx, int codeIdx);
  extern String (*getCodeLabel)    (int catIdx, int codeIdx);
  // Called when user has chosen category+code and confirmed → launches measure
  extern void   (*onMeasureWithCode)(const String& cod, const String& label, bool force);

  // ---- State readers (set from main.cpp) -----------------------------------
  extern bool        (*getBleState)     ();
  extern bool        (*getNtripInState) ();
  extern bool        (*getTcpInState)   ();
  extern bool        (*getNtripOutState)();
  extern bool        (*getTcpOutSrvState)();
  extern bool        (*getTcpOutCliState)();
  extern bool        (*getBuzzerState)  ();
  extern const char* (*getRateString)   ();
  extern String      (*getInfoString)   ();
  extern bool        (*getRawLogState)  ();

  // ---- Survey state readers (set from main.cpp) ---------------------------
  extern bool   (*isMeasuring)          ();
  extern bool   (*getQualityOK)         ();            // returns true if quality is good for measuring
  extern String (*getMeasureProgressStr)();  // short progress string for OLED
  extern String (*getMeasureResult)     ();  // short result string after measure
  extern int    (*getSurveyCount)       ();  // total number of surveys
  extern String (*getSurveyLabel)       (int idx);  // "(*) Title (N pts)" for index idx

  // Selected code getters (readable from main.cpp for result display)
  const String& getSelectedCod();
  const String& getSelectedCodLabel();

  // ---- Stakeout action callbacks (set from main.cpp) ----------------------
  extern void   (*onSelectStakeoutFile) (int fileIdx);   // load file's points for browsing
  extern void   (*onSetStakeoutActive)  (int pointIdx);  // set active point (uses cached file)

  // ---- Stakeout state readers (set from main.cpp) -------------------------
  extern int    (*getStakeoutFileCount) ();
  extern String (*getStakeoutFileLabel) (int idx);
  extern int    (*getStakeoutPointCount)();               // count in currently-browsed file
  extern String (*getStakeoutPointLabel)(int idx);        // label for point in browsed file
  extern String (*getStakeoutNavString) ();               // multi-line nav string for NAV screen

} // namespace OledMenu

#endif // OLEDMENU_H
