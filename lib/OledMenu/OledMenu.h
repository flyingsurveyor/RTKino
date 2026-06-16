// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef OLEDMENU_H
#define OLEDMENU_H

#include <Arduino.h>
#include <Adafruit_SSD1306.h>

// ---------------------------------------------------------------------------
// OledMenu — navigable menu for RTKino OLED display
// Driven by RotaryInput events; coexists with normal oledUpdate() display.
// ---------------------------------------------------------------------------

enum OledMenuState {
  OLED_NORMAL,              // Standard RTKino display (oledUpdate() handles it)
  OLED_PAGE2,               // Second home page
  OLED_PAGE3,               // Third home page: system info
  OLED_MENU_MAIN,           // Main menu
  // ---- Settings ----
  OLED_MENU_SETTINGS,       // Settings root: Rete / Display / Sistema / Back
  OLED_MENU_SETTINGS_NET,   // Network toggles sub-menu
  OLED_MENU_SETTINGS_DISP,  // Display sub-menu
  OLED_MENU_SETTINGS_SYS,   // System actions sub-menu
  OLED_SUBMENU_RATE,        // ZED measurement rate selector
  OLED_SUBMENU_BRIGHT,      // Brightness selector
  OLED_INFO_SCREEN,         // Read-only info screen
  OLED_CONFIRM_REBOOT,      // Reboot confirmation
  // ---- Survey states ----
  OLED_SURVEY_MENU,         // Survey sub-menu
  OLED_SURVEY_LIST,         // List of surveys
  OLED_SURVEY_CODE_CAT,     // Step 1: category picker
  OLED_SURVEY_CODE_CODE,    // Step 2: code picker
  OLED_SURVEY_MEASURE,      // Measuring screen (progress)
  OLED_SURVEY_RESULT,       // Result screen
  OLED_SURVEY_QUALITY_WARN, // Quality warning
  // ---- Base mode states ----
  OLED_MENU_BASE,           // Base mode root menu
  OLED_BASE_MEASURE_DUR,    // Duration selection for base survey measurement
  OLED_BASE_MEASURING,      // Progress screen during base survey
  OLED_BASE_LIST,           // Saved bases list (from bases.txt)
  OLED_BASE_RESULT,         // Post-base-measurement result/confirmation
  OLED_BASE_CONFIRM_STOP,   // Confirmation before Stop base→rover
  OLED_BASE_CONFIRM_SVIN,   // Confirmation before Survey-in auto
  // ---- Stakeout states ----
  OLED_STAKEOUT_MENU,
  OLED_STAKEOUT_FILE_LIST,
  OLED_STAKEOUT_POINT_LIST,
  OLED_STAKEOUT_NAV
};

namespace OledMenu {

  // ---- Lifecycle ----------------------------------------------------------
  void init();
  void handleInput(int direction, bool click, bool doubleClick, bool longPress);
  void draw(Adafruit_SSD1306& display);
  bool          isActive();
  OledMenuState getState();

  // ---- Network toggle callbacks -------------------------------------------
  extern void (*onToggleBLE)      (bool enable);
  extern void (*onToggleNtripIn)  (bool enable);
  extern void (*onToggleTcpIn)    (bool enable);
  extern void (*onToggleNtripOut) (bool enable);
  extern void (*onToggleTcpOutSrv)(bool enable);
  extern void (*onToggleTcpOutCli)(bool enable);
  extern void (*onToggleBuzzer)   (bool enable);

  // ---- System action callbacks --------------------------------------------
  extern void (*onSetRate)      (uint16_t measRateMs);
  extern void (*onReboot)       ();
  extern void (*onSyncToSD)     ();
  extern void (*onToggleRawLog) (bool enable);

  // ---- Survey action callbacks --------------------------------------------
  extern void   (*onMeasurePoint)      ();
  extern void   (*onForceMeasurePoint) ();
  extern void   (*onQuickMeasure)      (); // quick measure (double-click, no code, configured duration)
  extern void   (*onCreateSurvey)      ();
  extern void   (*onSetActiveSurvey)   (int idx);
  extern int    (*getCodeCatCount)     ();
  extern String (*getCodeCatLabel)     (int catIdx);
  extern int    (*getCodeCount)        (int catIdx);
  extern String (*getCodeCod)          (int catIdx, int codeIdx);
  extern String (*getCodeLabel)        (int catIdx, int codeIdx);
  extern void   (*onMeasureWithCode)   (const String& cod, const String& label, bool force);

  // ---- State readers ------------------------------------------------------
  extern bool        (*getBleState)          ();
  extern bool        (*getNtripInState)      ();
  extern bool        (*getTcpInState)        ();
  extern bool        (*getNtripOutState)     ();
  extern bool        (*getTcpOutSrvState)    ();
  extern bool        (*getTcpOutCliState)    ();
  extern bool        (*getBuzzerState)       ();
  extern const char* (*getRateString)        ();
  extern String      (*getInfoString)        ();
  extern bool        (*getRawLogState)       ();
  extern bool        (*isMeasuring)          ();
  extern bool        (*getQualityOK)         ();
  extern String      (*getMeasureProgressStr)();
  extern String      (*getMeasureResult)     ();
  extern int         (*getSurveyCount)       ();
  extern String      (*getSurveyLabel)       (int idx);
  extern int         (*getQuickMeasureDuration)(); // configured quick-measure duration in seconds

  // ---- Base mode callbacks ------------------------------------------------
  extern void   (*onStartBaseFromPoint)(int durationSec); // measure → activate as fixed base
  extern void   (*onStartBaseFromList) (int idx);         // select from saved bases → activate
  extern void   (*onStartSurveyIn)     ();                // start ZED survey-in mode
  extern void   (*onStopBase)          ();                // return to rover mode
  extern bool   (*getBaseActive)       ();                // is base mode currently active?
  extern int    (*getBaseListCount)    ();                // count of saved bases
  extern String (*getBaseListLabel)    (int idx);         // "Name  lat lon" display string

  // ---- Stakeout callbacks -------------------------------------------------
  extern void   (*onSelectStakeoutFile) (int fileIdx);
  extern void   (*onSetStakeoutActive)  (int pointIdx);
  extern int    (*getStakeoutFileCount) ();
  extern String (*getStakeoutFileLabel) (int idx);
  extern int    (*getStakeoutPointCount)();
  extern String (*getStakeoutPointLabel)(int idx);
  extern String (*getStakeoutNavString) ();

  // Selected code getters (readable from main.cpp)
  const String& getSelectedCod();
  const String& getSelectedCodLabel();

} // namespace OledMenu

#endif // OLEDMENU_H
