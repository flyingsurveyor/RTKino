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
  // ---- RTCM IN ----
  OLED_MENU_RTCMIN,         // RTCM IN sub-menu: NTRIP IN / TCP IN toggles
  // ---- Settings ----
  OLED_MENU_SETTINGS,       // Settings root: Network / System / Back
  OLED_MENU_SETTINGS_NET,   // Network toggles sub-menu
  OLED_MENU_SETTINGS_SYS,   // System actions sub-menu
  OLED_SUBMENU_RATE,        // ZED measurement rate selector
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
  OLED_BASE_LIST,           // Saved bases list (from bases.txt) — select only
  OLED_BASE_OUTPUTS_MENU,   // "Seleziona uscite": NTRIP OUT / TCP Cli OUT + auto-start flags
  OLED_BASE_NTRIPOUT_LIST,  // Saved NTRIP OUT profiles — select only
  OLED_BASE_TCPCLI_LIST,    // Saved TCP Client OUT profiles — select only
  OLED_BASE_CONFIRM_START,  // Confirmation before Start Base Mode
  OLED_BASE_WAIT,           // Wait screen while Start/Stop is in progress (async)
  OLED_BASE_RESULT,         // Result of the last Start/Stop action
  OLED_BASE_CONFIRM_STOP,   // Confirmation before Stop base→rover
  OLED_BASE_CONFIRM_SVIN,   // Confirmation before Survey-in auto
  // ---- Stakeout states ----
  OLED_STAKEOUT_MENU,
  OLED_STAKEOUT_FILE_LIST,
  OLED_STAKEOUT_POINT_LIST,
  OLED_STAKEOUT_NAV,
  OLED_STAKEOUT_MAP,        // North-up map view, toggled from NAV via click
  // ---- Tracking states ----
  OLED_TRACK_MENU,          // root: Start/Stop, Settings, Back
  OLED_TRACK_SETTINGS,      // Trigger Mode, Threshold, Back
  OLED_TRACK_TRIGGER_MODE,  // Time / Distance picker
  OLED_TRACK_THRESHOLD      // context-sensitive preset picker (seconds or metres)
};

namespace OledMenu {

  // ---- Lifecycle ----------------------------------------------------------
  void init();
  void handleInput(int direction, bool click, bool doubleClick, bool longPress);
  void draw(Adafruit_SSD1306& display);
  bool          isActive();
  OledMenuState getState();

  // ---- Network toggle callbacks -------------------------------------------
  // NTRIP OUT / TCP OUT Srv / TCP OUT Cli moved to Base -> Seleziona uscite
  // (they need a profile selection, not just an on/off toggle).
  extern void (*onToggleBLE)      (bool enable);
  extern void (*onToggleNtripIn)  (bool enable);
  extern void (*onToggleTcpIn)    (bool enable);
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
  // Mirrors the WebUI's Base page: profiles are SELECTED first (persisted,
  // not activated), then Start Base Mode applies the selected profile +
  // starts whichever RTCM outputs are configured to auto-start, and Stop
  // does the same full reset as the WebUI's "Rover" button.
  extern void   (*onSelectBaseFromList)(int idx);          // mark a saved base as selected (no activation)
  extern void   (*onStartBaseMode)     ();                 // apply selected base + auto-start outputs (async)
  extern void   (*onStartSurveyIn)     ();                 // start ZED survey-in mode
  extern void   (*onStopBase)          ();                 // full switch-to-rover reset (async)
  extern bool   (*getBaseActive)       ();                 // is base mode currently active?
  extern int    (*getBaseListCount)    ();                 // count of saved bases
  extern String (*getBaseListLabel)    (int idx);          // display label for base at idx
  extern int    (*getSelectedBaseIdx)  ();                 // index of the currently selected base, -1 if none
  extern String (*getSelectedBaseLabel)();                 // label of the currently selected base, "" if none
  extern bool   (*isBaseActionPending) ();                 // true while Start/Stop is still in progress
  extern String (*getBaseActionResult)();                  // result text once Start/Stop has finished

  // ---- Base RTCM output profile callbacks ("Seleziona uscite") ------------
  // Same select-then-toggle-autostart model as the base station profile,
  // mirroring the WebUI's RTCM Outputs page. Profiles themselves (host/port/
  // mount/pass) are only editable from the WebUI.
  extern void   (*onSelectNtripOutFromList)(int idx);
  extern void   (*onSelectTcpCliFromList)  (int idx);
  extern void   (*onToggleAutoNtrip)       (bool enable);   // auto-start NTRIP OUT (+bundled TCP Srv) with base
  extern void   (*onToggleAutoTcpCli)      (bool enable);   // auto-start TCP Client OUT with base
  extern int    (*getNtripOutListCount)    ();
  extern String (*getNtripOutListLabel)    (int idx);
  extern int    (*getSelectedNtripOutIdx)  ();
  extern int    (*getTcpCliListCount)      ();
  extern String (*getTcpCliListLabel)      (int idx);
  extern int    (*getSelectedTcpCliIdx)    ();
  extern bool   (*getAutoNtripState)       ();
  extern bool   (*getAutoTcpCliState)      ();

  // ---- Stakeout callbacks -------------------------------------------------
  extern void   (*onSelectStakeoutFile) (int fileIdx);
  extern void   (*onSetStakeoutActive)  (int pointIdx);
  extern int    (*getStakeoutFileCount) ();
  extern String (*getStakeoutFileLabel) (int idx);
  extern int    (*getStakeoutPointCount)();
  extern String (*getStakeoutPointLabel)(int idx);
  extern String (*getStakeoutNavString) ();

  // Raw distance/azimuth for the map view (OLED_STAKEOUT_MAP), decoupled from
  // the Stakeout library's own StakeoutStatus struct so this UI library
  // doesn't need to depend on it.
  struct OledStakeoutNavData {
    bool    valid         = false;
    double  distance      = 0.0;  // 2D ground distance to target (m)
    double  azimuth       = 0.0;  // degrees from North, 0-360
    uint8_t roverCarrSoln = 0;    // 0=no RTK, 1=float, 2=fixed
  };
  extern void (*getStakeoutNavData)(OledStakeoutNavData& out);

  // ---- Tracking callbacks --------------------------------------------------
  extern void   (*onTrackStartStop)    ();
  extern void   (*onSetTrackTrigger)   (int mode);
  extern void   (*onSetTrackThreshold) (float value);
  extern bool   (*getTrackRecording)   ();
  extern int    (*getTrackTriggerMode) ();
  extern String (*getTrackStatusString)();

  // Selected code getters (readable from main.cpp)
  const String& getSelectedCod();
  const String& getSelectedCodLabel();

} // namespace OledMenu

#endif // OLEDMENU_H
