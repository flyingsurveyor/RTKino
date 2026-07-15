// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "OledMenu.h"

// ---------------------------------------------------------------------------
// Callback / state-reader function pointers — default to nullptr
// ---------------------------------------------------------------------------
namespace OledMenu {
  // Network toggles
  void (*onToggleBLE)      (bool) = nullptr;
  void (*onToggleNtripIn)  (bool) = nullptr;
  void (*onToggleTcpIn)    (bool) = nullptr;
  void (*onToggleBuzzer)   (bool) = nullptr;

  // System actions
  void (*onSetRate)     (uint16_t) = nullptr;
  void (*onReboot)      ()         = nullptr;
  void (*onSyncToSD)    ()         = nullptr;
  void (*onToggleRawLog)(bool)     = nullptr;

  // Survey actions
  void   (*onMeasurePoint)      ()    = nullptr;
  void   (*onForceMeasurePoint) ()    = nullptr;
  void   (*onQuickMeasure)      ()    = nullptr;
  void   (*onCreateSurvey)      ()    = nullptr;
  void   (*onSetActiveSurvey)   (int) = nullptr;
  int    (*getCodeCatCount) ()                               = nullptr;
  String (*getCodeCatLabel) (int)                            = nullptr;
  int    (*getCodeCount)    (int)                            = nullptr;
  String (*getCodeCod)      (int, int)                       = nullptr;
  String (*getCodeLabel)    (int, int)                       = nullptr;
  void   (*onMeasureWithCode)(const String&, const String&, bool) = nullptr;

  // State readers
  bool        (*getBleState)          () = nullptr;
  bool        (*getNtripInState)      () = nullptr;
  bool        (*getTcpInState)        () = nullptr;
  bool        (*getBuzzerState)       () = nullptr;
  const char* (*getRateString)        () = nullptr;
  String      (*getInfoString)        () = nullptr;
  bool        (*getRawLogState)       () = nullptr;
  bool        (*isMeasuring)          () = nullptr;
  bool        (*getQualityOK)         () = nullptr;
  String      (*getMeasureProgressStr)() = nullptr;
  String      (*getMeasureResult)     () = nullptr;
  int         (*getSurveyCount)       () = nullptr;
  String      (*getSurveyLabel)       (int) = nullptr;
  int         (*getQuickMeasureDuration)() = nullptr;

  // Base mode
  void   (*onSelectBaseFromList)(int)  = nullptr;
  void   (*onStartBaseMode)     ()     = nullptr;
  void   (*onStartSurveyIn)     ()     = nullptr;
  void   (*onStopBase)          ()     = nullptr;
  bool   (*getBaseActive)       ()     = nullptr;
  int    (*getBaseListCount)    ()     = nullptr;
  String (*getBaseListLabel)    (int)  = nullptr;
  int    (*getSelectedBaseIdx)  ()     = nullptr;
  String (*getSelectedBaseLabel)()     = nullptr;
  bool   (*isBaseActionPending) ()     = nullptr;
  String (*getBaseActionResult)()      = nullptr;

  // Base RTCM output profiles ("Seleziona uscite")
  void   (*onSelectNtripOutFromList)(int)  = nullptr;
  void   (*onSelectTcpCliFromList)  (int)  = nullptr;
  void   (*onToggleAutoNtrip)       (bool) = nullptr;
  void   (*onToggleAutoTcpCli)      (bool) = nullptr;
  int    (*getNtripOutListCount)    ()     = nullptr;
  String (*getNtripOutListLabel)    (int)  = nullptr;
  int    (*getSelectedNtripOutIdx)  ()     = nullptr;
  int    (*getTcpCliListCount)      ()     = nullptr;
  String (*getTcpCliListLabel)      (int)  = nullptr;
  int    (*getSelectedTcpCliIdx)    ()     = nullptr;
  bool   (*getAutoNtripState)       ()     = nullptr;
  bool   (*getAutoTcpCliState)      ()     = nullptr;

  // Stakeout
  void   (*onSelectStakeoutFile) (int)  = nullptr;
  void   (*onSetStakeoutActive)  (int)  = nullptr;
  int    (*getStakeoutFileCount) ()     = nullptr;
  String (*getStakeoutFileLabel) (int)  = nullptr;
  int    (*getStakeoutPointCount)()     = nullptr;
  String (*getStakeoutPointLabel)(int)  = nullptr;
  String (*getStakeoutNavString) ()     = nullptr;
  void   (*getStakeoutNavData)(OledStakeoutNavData&) = nullptr;

  // Survey map
  void   (*onEnterSurveyMap)      ()             = nullptr;
  bool   (*getSurveyMapRoverValid)()             = nullptr;
  int    (*getSurveyMapPointCount)()             = nullptr;
  bool   (*getSurveyMapPoint)(int, OledSurveyMapPoint&) = nullptr;

  // Tracking
  void   (*onTrackStartStop)    ()      = nullptr;
  void   (*onSetTrackTrigger)   (int)   = nullptr;
  void   (*onSetTrackThreshold) (float) = nullptr;
  bool   (*getTrackRecording)   ()      = nullptr;
  int    (*getTrackTriggerMode) ()      = nullptr;
  String (*getTrackStatusString)()      = nullptr;
}

// ---------------------------------------------------------------------------
// Display geometry (font 6×8, 128×64 screen)
// ---------------------------------------------------------------------------
static const int SCREEN_W       = 128;
static const int ROW_H          = 9;
static const int TITLE_Y        = 0;
static const int SEP_Y          = 8;
static const int ITEMS_Y_START  = 11;
static const int HINT_Y         = 56;
static const int MAX_VISIBLE    = 5;
static const int MAX_DISP_CHARS = 21;

// ---------------------------------------------------------------------------
// Main menu  (8 items)
// ---------------------------------------------------------------------------
// "Misura punto" was removed: it's redundant with Surveys -> Misura punto
// and with the home-screen double-click quick-measure shortcut.
// Index 0 = Surveys
// Index 1 = Map (survey points around current position)
// Index 2 = Stakeout
// Index 3 = Tracking
// Index 4 = RTCM IN
// Index 5 = Base
// Index 6 = Settings
// Index 7 = Back
static const char* MAIN_ITEMS[] = {
  "Surveys", "Map", "Stakeout", "Tracking", "RTCM IN", "Base", "Settings", "Back"
};
static const int MAIN_COUNT = 8;

// ---------------------------------------------------------------------------
// Settings root (3 items) — Display removed: contrast/dim has no visible
// effect on the actual OLED module in use, so it was dead UI.
// ---------------------------------------------------------------------------
static const char* SETTINGS_ROOT[] = { "Network", "System", "Back" };
static const int   SETTINGS_ROOT_COUNT = 3;

// ---------------------------------------------------------------------------
// Network sub-menu (2 items — all toggles except Back)
// NTRIP OUT / TCP OUT Srv / TCP OUT Cli moved to Base -> Seleziona uscite;
// NTRIP IN / TCP IN moved to their own top-level "RTCM IN" menu (see below),
// since they're primary GNSS correction inputs, not general network settings.
// ---------------------------------------------------------------------------
enum NetItem {
  NI_BLE = 0, NI_BACK,
  NI_COUNT
};
static const char* NET_LABELS[NI_COUNT] = {
  "BLE", "Back"
};

// ---------------------------------------------------------------------------
// RTCM IN sub-menu (3 items) — top-level main menu entry, between Tracking
// and Base. Moved out of Settings -> Network for direct access, since these
// toggles are used far more often than the rest of Settings.
// ---------------------------------------------------------------------------
enum RtcmInItem { RI_NTRIP_IN = 0, RI_TCP_IN, RI_BACK, RI_COUNT };
static const char* RTCMIN_ITEMS[RI_COUNT] = { "NTRIP IN", "TCP IN", "Back" };

// ---------------------------------------------------------------------------
// System sub-menu (6 items)
// ---------------------------------------------------------------------------
enum SysItem {
  SYI_RATE = 0, SYI_RAWLOG, SYI_SYNC_SD, SYI_REBOOT, SYI_INFO, SYI_BACK,
  SYI_COUNT
};
static const char* SYS_LABELS[SYI_COUNT] = {
  "ZED Rate", "Raw Log", "Sync SD", "Reboot", "Info", "Back"
};

// ---------------------------------------------------------------------------
// ZED Rate sub-menu
// ---------------------------------------------------------------------------
static const char*    RATE_LABELS[] = { "1 Hz", "2 Hz", "5 Hz", "10 Hz", "15 Hz" };
static const uint16_t RATE_MS[]     = { 1000,   500,    200,    100,     67       };
static const int      RATE_COUNT    = 5;

// ---------------------------------------------------------------------------
// Survey sub-menu (4 items)
// ---------------------------------------------------------------------------
static const char* SURVEY_ITEMS[] = { "Misura punto", "Rilievi", "Nuovo rilievo", "Back" };
static const int   SURVEY_COUNT   = 4;

// ---------------------------------------------------------------------------
// Survey map menu (3 items) — opened with a click while on OLED_SURVEY_MAP.
// ---------------------------------------------------------------------------
enum SurveyMapMenuItem { SMI_FIT_ALL = 0, SMI_ZOOM_TO_POINT, SMI_BACK, SMI_COUNT };
static const char* SURVEY_MAP_MENU_ITEMS[SMI_COUNT] = { "Fit all", "Zoom to point", "Back" };

// ---------------------------------------------------------------------------
// Base mode menu (6 items) — mirrors the WebUI's Base page: select a saved
// station + output profiles first, then Start/Stop act on whatever is
// currently selected.
// ---------------------------------------------------------------------------
static const char* BASE_ITEMS[] = {
  "Seleziona base",
  "Seleziona uscite",
  "Start Base Mode",
  "Survey-in auto",
  "Stop base->rover",
  "Back"
};
static const int BASE_COUNT = 6;

// ---------------------------------------------------------------------------
// Base RTCM outputs sub-menu (5 items) — select which NTRIP OUT / TCP Client
// OUT profile is active, and whether each auto-starts with Start Base Mode.
// TCP OUT Srv has no separate profile: it rides along with NTRIP OUT (same
// record, same auto-start flag), matching startAllBaseOutputs()/WebUI.
// ---------------------------------------------------------------------------
enum BaseOutItem { BOI_NTRIP = 0, BOI_TCPCLI, BOI_AUTO_NTRIP, BOI_AUTO_TCP, BOI_BACK, BOI_COUNT };
static const char* BASE_OUT_ITEMS[BOI_COUNT] = {
  "NTRIP OUT", "TCP Client OUT", "Auto NTRIP", "Auto TCP Cli", "Back"
};
static const int BASE_OUT_COUNT = BOI_COUNT;

// ---------------------------------------------------------------------------
// Stakeout sub-menu
// ---------------------------------------------------------------------------
static const char* STAKEOUT_ITEMS[] = { "Naviga", "File", "Back" };
static const int   STAKEOUT_COUNT   = 3;

// ---------------------------------------------------------------------------
// Map views (Stakeout + Survey) — shared layout and discrete zoom presets
// (cm per pixel), cycled with the rotary encoder. Fine end (1cm/px) matches
// survey/stakeout centimeter precision; coarse end (10m/px) covers a ~640m-
// wide view for a long walk to target.
// ---------------------------------------------------------------------------
static const float MAP_SCALE_CM[] = { 1, 2, 5, 10, 20, 50, 100, 200, 500, 1000 };
static const int   MAP_SCALE_COUNT = 10;
static const int   MAP_CX = 64;
static const int   MAP_CY = 28;   // map canvas is y=0..55; bottom line y=56..63 is the scale/distance readout
static const int   MAP_MAX_RADIUS_PX = 26;

// ---------------------------------------------------------------------------
// Tracking sub-menu
// ---------------------------------------------------------------------------
static const char* TRACK_MENU_ITEMS[]     = { "Start/Stop", "Settings", "Back" };
static const int   TRACK_MENU_COUNT       = 3;
static const char* TRACK_SETTINGS_ITEMS[] = { "Trigger Mode", "Threshold", "Back" };
static const int   TRACK_SETTINGS_COUNT   = 3;
static const char* TRACK_MODE_LABELS[]    = { "Time", "Distance" };
static const int   TRACK_MODE_COUNT       = 2;
static const char*  TRACK_TIME_LABELS[]  = { "1 sec", "2 sec", "5 sec", "10 sec", "30 sec", "60 sec" };
static const float  TRACK_TIME_VALUES[]  = { 1, 2, 5, 10, 30, 60 };
static const char*  TRACK_DIST_LABELS[]  = { "2 m", "5 m", "10 m", "20 m", "50 m", "100 m" };
static const float  TRACK_DIST_VALUES[]  = { 2, 5, 10, 20, 50, 100 };
static const int    TRACK_THRESHOLD_COUNT = 6;

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
static OledMenuState s_state       = OLED_NORMAL;
static int           s_cursor      = 0;
static int           s_scrollTop   = 0;
static int           s_listCount   = 0;

// Code-selection state
static int    s_selCatIdx    = 0;
static String s_selCod       = "";
static String s_selCodLbl    = "";

// Which base action is in flight, so OLED_BASE_RESULT knows which cursor
// position to return to in OLED_MENU_BASE ("Start" vs "Stop").
static bool   s_baseActionWasStop  = false;

// Current zoom level (index into MAP_SCALE_CM) for each map view
static int    s_stakeoutMapZoomIdx = 4;
static int    s_surveyMapZoomIdx   = 4;

// Survey map: index of the point highlighted by "Zoom to point" (-1 = none,
// i.e. plain "show everything" view). Cleared by Fit all / re-entering the map.
static int    s_surveyMapHighlightIdx = -1;

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------
static void enterMenu(OledMenuState newState, int itemCount, int startCursor = 0) {
  s_state     = newState;
  s_listCount = itemCount;
  s_cursor    = startCursor;
  s_scrollTop = 0;
  if (s_cursor >= MAX_VISIBLE) s_scrollTop = s_cursor - MAX_VISIBLE + 1;
}

static void scrollCursor(int delta) {
  s_cursor += delta;
  if (s_cursor < 0)            s_cursor = s_listCount - 1;
  if (s_cursor >= s_listCount) s_cursor = 0;
  if (s_cursor < s_scrollTop)
    s_scrollTop = s_cursor;
  else if (s_cursor >= s_scrollTop + MAX_VISIBLE)
    s_scrollTop = s_cursor - MAX_VISIBLE + 1;
}

static bool getBool(bool (*fn)()) { return fn ? fn() : false; }
static void callBool(void (*fn)(bool), bool val) { if (fn) fn(val); }

// ---------------------------------------------------------------------------
// Network item helpers
// ---------------------------------------------------------------------------
static bool netItemState(int idx) {
  switch (idx) {
    case NI_BLE: return getBool(OledMenu::getBleState);
    default:     return false;
  }
}

static void toggleNetItem(int idx) {
  bool next = !netItemState(idx);
  switch (idx) {
    case NI_BLE: callBool(OledMenu::onToggleBLE, next); break;
    default: break;
  }
}

// ---------------------------------------------------------------------------
// RTCM IN item helpers
// ---------------------------------------------------------------------------
static bool rtcmInItemState(int idx) {
  switch (idx) {
    case RI_NTRIP_IN: return getBool(OledMenu::getNtripInState);
    case RI_TCP_IN:   return getBool(OledMenu::getTcpInState);
    default:          return false;
  }
}

static void toggleRtcmInItem(int idx) {
  bool next = !rtcmInItemState(idx);
  switch (idx) {
    case RI_NTRIP_IN: callBool(OledMenu::onToggleNtripIn, next); break;
    case RI_TCP_IN:   callBool(OledMenu::onToggleTcpIn,   next); break;
    default: break;
  }
}

// ---------------------------------------------------------------------------
// Map zoom: pick the tightest preset that still keeps a given distance
// within the visible radius. Shared by the Stakeout map (single target) and
// the Survey map (farthest of N points, or one picked point).
// ---------------------------------------------------------------------------
static int zoomIdxForMaxDistanceCm(double maxDistCm) {
  for (int i = 0; i < MAP_SCALE_COUNT; i++) {
    if (maxDistCm / MAP_SCALE_CM[i] <= MAP_MAX_RADIUS_PX) return i;
  }
  return MAP_SCALE_COUNT - 1;  // beyond even the widest preset
}

// Stakeout map: auto-fit zoom to the single active target, so the map is
// useful the moment it's opened instead of showing it off-screen.
static void pickInitialStakeoutMapZoom() {
  if (!OledMenu::getStakeoutNavData) return;
  OledMenu::OledStakeoutNavData nav;
  OledMenu::getStakeoutNavData(nav);
  if (!nav.valid) return;
  s_stakeoutMapZoomIdx = zoomIdxForMaxDistanceCm(nav.distance * 100.0);
}

// Survey map: auto-fit zoom to the farthest point of the active survey, so
// every point is visible the moment the map is opened. Also clears any
// "Zoom to point" highlight — Fit all is the plain "show everything" view.
static void fitAllSurveyMapPoints() {
  s_surveyMapHighlightIdx = -1;
  if (!OledMenu::getSurveyMapRoverValid || !OledMenu::getSurveyMapRoverValid()) return;
  int cnt = OledMenu::getSurveyMapPointCount ? OledMenu::getSurveyMapPointCount() : 0;
  double maxDistCm = 0.0;
  for (int i = 0; i < cnt; i++) {
    OledMenu::OledSurveyMapPoint p;
    if (OledMenu::getSurveyMapPoint && OledMenu::getSurveyMapPoint(i, p)) {
      double distCm = p.distance * 100.0;
      if (distCm > maxDistCm) maxDistCm = distCm;
    }
  }
  s_surveyMapZoomIdx = zoomIdxForMaxDistanceCm(maxDistCm);
}

// Survey map: auto-fit zoom to a single chosen point ("Zoom to point") and
// highlight it with a circle so the effect is visible even when its zoom
// bucket happens to match the previous one (e.g. tightly-clustered points).
static void zoomToSurveyMapPoint(int idx) {
  if (!OledMenu::getSurveyMapPoint) return;
  OledMenu::OledSurveyMapPoint p;
  if (OledMenu::getSurveyMapPoint(idx, p)) {
    s_surveyMapZoomIdx      = zoomIdxForMaxDistanceCm(p.distance * 100.0);
    s_surveyMapHighlightIdx = idx;
  }
}

// ---------------------------------------------------------------------------
// Launch a measurement for survey (normal path)
// ---------------------------------------------------------------------------
static void launchSurveyMeasure(bool force) {
  if (OledMenu::onMeasureWithCode) {
    OledMenu::onMeasureWithCode(s_selCod, s_selCodLbl, force);
  } else if (force) {
    if (OledMenu::onForceMeasurePoint) OledMenu::onForceMeasurePoint();
  } else {
    if (OledMenu::onMeasurePoint) OledMenu::onMeasurePoint();
  }
  s_state = OLED_SURVEY_MEASURE;
}

// ---------------------------------------------------------------------------
// OledMenu::init
// ---------------------------------------------------------------------------
namespace OledMenu {

void init() {
  s_state          = OLED_NORMAL;
  s_cursor         = 0;
  s_scrollTop      = 0;
  s_listCount      = 0;
}

bool isActive() {
  return s_state != OLED_NORMAL && s_state != OLED_PAGE2 && s_state != OLED_PAGE3;
}

OledMenuState getState() { return s_state; }

// ---------------------------------------------------------------------------
// handleInput
// ---------------------------------------------------------------------------
void handleInput(int direction, bool click, bool doubleClick, bool longPress) {

  // ---- Double-click from home: quick measure (no code, configured duration) ----
  if (doubleClick) {
    if (s_state == OLED_NORMAL || s_state == OLED_PAGE2 || s_state == OLED_PAGE3) {
      s_selCod      = "";
      s_selCodLbl   = "";
      bool ok = (!OledMenu::getQualityOK || OledMenu::getQualityOK());
      if (!ok) {
        s_state = OLED_SURVEY_QUALITY_WARN;
      } else {
        if (OledMenu::onQuickMeasure) OledMenu::onQuickMeasure();
        else if (OledMenu::onMeasurePoint) OledMenu::onMeasurePoint();
        s_state = OLED_SURVEY_MEASURE;
      }
      return;
    }
  }

  // ---- Long press: navigate up ----
  if (longPress) {
    switch (s_state) {
      case OLED_NORMAL:
      case OLED_PAGE2:
      case OLED_PAGE3:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT);
        break;
      case OLED_MENU_MAIN:
        s_state = OLED_NORMAL;
        break;
      // RTCM IN
      case OLED_MENU_RTCMIN:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 4);
        break;
      // Settings
      case OLED_MENU_SETTINGS:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 6);
        break;
      case OLED_MENU_SETTINGS_NET:
      case OLED_MENU_SETTINGS_SYS:
        enterMenu(OLED_MENU_SETTINGS, SETTINGS_ROOT_COUNT);
        break;
      case OLED_SUBMENU_RATE:
        enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_RATE);
        break;
      case OLED_INFO_SCREEN:
        enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_INFO);
        break;
      case OLED_CONFIRM_REBOOT:
        enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_REBOOT);
        break;
      // Surveys
      case OLED_SURVEY_MENU:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 0);
        break;
      case OLED_SURVEY_LIST:
        enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT, 1);
        break;
      case OLED_SURVEY_CODE_CAT:
        enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
        break;
      case OLED_SURVEY_CODE_CODE: {
        int catCount = OledMenu::getCodeCatCount ? OledMenu::getCodeCatCount() : 0;
        enterMenu(OLED_SURVEY_CODE_CAT, max(catCount, 1), s_selCatIdx);
        break;
      }
      case OLED_SURVEY_MEASURE:
      case OLED_SURVEY_RESULT:
      case OLED_SURVEY_QUALITY_WARN:
        enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
        break;
      // Survey map
      case OLED_SURVEY_MAP:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 1);
        break;
      case OLED_SURVEY_MAP_MENU:
        s_state = OLED_SURVEY_MAP;
        break;
      case OLED_SURVEY_MAP_POINT_LIST:
        enterMenu(OLED_SURVEY_MAP_MENU, SMI_COUNT, SMI_ZOOM_TO_POINT);
        break;
      // Base mode
      case OLED_MENU_BASE:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 5);
        break;
      case OLED_BASE_LIST:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, 0);
        break;
      case OLED_BASE_OUTPUTS_MENU:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, 1);
        break;
      case OLED_BASE_NTRIPOUT_LIST:
        enterMenu(OLED_BASE_OUTPUTS_MENU, BASE_OUT_COUNT, BOI_NTRIP);
        break;
      case OLED_BASE_TCPCLI_LIST:
        enterMenu(OLED_BASE_OUTPUTS_MENU, BASE_OUT_COUNT, BOI_TCPCLI);
        break;
      case OLED_BASE_CONFIRM_START:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, 2);
        break;
      case OLED_BASE_WAIT:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, s_baseActionWasStop ? 4 : 2);
        break;
      case OLED_BASE_RESULT:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, s_baseActionWasStop ? 4 : 2);
        break;
      case OLED_BASE_CONFIRM_STOP:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, 4);
        break;
      case OLED_BASE_CONFIRM_SVIN:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, 3);
        break;
      // Stakeout
      case OLED_STAKEOUT_MENU:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 2);
        break;
      case OLED_STAKEOUT_FILE_LIST:
        enterMenu(OLED_STAKEOUT_MENU, STAKEOUT_COUNT);
        break;
      case OLED_STAKEOUT_POINT_LIST:
        enterMenu(OLED_STAKEOUT_FILE_LIST,
                  OledMenu::getStakeoutFileCount ? OledMenu::getStakeoutFileCount() : 1);
        break;
      case OLED_STAKEOUT_NAV:
      case OLED_STAKEOUT_MAP:
        enterMenu(OLED_STAKEOUT_MENU, STAKEOUT_COUNT);
        break;
      // Tracking
      case OLED_TRACK_MENU:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 3);
        break;
      case OLED_TRACK_SETTINGS:
        enterMenu(OLED_TRACK_MENU, TRACK_MENU_COUNT, 1);
        break;
      case OLED_TRACK_TRIGGER_MODE:
        enterMenu(OLED_TRACK_SETTINGS, TRACK_SETTINGS_COUNT, 0);
        break;
      case OLED_TRACK_THRESHOLD:
        enterMenu(OLED_TRACK_SETTINGS, TRACK_SETTINGS_COUNT, 1);
        break;
      default:
        s_state = OLED_NORMAL;
        break;
    }
    return;
  }

  // ---- Rotation: scroll / cycle home pages ----
  if (direction != 0) {
    if (s_state == OLED_NORMAL || s_state == OLED_PAGE2 || s_state == OLED_PAGE3) {
      if (direction > 0) {
        if      (s_state == OLED_NORMAL) s_state = OLED_PAGE2;
        else if (s_state == OLED_PAGE2)  s_state = OLED_PAGE3;
        else                             s_state = OLED_NORMAL;
      } else {
        if      (s_state == OLED_NORMAL) s_state = OLED_PAGE3;
        else if (s_state == OLED_PAGE2)  s_state = OLED_NORMAL;
        else                             s_state = OLED_PAGE2;
      }
      return;
    }
    // ---- Map views: rotation zooms in/out instead of scrolling a list ----
    if (s_state == OLED_STAKEOUT_MAP || s_state == OLED_SURVEY_MAP) {
      int& zoomIdx = (s_state == OLED_STAKEOUT_MAP) ? s_stakeoutMapZoomIdx : s_surveyMapZoomIdx;
      zoomIdx += (direction > 0) ? 1 : -1;
      if (zoomIdx < 0) zoomIdx = 0;
      if (zoomIdx >= MAP_SCALE_COUNT) zoomIdx = MAP_SCALE_COUNT - 1;
      return;
    }
    switch (s_state) {
      case OLED_MENU_MAIN:
      case OLED_MENU_RTCMIN:
      case OLED_MENU_SETTINGS:
      case OLED_MENU_SETTINGS_NET:
      case OLED_MENU_SETTINGS_SYS:
      case OLED_SUBMENU_RATE:
      case OLED_SURVEY_MENU:
      case OLED_SURVEY_LIST:
      case OLED_SURVEY_CODE_CAT:
      case OLED_SURVEY_CODE_CODE:
      case OLED_SURVEY_MAP_MENU:
      case OLED_SURVEY_MAP_POINT_LIST:
      case OLED_MENU_BASE:
      case OLED_BASE_LIST:
      case OLED_BASE_OUTPUTS_MENU:
      case OLED_BASE_NTRIPOUT_LIST:
      case OLED_BASE_TCPCLI_LIST:
      case OLED_STAKEOUT_MENU:
      case OLED_STAKEOUT_FILE_LIST:
      case OLED_STAKEOUT_POINT_LIST:
      case OLED_TRACK_MENU:
      case OLED_TRACK_SETTINGS:
      case OLED_TRACK_TRIGGER_MODE:
      case OLED_TRACK_THRESHOLD:
        scrollCursor(direction);
        break;
      default:
        break;
    }
    return;
  }

  // ---- Click: confirm / toggle / enter ----
  if (!click) return;

  switch (s_state) {

    // ---- Main menu ----
    case OLED_MENU_MAIN:
      switch (s_cursor) {
        case 0: // Surveys
          enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
          break;
        case 1: // Map
          if (OledMenu::onEnterSurveyMap) OledMenu::onEnterSurveyMap();
          fitAllSurveyMapPoints();
          s_state = OLED_SURVEY_MAP;
          break;
        case 2: // Stakeout
          enterMenu(OLED_STAKEOUT_MENU, STAKEOUT_COUNT);
          break;
        case 3: // Tracking
          enterMenu(OLED_TRACK_MENU, TRACK_MENU_COUNT);
          break;
        case 4: { // RTCM IN
          enterMenu(OLED_MENU_RTCMIN, RI_COUNT);
          break;
        }
        case 5: // Base
          enterMenu(OLED_MENU_BASE, BASE_COUNT);
          break;
        case 6: // Settings
          enterMenu(OLED_MENU_SETTINGS, SETTINGS_ROOT_COUNT);
          break;
        case 7: // Back
          s_state = OLED_NORMAL;
          break;
      }
      break;

    // ---- RTCM IN sub-menu ----
    case OLED_MENU_RTCMIN:
      if (s_cursor == RI_BACK) {
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 4);
      } else {
        toggleRtcmInItem(s_cursor);
      }
      break;

    // ---- Settings root ----
    case OLED_MENU_SETTINGS:
      switch (s_cursor) {
        case 0: enterMenu(OLED_MENU_SETTINGS_NET, NI_COUNT);  break;
        case 1: enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT); break;
        case 2: enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 6);     break;
      }
      break;

    // ---- Network sub-menu ----
    case OLED_MENU_SETTINGS_NET:
      if (s_cursor == NI_BACK) {
        enterMenu(OLED_MENU_SETTINGS, SETTINGS_ROOT_COUNT, 0);
      } else {
        toggleNetItem(s_cursor);
      }
      break;

    // ---- System sub-menu ----
    case OLED_MENU_SETTINGS_SYS:
      switch (s_cursor) {
        case SYI_RATE:
          enterMenu(OLED_SUBMENU_RATE, RATE_COUNT);
          break;
        case SYI_RAWLOG:
          if (OledMenu::onToggleRawLog) {
            bool cur = getBool(OledMenu::getRawLogState);
            OledMenu::onToggleRawLog(!cur);
          }
          break;
        case SYI_SYNC_SD:
          if (OledMenu::onSyncToSD) OledMenu::onSyncToSD();
          break;
        case SYI_REBOOT:
          s_state = OLED_CONFIRM_REBOOT;
          break;
        case SYI_INFO:
          s_state = OLED_INFO_SCREEN;
          break;
        case SYI_BACK:
          enterMenu(OLED_MENU_SETTINGS, SETTINGS_ROOT_COUNT, 1);
          break;
      }
      break;

    // ---- ZED Rate sub-menu ----
    case OLED_SUBMENU_RATE:
      if (OledMenu::onSetRate) OledMenu::onSetRate(RATE_MS[s_cursor]);
      enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_RATE);
      break;

    // ---- Info screen ----
    case OLED_INFO_SCREEN:
      enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_INFO);
      break;

    // ---- Reboot confirmation ----
    case OLED_CONFIRM_REBOOT:
      if (OledMenu::onReboot) OledMenu::onReboot();
      break;

    // ---- Survey sub-menu ----
    case OLED_SURVEY_MENU:
      switch (s_cursor) {
        case 0: { // Misura punto
          int catCount = OledMenu::getCodeCatCount ? OledMenu::getCodeCatCount() : 0;
          s_selCod = ""; s_selCodLbl = "";
          if (catCount > 0) {
            enterMenu(OLED_SURVEY_CODE_CAT, catCount, s_selCatIdx);
          } else {
            bool ok = (!OledMenu::getQualityOK || OledMenu::getQualityOK());
            if (!ok) s_state = OLED_SURVEY_QUALITY_WARN;
            else launchSurveyMeasure(false);
          }
          break;
        }
        case 1: { // Rilievi (list)
          int cnt = OledMenu::getSurveyCount ? OledMenu::getSurveyCount() : 0;
          enterMenu(OLED_SURVEY_LIST, max(cnt, 1));
          break;
        }
        case 2: // Nuovo rilievo
          if (OledMenu::onCreateSurvey) OledMenu::onCreateSurvey();
          break;
        case 3: // Back
          enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 0);
          break;
      }
      break;

    // ---- Survey list ----
    case OLED_SURVEY_LIST:
      if (OledMenu::onSetActiveSurvey) OledMenu::onSetActiveSurvey(s_cursor);
      enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT, 1);
      break;

    // ---- Code category ----
    case OLED_SURVEY_CODE_CAT: {
      s_selCatIdx = s_cursor;
      int codeCount = OledMenu::getCodeCount ? OledMenu::getCodeCount(s_selCatIdx) : 0;
      enterMenu(OLED_SURVEY_CODE_CODE, max(codeCount, 1));
      break;
    }

    // ---- Code selection ----
    case OLED_SURVEY_CODE_CODE: {
      if (OledMenu::getCodeCod && OledMenu::getCodeLabel) {
        s_selCod    = OledMenu::getCodeCod(s_selCatIdx, s_cursor);
        s_selCodLbl = OledMenu::getCodeLabel(s_selCatIdx, s_cursor);
      }
      bool ok = (!OledMenu::getQualityOK || OledMenu::getQualityOK());
      if (!ok) s_state = OLED_SURVEY_QUALITY_WARN;
      else launchSurveyMeasure(false);
      break;
    }

    // ---- Measure screen ----
    case OLED_SURVEY_MEASURE: {
      bool m = OledMenu::isMeasuring ? OledMenu::isMeasuring() : false;
      if (!m) s_state = OLED_SURVEY_RESULT;
      break;
    }

    // ---- Result screen ----
    case OLED_SURVEY_RESULT:
      enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
      break;

    // ---- Quality warning ----
    case OLED_SURVEY_QUALITY_WARN:
      launchSurveyMeasure(true);
      break;

    // ---- Survey map: click dismisses an active highlight, else opens menu ----
    case OLED_SURVEY_MAP:
      if (s_surveyMapHighlightIdx >= 0) {
        s_surveyMapHighlightIdx = -1;
      } else {
        enterMenu(OLED_SURVEY_MAP_MENU, SMI_COUNT);
      }
      break;

    // ---- Survey map menu ----
    case OLED_SURVEY_MAP_MENU:
      switch (s_cursor) {
        case SMI_FIT_ALL:
          fitAllSurveyMapPoints();
          s_state = OLED_SURVEY_MAP;
          break;
        case SMI_ZOOM_TO_POINT: {
          int cnt = OledMenu::getSurveyMapPointCount ? OledMenu::getSurveyMapPointCount() : 0;
          enterMenu(OLED_SURVEY_MAP_POINT_LIST, max(cnt, 1));
          break;
        }
        case SMI_BACK:
          s_state = OLED_SURVEY_MAP;
          break;
      }
      break;

    // ---- Survey map: pick a point to zoom to ----
    case OLED_SURVEY_MAP_POINT_LIST:
      zoomToSurveyMapPoint(s_cursor);
      s_state = OLED_SURVEY_MAP;
      break;

    // ---- Base mode root ----
    case OLED_MENU_BASE:
      switch (s_cursor) {
        case 0: { // Seleziona base — pick which saved profile is active (no activation yet)
          int cnt = OledMenu::getBaseListCount ? OledMenu::getBaseListCount() : 0;
          enterMenu(OLED_BASE_LIST, max(cnt, 1));
          break;
        }
        case 1: // Seleziona uscite — NTRIP OUT / TCP Client OUT profiles + auto-start
          enterMenu(OLED_BASE_OUTPUTS_MENU, BASE_OUT_COUNT);
          break;
        case 2: // Start Base Mode → ask confirmation (shows selected station)
          s_state = OLED_BASE_CONFIRM_START;
          break;
        case 3: // Survey-in auto → ask confirmation first
          s_state = OLED_BASE_CONFIRM_SVIN;
          break;
        case 4: // Stop base → rover → ask confirmation first
          s_state = OLED_BASE_CONFIRM_STOP;
          break;
        case 5: // Back
          enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 5);
          break;
      }
      break;

    // ---- Base outputs sub-menu (NTRIP OUT / TCP Client OUT selection) ----
    case OLED_BASE_OUTPUTS_MENU:
      switch (s_cursor) {
        case BOI_NTRIP: {
          int cnt = OledMenu::getNtripOutListCount ? OledMenu::getNtripOutListCount() : 0;
          enterMenu(OLED_BASE_NTRIPOUT_LIST, max(cnt, 1));
          break;
        }
        case BOI_TCPCLI: {
          int cnt = OledMenu::getTcpCliListCount ? OledMenu::getTcpCliListCount() : 0;
          enterMenu(OLED_BASE_TCPCLI_LIST, max(cnt, 1));
          break;
        }
        case BOI_AUTO_NTRIP:
          callBool(OledMenu::onToggleAutoNtrip, !getBool(OledMenu::getAutoNtripState));
          break;
        case BOI_AUTO_TCP:
          callBool(OledMenu::onToggleAutoTcpCli, !getBool(OledMenu::getAutoTcpCliState));
          break;
        case BOI_BACK:
          enterMenu(OLED_MENU_BASE, BASE_COUNT, 1);
          break;
      }
      break;

    // ---- NTRIP OUT profile list (select only) ----
    case OLED_BASE_NTRIPOUT_LIST:
      if (OledMenu::onSelectNtripOutFromList) OledMenu::onSelectNtripOutFromList(s_cursor);
      enterMenu(OLED_BASE_OUTPUTS_MENU, BASE_OUT_COUNT, BOI_NTRIP);
      break;

    // ---- TCP Client OUT profile list (select only) ----
    case OLED_BASE_TCPCLI_LIST:
      if (OledMenu::onSelectTcpCliFromList) OledMenu::onSelectTcpCliFromList(s_cursor);
      enterMenu(OLED_BASE_OUTPUTS_MENU, BASE_OUT_COUNT, BOI_TCPCLI);
      break;

    // ---- Confirm: Start Base Mode ----
    case OLED_BASE_CONFIRM_START:
      s_baseActionWasStop = false;
      if (OledMenu::onStartBaseMode) OledMenu::onStartBaseMode();
      s_state = OLED_BASE_WAIT;
      break;

    // ---- Wait screen: polls isBaseActionPending() from draw() ----
    case OLED_BASE_WAIT: {
      bool pending = OledMenu::isBaseActionPending ? OledMenu::isBaseActionPending() : false;
      if (!pending) s_state = OLED_BASE_RESULT;
      break;
    }

    // ---- Base result screen ----
    case OLED_BASE_RESULT:
      enterMenu(OLED_MENU_BASE, BASE_COUNT, s_baseActionWasStop ? 4 : 2);
      break;

    // ---- Confirm: Stop base → rover ----
    case OLED_BASE_CONFIRM_STOP:
      s_baseActionWasStop = true;
      if (OledMenu::onStopBase) OledMenu::onStopBase();
      s_state = OLED_BASE_WAIT;
      break;

    // ---- Confirm: Survey-in auto ----
    case OLED_BASE_CONFIRM_SVIN:
      if (OledMenu::onStartSurveyIn) OledMenu::onStartSurveyIn();
      enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 5);
      break;

    // ---- Base list (select only) ----
    case OLED_BASE_LIST:
      if (OledMenu::onSelectBaseFromList) OledMenu::onSelectBaseFromList(s_cursor);
      enterMenu(OLED_MENU_BASE, BASE_COUNT);
      break;

    // ---- Stakeout sub-menu ----
    case OLED_STAKEOUT_MENU:
      switch (s_cursor) {
        case 0: s_state = OLED_STAKEOUT_NAV; break;
        case 1: {
          int cnt = OledMenu::getStakeoutFileCount ? OledMenu::getStakeoutFileCount() : 0;
          enterMenu(OLED_STAKEOUT_FILE_LIST, max(cnt, 1));
          break;
        }
        case 2: enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 2); break;
      }
      break;

    // ---- Stakeout file list ----
    case OLED_STAKEOUT_FILE_LIST:
      if (OledMenu::onSelectStakeoutFile) OledMenu::onSelectStakeoutFile(s_cursor);
      {
        int cnt = OledMenu::getStakeoutPointCount ? OledMenu::getStakeoutPointCount() : 0;
        enterMenu(OLED_STAKEOUT_POINT_LIST, max(cnt, 1));
      }
      break;

    // ---- Stakeout point list ----
    case OLED_STAKEOUT_POINT_LIST:
      if (OledMenu::onSetStakeoutActive) OledMenu::onSetStakeoutActive(s_cursor);
      s_state = OLED_STAKEOUT_NAV;
      break;

    // ---- Stakeout nav screen (click toggles into the map view) ----
    case OLED_STAKEOUT_NAV:
      pickInitialStakeoutMapZoom();
      s_state = OLED_STAKEOUT_MAP;
      break;

    // ---- Stakeout map view (click toggles back to the text nav screen) ----
    case OLED_STAKEOUT_MAP:
      s_state = OLED_STAKEOUT_NAV;
      break;

    // ---- Tracking sub-menu ----
    case OLED_TRACK_MENU:
      switch (s_cursor) {
        case 0: if (OledMenu::onTrackStartStop) OledMenu::onTrackStartStop(); break;
        case 1: enterMenu(OLED_TRACK_SETTINGS, TRACK_SETTINGS_COUNT); break;
        case 2: enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 3); break;
      }
      break;

    // ---- Tracking settings ----
    case OLED_TRACK_SETTINGS:
      switch (s_cursor) {
        case 0: enterMenu(OLED_TRACK_TRIGGER_MODE, TRACK_MODE_COUNT); break;
        case 1: enterMenu(OLED_TRACK_THRESHOLD, TRACK_THRESHOLD_COUNT); break;
        case 2: enterMenu(OLED_TRACK_MENU, TRACK_MENU_COUNT, 1); break;
      }
      break;

    // ---- Tracking trigger mode picker ----
    case OLED_TRACK_TRIGGER_MODE:
      if (OledMenu::onSetTrackTrigger) OledMenu::onSetTrackTrigger(s_cursor);
      enterMenu(OLED_TRACK_SETTINGS, TRACK_SETTINGS_COUNT, 0);
      break;

    // ---- Tracking threshold picker (seconds or metres, depending on mode) ----
    case OLED_TRACK_THRESHOLD: {
      int mode = OledMenu::getTrackTriggerMode ? OledMenu::getTrackTriggerMode() : 0;
      float val = (mode == 0) ? TRACK_TIME_VALUES[s_cursor] : TRACK_DIST_VALUES[s_cursor];
      if (OledMenu::onSetTrackThreshold) OledMenu::onSetTrackThreshold(val);
      enterMenu(OLED_TRACK_SETTINGS, TRACK_SETTINGS_COUNT, 1);
      break;
    }

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// draw
// ---------------------------------------------------------------------------
void draw(Adafruit_SSD1306& disp) {

  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(1);

  auto dashedLine = [&](int y) {
    for (int x = 0; x < SCREEN_W; x += 4) {
      disp.drawPixel(x,   y, SSD1306_WHITE);
      disp.drawPixel(x+1, y, SSD1306_WHITE);
    }
  };

  auto drawItem = [&](int y, const char* label, const char* rightLabel, bool selected) {
    if (selected) {
      disp.fillRect(0, y, SCREEN_W, ROW_H - 1, SSD1306_WHITE);
      disp.setTextColor(SSD1306_BLACK);
    } else {
      disp.setTextColor(SSD1306_WHITE);
    }
    disp.setCursor(2, y);
    disp.print(label);
    if (rightLabel && rightLabel[0] != '\0') {
      int16_t x1, y1; uint16_t w, h;
      disp.getTextBounds(rightLabel, 0, 0, &x1, &y1, &w, &h);
      disp.setCursor(SCREEN_W - w - 2, y);
      disp.print(rightLabel);
    }
    if (selected) disp.setTextColor(SSD1306_WHITE);
  };

  auto drawHint = [&](const char* hint) {
    disp.setTextColor(SSD1306_WHITE);
    disp.setCursor(0, HINT_Y);
    disp.print(hint);
  };

  // Shared "pick one from a saved-profile list" screen — used for base
  // station, NTRIP OUT and TCP Client OUT selection (all three are select-
  // only lists: the currently selected entry is marked '*', editing the
  // profiles themselves stays on the WebUI).
  auto drawSelectableList = [&](const char* title, int cnt, String (*getLabel)(int), int selIdx) {
    disp.setCursor(0, TITLE_Y);
    disp.print(title);
    dashedLine(SEP_Y);
    if (cnt == 0) {
      disp.setCursor(0, ITEMS_Y_START);
      disp.print("Nessun profilo");
    } else {
      for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        String lbl = getLabel ? getLabel(i) : String(i);
        lbl = String((i == selIdx) ? "*" : " ") + lbl;
        if (lbl.length() > MAX_DISP_CHARS) lbl = lbl.substring(0, MAX_DISP_CHARS);
        drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
      }
    }
    drawHint("^ scroll *sel <back");
  };

  // =========================================================================
  switch (s_state) {

    // ---- Main menu ----
    case OLED_MENU_MAIN: {
      disp.setCursor(0, TITLE_Y);
      disp.print("  MENU");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < s_listCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, MAIN_ITEMS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Settings root ----
    case OLED_MENU_SETTINGS: {
      disp.setCursor(0, TITLE_Y);
      disp.print("* SETTINGS");
      dashedLine(SEP_Y);
      for (int i = 0; i < SETTINGS_ROOT_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        drawItem(y, SETTINGS_ROOT[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Network sub-menu ----
    case OLED_MENU_SETTINGS_NET: {
      disp.setCursor(0, TITLE_Y);
      disp.print("NETWORK");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < NI_COUNT && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        char right[8] = "";
        if (i != NI_BACK) strncpy(right, netItemState(i) ? "ON" : "OFF", sizeof(right)-1);
        drawItem(y, NET_LABELS[i], right, i == s_cursor);
      }
      drawHint("^ scroll *toggle <back");
      break;
    }

    // ---- RTCM IN sub-menu ----
    case OLED_MENU_RTCMIN: {
      disp.setCursor(0, TITLE_Y);
      disp.print("RTCM IN");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < RI_COUNT && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        char right[8] = "";
        if (i != RI_BACK) strncpy(right, rtcmInItemState(i) ? "ON" : "OFF", sizeof(right)-1);
        drawItem(y, RTCMIN_ITEMS[i], right, i == s_cursor);
      }
      drawHint("^ scroll *toggle <back");
      break;
    }

    // ---- System sub-menu ----
    case OLED_MENU_SETTINGS_SYS: {
      disp.setCursor(0, TITLE_Y);
      disp.print("SISTEMA");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < SYI_COUNT && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        char right[10] = "";
        if (i == SYI_RATE && OledMenu::getRateString)
          strncpy(right, OledMenu::getRateString(), sizeof(right)-1);
        else if (i == SYI_RAWLOG)
          strncpy(right, getBool(OledMenu::getRawLogState) ? "ON" : "OFF", sizeof(right)-1);
        drawItem(y, SYS_LABELS[i], right, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- ZED Rate sub-menu ----
    case OLED_SUBMENU_RATE: {
      disp.setCursor(0, TITLE_Y);
      disp.print("ZED RATE");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < RATE_COUNT && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, RATE_LABELS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *set <back");
      break;
    }

    // ---- Info screen ----
    case OLED_INFO_SCREEN: {
      disp.setCursor(0, TITLE_Y);
      disp.print("INFO");
      dashedLine(SEP_Y);
      if (OledMenu::getInfoString) {
        String info = OledMenu::getInfoString();
        int lineIdx = 0, start = 0;
        while (start < (int)info.length() && lineIdx < MAX_VISIBLE) {
          int end = info.indexOf('\n', start);
          if (end < 0) end = info.length();
          disp.setCursor(0, ITEMS_Y_START + lineIdx * ROW_H);
          disp.print(info.substring(start, end));
          start = end + 1; lineIdx++;
        }
      }
      drawHint("*=back");
      break;
    }

    // ---- Reboot confirmation ----
    case OLED_CONFIRM_REBOOT: {
      disp.setCursor(0, 20);
      disp.print("Reboot?");
      disp.setCursor(0, 32);
      disp.print("*=Si  <=No");
      break;
    }

    // ---- Survey sub-menu ----
    case OLED_SURVEY_MENU: {
      disp.setCursor(0, TITLE_Y);
      disp.print("* RILIEVI");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < SURVEY_COUNT && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, SURVEY_ITEMS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Survey list ----
    case OLED_SURVEY_LIST: {
      disp.setCursor(0, TITLE_Y);
      disp.print("RILIEVI");
      dashedLine(SEP_Y);
      int cnt = OledMenu::getSurveyCount ? OledMenu::getSurveyCount() : 0;
      if (cnt == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("Nessun rilievo");
      } else {
        for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String lbl = OledMenu::getSurveyLabel ? OledMenu::getSurveyLabel(i) : String(i);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *attiva <back");
      break;
    }

    // ---- Code category ----
    case OLED_SURVEY_CODE_CAT: {
      disp.setCursor(0, TITLE_Y);
      disp.print("CATEGORIA");
      dashedLine(SEP_Y);
      int catCount = OledMenu::getCodeCatCount ? OledMenu::getCodeCatCount() : 0;
      if (catCount == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("Nessuna categoria");
      } else {
        for (int i = s_scrollTop; i < catCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String lbl = OledMenu::getCodeCatLabel ? OledMenu::getCodeCatLabel(i) : String(i);
          if (lbl.length() > MAX_DISP_CHARS) lbl = lbl.substring(0, MAX_DISP_CHARS);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Code selection ----
    case OLED_SURVEY_CODE_CODE: {
      disp.setCursor(0, TITLE_Y);
      if (OledMenu::getCodeCatLabel) {
        String catLbl = OledMenu::getCodeCatLabel(s_selCatIdx);
        if (catLbl.length() > 10) catLbl = catLbl.substring(0, 10);
        disp.print(catLbl);
      } else {
        disp.print("CODICE");
      }
      dashedLine(SEP_Y);
      int codeCount = OledMenu::getCodeCount ? OledMenu::getCodeCount(s_selCatIdx) : 0;
      if (codeCount == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("Nessun codice");
      } else {
        for (int i = s_scrollTop; i < codeCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String cod = OledMenu::getCodeCod   ? OledMenu::getCodeCod(s_selCatIdx, i)   : "";
          String lbl = OledMenu::getCodeLabel ? OledMenu::getCodeLabel(s_selCatIdx, i) : "";
          String entry = cod + " - " + lbl;
          if (entry.length() > MAX_DISP_CHARS) entry = cod;
          if (entry.length() > MAX_DISP_CHARS) entry = entry.substring(0, MAX_DISP_CHARS);
          drawItem(y, entry.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Survey measure screen ----
    case OLED_SURVEY_MEASURE: {
      disp.setCursor(0, TITLE_Y);
      if (s_selCod.length() > 0) {
        char tbuf[MAX_DISP_CHARS + 1];
        snprintf(tbuf, sizeof(tbuf), "%-7s %s", "MISURA",
                 s_selCod.substring(0, 8).c_str());
        disp.print(tbuf);
      } else {
        disp.print("MISURA...");
      }
      dashedLine(SEP_Y);
      bool m = OledMenu::isMeasuring ? OledMenu::isMeasuring() : false;
      if (m) {
        String prog = OledMenu::getMeasureProgressStr ? OledMenu::getMeasureProgressStr() : "...";
        int lineIdx = 0, start = 0;
        while (start < (int)prog.length() && lineIdx < MAX_VISIBLE) {
          int end = prog.indexOf('\n', start);
          if (end < 0) end = prog.length();
          disp.setCursor(0, ITEMS_Y_START + lineIdx * ROW_H);
          disp.print(prog.substring(start, end));
          start = end + 1; lineIdx++;
        }
      } else {
        s_state = OLED_SURVEY_RESULT;
      }
      drawHint("*=ok");
      break;
    }

    // ---- Survey result ----
    case OLED_SURVEY_RESULT: {
      disp.setCursor(0, TITLE_Y);
      disp.print("RISULTATO");
      dashedLine(SEP_Y);
      String res = OledMenu::getMeasureResult ? OledMenu::getMeasureResult() : "---";
      int lineIdx = 0, start = 0;
      while (start < (int)res.length() && lineIdx < MAX_VISIBLE - 1) {
        int end = res.indexOf('\n', start);
        if (end < 0) end = res.length();
        disp.setCursor(0, ITEMS_Y_START + lineIdx * ROW_H);
        disp.print(res.substring(start, end));
        start = end + 1; lineIdx++;
      }
      if (s_selCod.length() > 0 && lineIdx < MAX_VISIBLE) {
        String codLine = s_selCod;
        if (codLine.length() > MAX_DISP_CHARS) codLine = codLine.substring(0, MAX_DISP_CHARS);
        disp.setCursor(0, ITEMS_Y_START + lineIdx * ROW_H);
        disp.print(codLine);
      }
      drawHint("*=ok <back");
      break;
    }

    // ---- Quality warning ----
    case OLED_SURVEY_QUALITY_WARN: {
      disp.setCursor(0, 10);
      disp.print("Qualita' bassa!");
      disp.setCursor(0, 22);
      disp.print("*=Forza  <=Annulla");
      break;
    }

    // ---- Survey MAP screen: north-up, rover always centered, N points ----
    case OLED_SURVEY_MAP: {
      bool roverOK = OledMenu::getSurveyMapRoverValid && OledMenu::getSurveyMapRoverValid();
      int  cnt     = OledMenu::getSurveyMapPointCount ? OledMenu::getSurveyMapPointCount() : 0;

      // "You are here" marker: fixed cross at screen center
      disp.drawFastHLine(MAP_CX - 3, MAP_CY, 7, SSD1306_WHITE);
      disp.drawFastVLine(MAP_CX, MAP_CY - 3, 7, SSD1306_WHITE);

      float scaleCm = MAP_SCALE_CM[s_surveyMapZoomIdx];

      if (roverOK) {
        for (int i = 0; i < cnt; i++) {
          OledMenu::OledSurveyMapPoint p;
          if (!OledMenu::getSurveyMapPoint || !OledMenu::getSurveyMapPoint(i, p)) continue;
          double distCm = p.distance * 100.0;
          double pxOff  = distCm / scaleCm;
          double azRad  = p.azimuth * DEG_TO_RAD;
          int tx = MAP_CX + (int)round(pxOff * sin(azRad));
          int ty = MAP_CY - (int)round(pxOff * cos(azRad)); // North = up
          const int margin = 3;  // room for the highlight circle (radius 3)
          if (tx < margin) tx = margin;
          if (tx > SCREEN_W - 1 - margin) tx = SCREEN_W - 1 - margin;
          if (ty < margin) ty = margin;
          if (ty > 55 - margin) ty = 55 - margin;
          if (i == s_surveyMapHighlightIdx) {
            disp.drawCircle(tx, ty, 3, SSD1306_WHITE);
          } else {
            disp.drawPixel(tx,     ty,     SSD1306_WHITE);
            disp.drawPixel(tx + 1, ty,     SSD1306_WHITE);
            disp.drawPixel(tx,     ty + 1, SSD1306_WHITE);
          }
        }
      }

      // Bottom info bar: current scale (left) + point count / fix state (right)
      char scaleBuf[12];
      if (scaleCm < 100) snprintf(scaleBuf, sizeof(scaleBuf), "%.0fcm/px", scaleCm);
      else               snprintf(scaleBuf, sizeof(scaleBuf), "%.0fm/px", scaleCm / 100.0f);
      disp.setCursor(0, HINT_Y);
      disp.print(scaleBuf);

      char rightBuf[12];
      if (!roverOK) snprintf(rightBuf, sizeof(rightBuf), "NO FIX");
      else          snprintf(rightBuf, sizeof(rightBuf), "%d pt", cnt);
      int16_t x1, y1; uint16_t w, h;
      disp.getTextBounds(rightBuf, 0, 0, &x1, &y1, &w, &h);
      disp.setCursor(SCREEN_W - (int)w, HINT_Y);
      disp.print(rightBuf);
      break;
    }

    // ---- Survey map menu (Fit all / Zoom to point / Back) ----
    case OLED_SURVEY_MAP_MENU: {
      disp.setCursor(0, TITLE_Y);
      disp.print("MAP MENU");
      dashedLine(SEP_Y);
      for (int i = 0; i < SMI_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        drawItem(y, SURVEY_MAP_MENU_ITEMS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Survey map: point picker for "Zoom to point" ----
    case OLED_SURVEY_MAP_POINT_LIST: {
      disp.setCursor(0, TITLE_Y);
      disp.print("ZOOM TO POINT");
      dashedLine(SEP_Y);
      int cnt = OledMenu::getSurveyMapPointCount ? OledMenu::getSurveyMapPointCount() : 0;
      if (cnt == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("No points");
      } else {
        for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          OledMenu::OledSurveyMapPoint p;
          String lbl = (OledMenu::getSurveyMapPoint && OledMenu::getSurveyMapPoint(i, p)) ? p.label : String(i);
          if (lbl.length() > MAX_DISP_CHARS) lbl = lbl.substring(0, MAX_DISP_CHARS);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Base mode root ----
    case OLED_MENU_BASE: {
      disp.setCursor(0, TITLE_Y);
      bool baseOn = getBool(OledMenu::getBaseActive);
      disp.print(baseOn ? "* BASE [ON]" : "* BASE");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < BASE_COUNT && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, BASE_ITEMS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Confirm: Start Base Mode ----
    case OLED_BASE_CONFIRM_START: {
      disp.setCursor(0, 10);
      disp.print("Start Base Mode?");
      disp.setCursor(0, 22);
      String sel = OledMenu::getSelectedBaseLabel ? OledMenu::getSelectedBaseLabel() : "";
      disp.print(sel.length() ? sel : "Nessuna base sel.!");
      disp.setCursor(0, 38);
      disp.print("*=Conferma  <=Annulla");
      break;
    }

    // ---- Wait screen: Start/Stop in progress (async, see BasePendingAction) ----
    case OLED_BASE_WAIT: {
      disp.setCursor(0, TITLE_Y);
      disp.print(s_baseActionWasStop ? "RITORNO ROVER" : "AVVIO BASE");
      dashedLine(SEP_Y);
      bool pending = OledMenu::isBaseActionPending ? OledMenu::isBaseActionPending() : false;
      if (pending) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("Attendere...");
      } else {
        s_state = OLED_BASE_RESULT;
      }
      drawHint("*=ok");
      break;
    }

    // ---- Base result ----
    case OLED_BASE_RESULT: {
      disp.setCursor(0, TITLE_Y);
      disp.print(s_baseActionWasStop ? "ROVER" : "BASE");
      dashedLine(SEP_Y);
      String res = OledMenu::getBaseActionResult ? OledMenu::getBaseActionResult() : "---";
      int lineIdx = 0, start = 0;
      while (start < (int)res.length() && lineIdx < MAX_VISIBLE) {
        int end = res.indexOf('\n', start);
        if (end < 0) end = res.length();
        disp.setCursor(0, ITEMS_Y_START + lineIdx * ROW_H);
        disp.print(res.substring(start, end));
        start = end + 1; lineIdx++;
      }
      drawHint("*=ok");
      break;
    }

    // ---- Confirm: Stop base → rover ----
    case OLED_BASE_CONFIRM_STOP: {
      disp.setCursor(0, 10);
      disp.print("Stop base?");
      disp.setCursor(0, 22);
      disp.print("ZED reset + stop out.");
      disp.setCursor(0, 38);
      disp.print("*=Conferma  <=Annulla");
      break;
    }

    // ---- Confirm: Survey-in auto ----
    case OLED_BASE_CONFIRM_SVIN: {
      disp.setCursor(0, 10);
      disp.print("Avvia Survey-in?");
      disp.setCursor(0, 22);
      disp.print("ZED inizia survey.");
      disp.setCursor(0, 38);
      disp.print("*=Conferma  <=Annulla");
      break;
    }

    // ---- Base list (select only) ----
    case OLED_BASE_LIST:
      drawSelectableList("SELEZIONA BASE",
                          OledMenu::getBaseListCount ? OledMenu::getBaseListCount() : 0,
                          OledMenu::getBaseListLabel,
                          OledMenu::getSelectedBaseIdx ? OledMenu::getSelectedBaseIdx() : -1);
      break;

    // ---- Base outputs sub-menu ----
    case OLED_BASE_OUTPUTS_MENU: {
      disp.setCursor(0, TITLE_Y);
      disp.print("USCITE RTCM");
      dashedLine(SEP_Y);
      for (int i = 0; i < BASE_OUT_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        String right = "";
        if (i == BOI_NTRIP) {
          int idx = OledMenu::getSelectedNtripOutIdx ? OledMenu::getSelectedNtripOutIdx() : -1;
          right = (idx >= 0 && OledMenu::getNtripOutListLabel) ? OledMenu::getNtripOutListLabel(idx) : String("---");
        } else if (i == BOI_TCPCLI) {
          int idx = OledMenu::getSelectedTcpCliIdx ? OledMenu::getSelectedTcpCliIdx() : -1;
          right = (idx >= 0 && OledMenu::getTcpCliListLabel) ? OledMenu::getTcpCliListLabel(idx) : String("---");
        } else if (i == BOI_AUTO_NTRIP) {
          right = getBool(OledMenu::getAutoNtripState) ? "ON" : "OFF";
        } else if (i == BOI_AUTO_TCP) {
          right = getBool(OledMenu::getAutoTcpCliState) ? "ON" : "OFF";
        }
        if (right.length() > 9) right = right.substring(0, 9);
        drawItem(y, BASE_OUT_ITEMS[i], right.length() ? right.c_str() : nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- NTRIP OUT profile list (select only) ----
    case OLED_BASE_NTRIPOUT_LIST:
      drawSelectableList("NTRIP OUT",
                          OledMenu::getNtripOutListCount ? OledMenu::getNtripOutListCount() : 0,
                          OledMenu::getNtripOutListLabel,
                          OledMenu::getSelectedNtripOutIdx ? OledMenu::getSelectedNtripOutIdx() : -1);
      break;

    // ---- TCP Client OUT profile list (select only) ----
    case OLED_BASE_TCPCLI_LIST:
      drawSelectableList("TCP CLIENT OUT",
                          OledMenu::getTcpCliListCount ? OledMenu::getTcpCliListCount() : 0,
                          OledMenu::getTcpCliListLabel,
                          OledMenu::getSelectedTcpCliIdx ? OledMenu::getSelectedTcpCliIdx() : -1);
      break;

    // ---- Stakeout sub-menu ----
    case OLED_STAKEOUT_MENU: {
      disp.setCursor(0, TITLE_Y);
      disp.print("* STAKEOUT");
      dashedLine(SEP_Y);
      for (int i = 0; i < STAKEOUT_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        drawItem(y, STAKEOUT_ITEMS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Stakeout file list ----
    case OLED_STAKEOUT_FILE_LIST: {
      disp.setCursor(0, TITLE_Y);
      disp.print("FILE STAKEOUT");
      dashedLine(SEP_Y);
      int cnt = OledMenu::getStakeoutFileCount ? OledMenu::getStakeoutFileCount() : 0;
      if (cnt == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("Nessun file");
      } else {
        for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String lbl = OledMenu::getStakeoutFileLabel ? OledMenu::getStakeoutFileLabel(i) : String(i);
          if (lbl.length() > MAX_DISP_CHARS) lbl = lbl.substring(0, MAX_DISP_CHARS);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *apri <back");
      break;
    }

    // ---- Stakeout point list ----
    case OLED_STAKEOUT_POINT_LIST: {
      disp.setCursor(0, TITLE_Y);
      disp.print("PUNTI");
      dashedLine(SEP_Y);
      int cnt = OledMenu::getStakeoutPointCount ? OledMenu::getStakeoutPointCount() : 0;
      if (cnt == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("Nessun punto");
      } else {
        for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String lbl = OledMenu::getStakeoutPointLabel ? OledMenu::getStakeoutPointLabel(i) : String(i);
          if (lbl.length() > MAX_DISP_CHARS) lbl = lbl.substring(0, MAX_DISP_CHARS);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *imposta <back");
      break;
    }

    // ---- Stakeout NAV screen ----
    case OLED_STAKEOUT_NAV: {
      disp.setCursor(0, TITLE_Y);
      disp.print("STAKEOUT");
      dashedLine(SEP_Y);
      if (OledMenu::getStakeoutNavString) {
        String nav = OledMenu::getStakeoutNavString();
        int lineIdx = 0, start = 0;
        while (start < (int)nav.length() && lineIdx < MAX_VISIBLE) {
          int end = nav.indexOf('\n', start);
          if (end < 0) end = nav.length();
          disp.setCursor(0, ITEMS_Y_START + lineIdx * ROW_H);
          disp.print(nav.substring(start, end));
          start = end + 1; lineIdx++;
        }
      } else {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("Nessun target");
      }
      drawHint("*=map <back");
      break;
    }

    // ---- Stakeout MAP screen: north-up, rover always centered ----------
    case OLED_STAKEOUT_MAP: {
      OledMenu::OledStakeoutNavData nav;
      if (OledMenu::getStakeoutNavData) OledMenu::getStakeoutNavData(nav);

      // "You are here" marker: fixed cross at screen center
      disp.drawFastHLine(MAP_CX - 3, MAP_CY, 7, SSD1306_WHITE);
      disp.drawFastVLine(MAP_CX, MAP_CY - 3, 7, SSD1306_WHITE);

      float scaleCm = MAP_SCALE_CM[s_stakeoutMapZoomIdx];

      if (nav.valid) {
        double distCm  = nav.distance * 100.0;
        double pxOff   = distCm / scaleCm;
        double azRad   = nav.azimuth * DEG_TO_RAD;
        int tx = MAP_CX + (int)round(pxOff * sin(azRad));
        int ty = MAP_CY - (int)round(pxOff * cos(azRad)); // North = up
        // Clamp to the map canvas (y 0..55) so the target stays visible
        // (with a direction cue) even if it doesn't fit at the current zoom.
        const int margin = 3;
        if (tx < margin) tx = margin;
        if (tx > SCREEN_W - 1 - margin) tx = SCREEN_W - 1 - margin;
        if (ty < margin) ty = margin;
        if (ty > 55 - margin) ty = 55 - margin;
        disp.drawCircle(tx, ty, 2, SSD1306_WHITE);
      }

      // Bottom info bar: current scale (left) + live distance (right)
      char scaleBuf[12];
      if (scaleCm < 100) snprintf(scaleBuf, sizeof(scaleBuf), "%.0fcm/px", scaleCm);
      else               snprintf(scaleBuf, sizeof(scaleBuf), "%.0fm/px", scaleCm / 100.0f);
      disp.setCursor(0, HINT_Y);
      disp.print(scaleBuf);
      if (nav.valid) {
        char distBuf[16];
        snprintf(distBuf, sizeof(distBuf), "%.2fm", nav.distance);
        int16_t x1, y1; uint16_t w, h;
        disp.getTextBounds(distBuf, 0, 0, &x1, &y1, &w, &h);
        disp.setCursor(SCREEN_W - w, HINT_Y);
        disp.print(distBuf);
      } else {
        disp.setCursor(SCREEN_W - 42, HINT_Y);
        disp.print("no target");
      }
      break;
    }

    // ---- Tracking menu ----
    case OLED_TRACK_MENU: {
      disp.setCursor(0, TITLE_Y);
      disp.print("* TRACKING");
      dashedLine(SEP_Y);
      bool rec = OledMenu::getTrackRecording && OledMenu::getTrackRecording();
      int y0 = ITEMS_Y_START;
      if (rec && OledMenu::getTrackStatusString) {
        String st = OledMenu::getTrackStatusString();
        int nl = st.indexOf('\n');
        String line1 = nl > 0 ? st.substring(0, nl) : st;
        disp.setCursor(0, y0);
        disp.print(line1);
        y0 += ROW_H;
      }
      for (int i = 0; i < TRACK_MENU_COUNT; i++) {
        int y = y0 + i * ROW_H;
        char right[6] = "";
        if (i == 0) strncpy(right, rec ? "REC" : "OFF", sizeof(right) - 1);
        drawItem(y, TRACK_MENU_ITEMS[i], right, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Tracking settings ----
    case OLED_TRACK_SETTINGS: {
      disp.setCursor(0, TITLE_Y);
      disp.print("TRACK SETTINGS");
      dashedLine(SEP_Y);
      for (int i = 0; i < TRACK_SETTINGS_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        char right[10] = "";
        if (i == 0) {
          int mode = OledMenu::getTrackTriggerMode ? OledMenu::getTrackTriggerMode() : 0;
          strncpy(right, TRACK_MODE_LABELS[mode], sizeof(right) - 1);
        }
        drawItem(y, TRACK_SETTINGS_ITEMS[i], right, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Tracking trigger mode picker ----
    case OLED_TRACK_TRIGGER_MODE: {
      disp.setCursor(0, TITLE_Y);
      disp.print("TRIGGER MODE");
      dashedLine(SEP_Y);
      for (int i = 0; i < TRACK_MODE_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        drawItem(y, TRACK_MODE_LABELS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *set <back");
      break;
    }

    // ---- Tracking threshold picker (seconds or metres, depending on mode) ----
    case OLED_TRACK_THRESHOLD: {
      disp.setCursor(0, TITLE_Y);
      int mode = OledMenu::getTrackTriggerMode ? OledMenu::getTrackTriggerMode() : 0;
      disp.print(mode == 0 ? "INTERVAL" : "DISTANCE");
      dashedLine(SEP_Y);
      const char** labels = (mode == 0) ? TRACK_TIME_LABELS : TRACK_DIST_LABELS;
      for (int i = s_scrollTop; i < TRACK_THRESHOLD_COUNT && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, labels[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *set <back");
      break;
    }

    default:
      break;
  }

  disp.display();
}

const String& getSelectedCod()      { return s_selCod;    }
const String& getSelectedCodLabel() { return s_selCodLbl; }

} // namespace OledMenu
