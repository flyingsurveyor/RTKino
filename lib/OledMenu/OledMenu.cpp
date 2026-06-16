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
  void (*onToggleNtripOut) (bool) = nullptr;
  void (*onToggleTcpOutSrv)(bool) = nullptr;
  void (*onToggleTcpOutCli)(bool) = nullptr;
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
  bool        (*getNtripOutState)     () = nullptr;
  bool        (*getTcpOutSrvState)    () = nullptr;
  bool        (*getTcpOutCliState)    () = nullptr;
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
  void   (*onStartBaseFromPoint)(int)  = nullptr;
  void   (*onStartBaseFromList) (int)  = nullptr;
  void   (*onStartSurveyIn)     ()     = nullptr;
  void   (*onStopBase)          ()     = nullptr;
  bool   (*getBaseActive)       ()     = nullptr;
  int    (*getBaseListCount)    ()     = nullptr;
  String (*getBaseListLabel)    (int)  = nullptr;

  // Stakeout
  void   (*onSelectStakeoutFile) (int)  = nullptr;
  void   (*onSetStakeoutActive)  (int)  = nullptr;
  int    (*getStakeoutFileCount) ()     = nullptr;
  String (*getStakeoutFileLabel) (int)  = nullptr;
  int    (*getStakeoutPointCount)()     = nullptr;
  String (*getStakeoutPointLabel)(int)  = nullptr;
  String (*getStakeoutNavString) ()     = nullptr;
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
// Main menu  (6 items)
// ---------------------------------------------------------------------------
// Index 0 = Misura punto (direct shortcut, skips Survey sub-menu)
// Index 1 = Surveys
// Index 2 = Stakeout
// Index 3 = Base
// Index 4 = Settings
// Index 5 = Back
static const char* MAIN_ITEMS[] = {
  "Misura punto", "Surveys", "Stakeout", "Base", "Settings", "Back"
};
static const int MAIN_COUNT = 6;

// ---------------------------------------------------------------------------
// Settings root (4 items)
// ---------------------------------------------------------------------------
static const char* SETTINGS_ROOT[] = { "Rete", "Display", "Sistema", "Back" };
static const int   SETTINGS_ROOT_COUNT = 4;

// ---------------------------------------------------------------------------
// Network sub-menu (7 items — all toggles except Back)
// ---------------------------------------------------------------------------
enum NetItem {
  NI_BLE = 0, NI_NTRIP_IN, NI_TCP_IN,
  NI_NTRIP_OUT, NI_TCP_OUT_SRV, NI_TCP_OUT_CLI, NI_BACK,
  NI_COUNT
};
static const char* NET_LABELS[NI_COUNT] = {
  "BLE", "NTRIP IN", "TCP IN",
  "NTRIP OUT", "TCP OUT Srv", "TCP OUT Cli", "Back"
};

// ---------------------------------------------------------------------------
// Display sub-menu (2 items)
// ---------------------------------------------------------------------------
static const char* DISP_LABELS[] = { "Brightness", "Back" };
static const int   DISP_COUNT    = 2;

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
// Brightness sub-menu
// ---------------------------------------------------------------------------
static const char*   BRIGHT_LABELS[] = { "Normal", "Dim", "Off" };
static const int     BRIGHT_COUNT    = 3;
static const uint8_t BRIGHTNESS_NORMAL = 0xCF;
static const uint8_t BRIGHTNESS_OFF    = 0x00;

// ---------------------------------------------------------------------------
// Survey sub-menu (4 items)
// ---------------------------------------------------------------------------
static const char* SURVEY_ITEMS[] = { "Misura punto", "Rilievi", "Nuovo rilievo", "Back" };
static const int   SURVEY_COUNT   = 4;

// ---------------------------------------------------------------------------
// Base mode menu (5 items)
// ---------------------------------------------------------------------------
static const char* BASE_ITEMS[] = {
  "Da punto misurato",
  "Da lista basi",
  "Survey-in auto",
  "Stop base->rover",
  "Back"
};
static const int BASE_COUNT = 5;

// ---------------------------------------------------------------------------
// Base measurement duration options
// ---------------------------------------------------------------------------
static const char* BASE_DUR_LABELS[] = { "30 sec", "60 sec", "2 min", "5 min" };
static const int   BASE_DUR_VALUES[] = { 30,       60,       120,     300      };
static const int   BASE_DUR_COUNT    = 4;

// ---------------------------------------------------------------------------
// Stakeout sub-menu
// ---------------------------------------------------------------------------
static const char* STAKEOUT_ITEMS[] = { "Naviga", "File", "Back" };
static const int   STAKEOUT_COUNT   = 3;

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

// Base mode flag: distinguishes base measurement from survey measurement
static bool   s_isMeasuringBase    = false;
static int    s_pendingBaseDurSec  = 30;   // duration chosen in OLED_BASE_MEASURE_DUR

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
    case NI_BLE:         return getBool(OledMenu::getBleState);
    case NI_NTRIP_IN:    return getBool(OledMenu::getNtripInState);
    case NI_TCP_IN:      return getBool(OledMenu::getTcpInState);
    case NI_NTRIP_OUT:   return getBool(OledMenu::getNtripOutState);
    case NI_TCP_OUT_SRV: return getBool(OledMenu::getTcpOutSrvState);
    case NI_TCP_OUT_CLI: return getBool(OledMenu::getTcpOutCliState);
    default:             return false;
  }
}

static void toggleNetItem(int idx) {
  bool next = !netItemState(idx);
  switch (idx) {
    case NI_BLE:         callBool(OledMenu::onToggleBLE,       next); break;
    case NI_NTRIP_IN:    callBool(OledMenu::onToggleNtripIn,   next); break;
    case NI_TCP_IN:      callBool(OledMenu::onToggleTcpIn,     next); break;
    case NI_NTRIP_OUT:   callBool(OledMenu::onToggleNtripOut,  next); break;
    case NI_TCP_OUT_SRV: callBool(OledMenu::onToggleTcpOutSrv, next); break;
    case NI_TCP_OUT_CLI: callBool(OledMenu::onToggleTcpOutCli, next); break;
    default: break;
  }
}

// ---------------------------------------------------------------------------
// Launch a measurement for survey (normal path)
// ---------------------------------------------------------------------------
static void launchSurveyMeasure(bool force) {
  s_isMeasuringBase = false;
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
  s_isMeasuringBase = false;
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
      s_isMeasuringBase = false;
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
      // Settings
      case OLED_MENU_SETTINGS:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 4);
        break;
      case OLED_MENU_SETTINGS_NET:
      case OLED_MENU_SETTINGS_DISP:
      case OLED_MENU_SETTINGS_SYS:
        enterMenu(OLED_MENU_SETTINGS, SETTINGS_ROOT_COUNT);
        break;
      case OLED_SUBMENU_RATE:
        enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_RATE);
        break;
      case OLED_SUBMENU_BRIGHT:
        enterMenu(OLED_MENU_SETTINGS_DISP, DISP_COUNT);
        break;
      case OLED_INFO_SCREEN:
        enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_INFO);
        break;
      case OLED_CONFIRM_REBOOT:
        enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_REBOOT);
        break;
      // Surveys
      case OLED_SURVEY_MENU:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 1);
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
      // Base mode
      case OLED_MENU_BASE:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 3);
        break;
      case OLED_BASE_MEASURE_DUR:
        enterMenu(OLED_MENU_BASE, BASE_COUNT);
        break;
      case OLED_BASE_MEASURING:
        enterMenu(OLED_MENU_BASE, BASE_COUNT);
        break;
      case OLED_BASE_LIST:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, 1);
        break;
      case OLED_BASE_RESULT:
        enterMenu(OLED_MENU_BASE, BASE_COUNT);
        break;
      case OLED_BASE_CONFIRM_STOP:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, 3);
        break;
      case OLED_BASE_CONFIRM_SVIN:
        enterMenu(OLED_MENU_BASE, BASE_COUNT, 2);
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
        enterMenu(OLED_STAKEOUT_MENU, STAKEOUT_COUNT);
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
    switch (s_state) {
      case OLED_MENU_MAIN:
      case OLED_MENU_SETTINGS:
      case OLED_MENU_SETTINGS_NET:
      case OLED_MENU_SETTINGS_DISP:
      case OLED_MENU_SETTINGS_SYS:
      case OLED_SUBMENU_RATE:
      case OLED_SUBMENU_BRIGHT:
      case OLED_SURVEY_MENU:
      case OLED_SURVEY_LIST:
      case OLED_SURVEY_CODE_CAT:
      case OLED_SURVEY_CODE_CODE:
      case OLED_MENU_BASE:
      case OLED_BASE_MEASURE_DUR:
      case OLED_BASE_LIST:
      case OLED_STAKEOUT_MENU:
      case OLED_STAKEOUT_FILE_LIST:
      case OLED_STAKEOUT_POINT_LIST:
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
        case 0: { // Misura punto — direct shortcut: go to code category selection
          int catCount = OledMenu::getCodeCatCount ? OledMenu::getCodeCatCount() : 0;
          s_selCod = ""; s_selCodLbl = "";
          s_isMeasuringBase = false;
          if (catCount > 0) {
            enterMenu(OLED_SURVEY_CODE_CAT, catCount, s_selCatIdx);
          } else {
            bool ok = (!OledMenu::getQualityOK || OledMenu::getQualityOK());
            if (!ok) s_state = OLED_SURVEY_QUALITY_WARN;
            else launchSurveyMeasure(false);
          }
          break;
        }
        case 1: // Surveys
          enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
          break;
        case 2: // Stakeout
          enterMenu(OLED_STAKEOUT_MENU, STAKEOUT_COUNT);
          break;
        case 3: // Base
          enterMenu(OLED_MENU_BASE, BASE_COUNT);
          break;
        case 4: // Settings
          enterMenu(OLED_MENU_SETTINGS, SETTINGS_ROOT_COUNT);
          break;
        case 5: // Back
          s_state = OLED_NORMAL;
          break;
      }
      break;

    // ---- Settings root ----
    case OLED_MENU_SETTINGS:
      switch (s_cursor) {
        case 0: enterMenu(OLED_MENU_SETTINGS_NET,  NI_COUNT);   break;
        case 1: enterMenu(OLED_MENU_SETTINGS_DISP, DISP_COUNT); break;
        case 2: enterMenu(OLED_MENU_SETTINGS_SYS,  SYI_COUNT);  break;
        case 3: enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 4);       break;
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

    // ---- Display sub-menu ----
    case OLED_MENU_SETTINGS_DISP:
      switch (s_cursor) {
        case 0: enterMenu(OLED_SUBMENU_BRIGHT, BRIGHT_COUNT); break;
        case 1: enterMenu(OLED_MENU_SETTINGS, SETTINGS_ROOT_COUNT, 1); break;
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
          enterMenu(OLED_MENU_SETTINGS, SETTINGS_ROOT_COUNT, 2);
          break;
      }
      break;

    // ---- ZED Rate sub-menu ----
    case OLED_SUBMENU_RATE:
      if (OledMenu::onSetRate) OledMenu::onSetRate(RATE_MS[s_cursor]);
      enterMenu(OLED_MENU_SETTINGS_SYS, SYI_COUNT, SYI_RATE);
      break;

    // ---- Brightness sub-menu ----
    case OLED_SUBMENU_BRIGHT:
      enterMenu(OLED_MENU_SETTINGS_DISP, DISP_COUNT);
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
          s_isMeasuringBase = false;
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
          enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 1);
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
      s_isMeasuringBase = false;
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
      if (s_isMeasuringBase) {
        // Force-start base measurement anyway
        s_isMeasuringBase = true;
        if (OledMenu::onStartBaseFromPoint) OledMenu::onStartBaseFromPoint(s_pendingBaseDurSec);
        s_state = OLED_BASE_MEASURING;
      } else {
        launchSurveyMeasure(true);
      }
      break;

    // ---- Base mode root ----
    case OLED_MENU_BASE:
      switch (s_cursor) {
        case 0: // Da punto misurato → duration selection
          enterMenu(OLED_BASE_MEASURE_DUR, BASE_DUR_COUNT);
          break;
        case 1: { // Da lista basi
          int cnt = OledMenu::getBaseListCount ? OledMenu::getBaseListCount() : 0;
          enterMenu(OLED_BASE_LIST, max(cnt, 1));
          break;
        }
        case 2: // Survey-in auto → ask confirmation first
          s_state = OLED_BASE_CONFIRM_SVIN;
          break;
        case 3: // Stop base → rover → ask confirmation first
          s_state = OLED_BASE_CONFIRM_STOP;
          break;
        case 4: // Back
          enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 3);
          break;
      }
      break;

    // ---- Base duration selection ----
    case OLED_BASE_MEASURE_DUR: {
      s_pendingBaseDurSec = BASE_DUR_VALUES[s_cursor];
      s_isMeasuringBase   = true;
      // Quality gate check before starting
      bool ok = (!OledMenu::getQualityOK || OledMenu::getQualityOK());
      if (!ok) {
        s_state = OLED_SURVEY_QUALITY_WARN;
      } else {
        if (OledMenu::onStartBaseFromPoint) OledMenu::onStartBaseFromPoint(s_pendingBaseDurSec);
        s_state = OLED_BASE_MEASURING;
      }
      break;
    }

    // ---- Base measuring screen ----
    case OLED_BASE_MEASURING: {
      bool m = OledMenu::isMeasuring ? OledMenu::isMeasuring() : false;
      if (!m) s_state = OLED_BASE_RESULT;
      break;
    }

    // ---- Base result screen ----
    case OLED_BASE_RESULT:
      enterMenu(OLED_MENU_BASE, BASE_COUNT);
      break;

    // ---- Confirm: Stop base → rover ----
    case OLED_BASE_CONFIRM_STOP:
      if (OledMenu::onStopBase) OledMenu::onStopBase();
      enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 3);
      break;

    // ---- Confirm: Survey-in auto ----
    case OLED_BASE_CONFIRM_SVIN:
      if (OledMenu::onStartSurveyIn) OledMenu::onStartSurveyIn();
      enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 3);
      break;

    // ---- Base list ----
    case OLED_BASE_LIST:
      if (OledMenu::onStartBaseFromList) OledMenu::onStartBaseFromList(s_cursor);
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

    // ---- Stakeout nav screen ----
    case OLED_STAKEOUT_NAV:
      enterMenu(OLED_STAKEOUT_MENU, STAKEOUT_COUNT);
      break;

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
      disp.print("RETE");
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

    // ---- Display sub-menu ----
    case OLED_MENU_SETTINGS_DISP: {
      disp.setCursor(0, TITLE_Y);
      disp.print("DISPLAY");
      dashedLine(SEP_Y);
      for (int i = 0; i < DISP_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        drawItem(y, DISP_LABELS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
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

    // ---- Brightness sub-menu ----
    case OLED_SUBMENU_BRIGHT: {
      disp.setCursor(0, TITLE_Y);
      disp.print("LUMINOSITA'");
      dashedLine(SEP_Y);
      for (int i = 0; i < BRIGHT_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        drawItem(y, BRIGHT_LABELS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *set <back");
      // Apply brightness live as user scrolls
      if (s_cursor == 0) {
        disp.ssd1306_command(SSD1306_SETCONTRAST);
        disp.ssd1306_command(BRIGHTNESS_NORMAL);
        disp.dim(false);
      } else if (s_cursor == 1) {
        disp.dim(true);
      } else {
        disp.dim(true);
        disp.ssd1306_command(SSD1306_SETCONTRAST);
        disp.ssd1306_command(BRIGHTNESS_OFF);
      }
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

    // ---- Base duration selection ----
    case OLED_BASE_MEASURE_DUR: {
      disp.setCursor(0, TITLE_Y);
      disp.print("DURATA MISURA");
      dashedLine(SEP_Y);
      for (int i = 0; i < BASE_DUR_COUNT; i++) {
        int y = ITEMS_Y_START + i * ROW_H;
        drawItem(y, BASE_DUR_LABELS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Base measuring screen ----
    case OLED_BASE_MEASURING: {
      disp.setCursor(0, TITLE_Y);
      disp.print("MISURA BASE");
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
        s_state = OLED_BASE_RESULT;
      }
      drawHint("*=ok");
      break;
    }

    // ---- Base result ----
    case OLED_BASE_RESULT: {
      disp.setCursor(0, TITLE_Y);
      disp.print("BASE IMPOSTATA");
      dashedLine(SEP_Y);
      String res = OledMenu::getMeasureResult ? OledMenu::getMeasureResult() : "---";
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
      disp.print("ZED torna in rover.");
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

    // ---- Base list ----
    case OLED_BASE_LIST: {
      disp.setCursor(0, TITLE_Y);
      disp.print("BASI SALVATE");
      dashedLine(SEP_Y);
      int cnt = OledMenu::getBaseListCount ? OledMenu::getBaseListCount() : 0;
      if (cnt == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("Nessuna base");
      } else {
        for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String lbl = OledMenu::getBaseListLabel ? OledMenu::getBaseListLabel(i) : String(i);
          if (lbl.length() > MAX_DISP_CHARS) lbl = lbl.substring(0, MAX_DISP_CHARS);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *attiva <back");
      break;
    }

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
      drawHint("*=menu");
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
