// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "OledMenu.h"

// ---------------------------------------------------------------------------
// Callback / state-reader function pointers — default to nullptr
// ---------------------------------------------------------------------------
namespace OledMenu {
  void (*onToggleBLE)      (bool) = nullptr;
  void (*onToggleNtripIn)  (bool) = nullptr;
  void (*onToggleTcpIn)    (bool) = nullptr;
  void (*onToggleNtripOut) (bool) = nullptr;
  void (*onToggleTcpOutSrv)(bool) = nullptr;
  void (*onToggleTcpOutCli)(bool) = nullptr;
  void (*onToggleBuzzer)   (bool) = nullptr;
  void (*onSetRate)        (uint16_t) = nullptr;
  void (*onReboot)         ()     = nullptr;
  void (*onSyncToSD)       ()     = nullptr;
  void (*onToggleRawLog)   (bool) = nullptr;

  // Survey action callbacks
  void (*onMeasurePoint)      ()    = nullptr;
  void (*onForceMeasurePoint) ()    = nullptr;
  void (*onCreateSurvey)      ()    = nullptr;
  void (*onSetActiveSurvey)   (int) = nullptr;

  // Point-code callbacks
  int    (*getCodeCatCount) ()                     = nullptr;
  String (*getCodeCatLabel) (int)                  = nullptr;
  int    (*getCodeCount)    (int)                  = nullptr;
  String (*getCodeCod)      (int, int)             = nullptr;
  String (*getCodeLabel)    (int, int)             = nullptr;
  void   (*onMeasureWithCode)(const String&, const String&, bool) = nullptr;

  bool        (*getBleState)      () = nullptr;
  bool        (*getNtripInState)  () = nullptr;
  bool        (*getTcpInState)    () = nullptr;
  bool        (*getNtripOutState) () = nullptr;
  bool        (*getTcpOutSrvState)() = nullptr;
  bool        (*getTcpOutCliState)() = nullptr;
  bool        (*getBuzzerState)   () = nullptr;
  const char* (*getRateString)    () = nullptr;
  String      (*getInfoString)    () = nullptr;
  bool        (*getRawLogState)   () = nullptr;

  // Survey state readers
  bool   (*isMeasuring)          () = nullptr;
  bool   (*getQualityOK)         () = nullptr;
  String (*getMeasureProgressStr)() = nullptr;
  String (*getMeasureResult)     () = nullptr;
  int    (*getSurveyCount)       () = nullptr;
  String (*getSurveyLabel)       (int) = nullptr;

  // Stakeout action callbacks
  void   (*onSelectStakeoutFile) (int)  = nullptr;
  void   (*onSetStakeoutActive)  (int)  = nullptr;

  // Stakeout state readers
  int    (*getStakeoutFileCount) ()    = nullptr;
  String (*getStakeoutFileLabel) (int) = nullptr;
  int    (*getStakeoutPointCount)()    = nullptr;
  String (*getStakeoutPointLabel)(int) = nullptr;
  String (*getStakeoutNavString) ()    = nullptr;
}

// ---------------------------------------------------------------------------
// Display geometry (font 6×8, 128×64 screen)
// ---------------------------------------------------------------------------
static const int SCREEN_W       = 128;
static const int ROW_H          = 9;    // pixel height per text row
static const int TITLE_Y        = 0;
static const int SEP_Y          = 8;
static const int ITEMS_Y_START  = 11;
static const int HINT_Y         = 56;
static const int MAX_VISIBLE    = 5;    // rows available for list items
static const int MAX_DISP_CHARS = 21;  // max chars per row at font size 1

// ---------------------------------------------------------------------------
// Main menu items  (index 0 = Surveys, 1 = Stakeout, 2 = Settings, ...)
// ---------------------------------------------------------------------------
static const char* MAIN_ITEMS[] = { "Surveys", "Stakeout", "Settings", "Start Raw Log", "Back" };
static const int   MAIN_COUNT   = 5;

// ---------------------------------------------------------------------------
// Settings items
// ---------------------------------------------------------------------------
enum SettingsItem {
  SI_BLE = 0,
  SI_NTRIP_IN,
  SI_TCP_IN,
  SI_NTRIP_OUT,
  SI_TCP_OUT_SRV,
  SI_TCP_OUT_CLI,
  SI_BUZZER,
  SI_ZED_RATE,
  SI_BRIGHTNESS,
  SI_REBOOT,
  SI_SYNC_SD,
  SI_INFO,
  SI_BACK,
  SI_COUNT
};

static const char* SETTINGS_LABELS[SI_COUNT] = {
  "BLE",
  "NTRIP IN",
  "TCP IN",
  "NTRIP OUT",
  "TCP OUT Srv",
  "TCP OUT Cli",
  "Buzzer",
  "ZED Rate",
  "Brightness",
  "Reboot",
  "Sync SD",
  "Info",
  "Back"
};

// Which settings items are toggles (vs. sub-menu/action)
static const bool SETTINGS_IS_TOGGLE[SI_COUNT] = {
  true,  // BLE
  true,  // NTRIP IN
  true,  // TCP IN
  true,  // NTRIP OUT
  true,  // TCP OUT Srv
  true,  // TCP OUT CLI
  true,  // Buzzer
  false, // ZED Rate → sub-menu
  false, // Brightness → sub-menu
  false, // Reboot → action
  false, // Sync SD → action
  false, // Info → screen
  false  // Back → action
};

// ---------------------------------------------------------------------------
// ZED Rate sub-menu
// ---------------------------------------------------------------------------
static const char*    RATE_LABELS[] = { "1 Hz", "2 Hz", "5 Hz", "10 Hz", "15 Hz" };
static const uint16_t RATE_MS[]     = { 1000,   500,    200,    100,     67 };
static const int      RATE_COUNT    = 5;

// ---------------------------------------------------------------------------
// Brightness sub-menu
// ---------------------------------------------------------------------------
static const char*   BRIGHT_LABELS[] = { "Normal", "Dim", "Off" };
static const int     BRIGHT_COUNT    = 3;
static const uint8_t BRIGHTNESS_NORMAL = 0xCF;  // SSD1306 contrast for normal brightness
static const uint8_t BRIGHTNESS_OFF    = 0x00;  // SSD1306 contrast for display off

// ---------------------------------------------------------------------------
// Survey sub-menu items
// ---------------------------------------------------------------------------
static const char* SURVEY_ITEMS[] = {
  "Measure point",
  "Surveys",
  "New survey",
  "Back"
};
static const int SURVEY_COUNT = 4;

// ---------------------------------------------------------------------------
// Stakeout sub-menu items
// ---------------------------------------------------------------------------
static const char* STAKEOUT_ITEMS[] = { "Navigate", "Files", "Back" };
static const int   STAKEOUT_COUNT   = 3;

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------
static OledMenuState s_state       = OLED_NORMAL;
static int           s_cursor      = 0;   // highlighted item index
static int           s_scrollTop   = 0;   // first visible item index
static int           s_listCount   = 0;   // total items in current list

// ---------------------------------------------------------------------------
// Code-selection state
// ---------------------------------------------------------------------------
static int    s_selCatIdx  = 0;    // selected category index
static String s_selCod     = "";   // selected cod string (e.g. "SPIG_FAB")
static String s_selCodLbl  = "";   // selected label for display

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static void enterMenu(OledMenuState newState, int itemCount, int startCursor = 0) {
  s_state     = newState;
  s_listCount = itemCount;
  s_cursor    = startCursor;
  s_scrollTop = 0;
  // Adjust scroll so cursor is visible
  if (s_cursor >= MAX_VISIBLE) {
    s_scrollTop = s_cursor - MAX_VISIBLE + 1;
  }
}

static void scrollCursor(int delta) {
  s_cursor += delta;
  if (s_cursor < 0)           s_cursor = s_listCount - 1;
  if (s_cursor >= s_listCount) s_cursor = 0;

  // Keep cursor within visible window
  if (s_cursor < s_scrollTop) {
    s_scrollTop = s_cursor;
  } else if (s_cursor >= s_scrollTop + MAX_VISIBLE) {
    s_scrollTop = s_cursor - MAX_VISIBLE + 1;
  }
}

// Safe callback helpers
static bool getBool(bool (*fn)()) { return fn ? fn() : false; }
static void callBool(void (*fn)(bool), bool val) { if (fn) fn(val); }

// ---------------------------------------------------------------------------
// Settings item state reader
// ---------------------------------------------------------------------------
static bool settingsItemState(int idx) {
  switch (idx) {
    case SI_BLE:         return getBool(OledMenu::getBleState);
    case SI_NTRIP_IN:    return getBool(OledMenu::getNtripInState);
    case SI_TCP_IN:      return getBool(OledMenu::getTcpInState);
    case SI_NTRIP_OUT:   return getBool(OledMenu::getNtripOutState);
    case SI_TCP_OUT_SRV: return getBool(OledMenu::getTcpOutSrvState);
    case SI_TCP_OUT_CLI: return getBool(OledMenu::getTcpOutCliState);
    case SI_BUZZER:      return getBool(OledMenu::getBuzzerState);
    default:             return false;
  }
}

// ---------------------------------------------------------------------------
// Toggle a settings item
// ---------------------------------------------------------------------------
static void toggleSettingsItem(int idx) {
  bool cur = settingsItemState(idx);
  bool next = !cur;
  switch (idx) {
    case SI_BLE:         callBool(OledMenu::onToggleBLE,       next); break;
    case SI_NTRIP_IN:    callBool(OledMenu::onToggleNtripIn,   next); break;
    case SI_TCP_IN:      callBool(OledMenu::onToggleTcpIn,     next); break;
    case SI_NTRIP_OUT:   callBool(OledMenu::onToggleNtripOut,  next); break;
    case SI_TCP_OUT_SRV: callBool(OledMenu::onToggleTcpOutSrv, next); break;
    case SI_TCP_OUT_CLI: callBool(OledMenu::onToggleTcpOutCli, next); break;
    case SI_BUZZER:      callBool(OledMenu::onToggleBuzzer,    next); break;
    default: break;
  }
}

// ---------------------------------------------------------------------------
// OledMenu::init
// ---------------------------------------------------------------------------
namespace OledMenu {

void init() {
  s_state     = OLED_NORMAL;
  s_cursor    = 0;
  s_scrollTop = 0;
  s_listCount = 0;
}

// ---------------------------------------------------------------------------
// OledMenu::isActive / getState
// ---------------------------------------------------------------------------
bool isActive() { return s_state != OLED_NORMAL && s_state != OLED_PAGE2 && s_state != OLED_PAGE3; }
OledMenuState getState() { return s_state; }

// ---------------------------------------------------------------------------
// OledMenu::handleInput
// ---------------------------------------------------------------------------
void handleInput(int direction, bool click, bool doubleClick, bool longPress) {

  // --- Long press: navigate up or toggle menu ---
  if (longPress) {
    switch (s_state) {
      case OLED_NORMAL:
      case OLED_PAGE2:
      case OLED_PAGE3:     // long press from any home page enters menu; on exit returns to NORMAL
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT);
        break;
      case OLED_MENU_MAIN:
        s_state = OLED_NORMAL;
        break;
      case OLED_MENU_SETTINGS:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT);
        break;
      case OLED_SUBMENU_RATE:
      case OLED_SUBMENU_BRIGHT:
        enterMenu(OLED_MENU_SETTINGS, SI_COUNT);
        break;
      case OLED_INFO_SCREEN:
        enterMenu(OLED_MENU_SETTINGS, SI_COUNT, SI_INFO);
        break;
      case OLED_CONFIRM_REBOOT:
        enterMenu(OLED_MENU_SETTINGS, SI_COUNT, SI_REBOOT);
        break;
      case OLED_SURVEY_MENU:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 0);
        break;
      case OLED_SURVEY_LIST:
        enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
        break;
      case OLED_SURVEY_CODE_CAT:
        enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
        break;
      case OLED_SURVEY_CODE_CODE:
        // Back to category selection, restoring cursor on current category
        {
          int catCount = OledMenu::getCodeCatCount ? OledMenu::getCodeCatCount() : 0;
          enterMenu(OLED_SURVEY_CODE_CAT, max(catCount, 1), s_selCatIdx);
        }
        break;
      case OLED_SURVEY_MEASURE:
      case OLED_SURVEY_RESULT:
      case OLED_SURVEY_QUALITY_WARN:
        enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
        break;
      case OLED_STAKEOUT_MENU:
        enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 1);
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

  // --- Rotation: scroll the list ---
  if (direction != 0) {
    // Cycle home pages with rotation when not in a menu
    if (s_state == OLED_NORMAL || s_state == OLED_PAGE2 || s_state == OLED_PAGE3) {
      if (direction > 0) {
        // CW: next page
        if (s_state == OLED_NORMAL)       s_state = OLED_PAGE2;
        else if (s_state == OLED_PAGE2)   s_state = OLED_PAGE3;
        else if (s_state == OLED_PAGE3)   s_state = OLED_NORMAL;
      } else {
        // CCW: previous page
        if (s_state == OLED_NORMAL)       s_state = OLED_PAGE3;
        else if (s_state == OLED_PAGE2)   s_state = OLED_NORMAL;
        else if (s_state == OLED_PAGE3)   s_state = OLED_PAGE2;
      }
      return;
    }
    switch (s_state) {
      case OLED_MENU_MAIN:
      case OLED_MENU_SETTINGS:
      case OLED_SUBMENU_RATE:
      case OLED_SUBMENU_BRIGHT:
      case OLED_SURVEY_MENU:
      case OLED_SURVEY_LIST:
      case OLED_SURVEY_CODE_CAT:
      case OLED_SURVEY_CODE_CODE:
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

  // --- Click: confirm / toggle / enter sub-menu ---
  if (!click) return;

  switch (s_state) {

    // ---- Main menu ----
    case OLED_MENU_MAIN:
      switch (s_cursor) {
        case 0: // Surveys (first item)
          enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
          break;
        case 1: // Stakeout
          enterMenu(OLED_STAKEOUT_MENU, STAKEOUT_COUNT);
          break;
        case 2: // Settings
          enterMenu(OLED_MENU_SETTINGS, SI_COUNT);
          break;
        case 3: // Toggle Raw Log
          if (OledMenu::onToggleRawLog) {
            bool cur = getBool(OledMenu::getRawLogState);
            OledMenu::onToggleRawLog(!cur);
          }
          break;
        case 4: // Back
          s_state = OLED_NORMAL;
          break;
      }
      break;

    // ---- Survey sub-menu ----
    case OLED_SURVEY_MENU:
      switch (s_cursor) {
        case 0: // Measure point — enter code category selection first
          {
            int catCount = OledMenu::getCodeCatCount ? OledMenu::getCodeCatCount() : 0;
            if (catCount > 0) {
              enterMenu(OLED_SURVEY_CODE_CAT, catCount, s_selCatIdx);
            } else {
              // No codes configured — fall back to direct measure
              bool ok = (!OledMenu::getQualityOK || OledMenu::getQualityOK());
              if (!ok) {
                s_state = OLED_SURVEY_QUALITY_WARN;
              } else {
                if (OledMenu::onMeasurePoint) OledMenu::onMeasurePoint();
                s_state = OLED_SURVEY_MEASURE;
              }
            }
          }
          break;
        case 1: { // Surveys (list)
          int cnt = OledMenu::getSurveyCount ? OledMenu::getSurveyCount() : 0;
          enterMenu(OLED_SURVEY_LIST, max(cnt, 1));
          break;
        }
        case 2: // New survey
          if (OledMenu::onCreateSurvey) OledMenu::onCreateSurvey();
          break;
        case 3: // Back
          enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 0);
          break;
      }
      break;

    // ---- Survey list ----
    case OLED_SURVEY_LIST:
      // Click = set active survey
      if (OledMenu::onSetActiveSurvey) OledMenu::onSetActiveSurvey(s_cursor);
      enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
      break;

    // ---- Code category selection ----
    case OLED_SURVEY_CODE_CAT:
      {
        s_selCatIdx = s_cursor;
        int codeCount = OledMenu::getCodeCount ? OledMenu::getCodeCount(s_selCatIdx) : 0;
        enterMenu(OLED_SURVEY_CODE_CODE, max(codeCount, 1));
      }
      break;

    // ---- Code selection ----
    case OLED_SURVEY_CODE_CODE:
      {
        if (OledMenu::getCodeCod && OledMenu::getCodeLabel) {
          s_selCod    = OledMenu::getCodeCod(s_selCatIdx, s_cursor);
          s_selCodLbl = OledMenu::getCodeLabel(s_selCatIdx, s_cursor);
        }
        // Quality check
        bool ok = (!OledMenu::getQualityOK || OledMenu::getQualityOK());
        if (!ok) {
          s_state = OLED_SURVEY_QUALITY_WARN;
        } else {
          if (OledMenu::onMeasureWithCode) {
            OledMenu::onMeasureWithCode(s_selCod, s_selCodLbl, false);
          } else if (OledMenu::onMeasurePoint) {
            OledMenu::onMeasurePoint();
          }
          s_state = OLED_SURVEY_MEASURE;
        }
      }
      break;

    // ---- Measure screen ----
    case OLED_SURVEY_MEASURE: {
      // Check if measuring is done
      bool m = OledMenu::isMeasuring ? OledMenu::isMeasuring() : false;
      if (!m) {
        s_state = OLED_SURVEY_RESULT;
      }
      break;
    }

    // ---- Result screen ----
    case OLED_SURVEY_RESULT:
      enterMenu(OLED_SURVEY_MENU, SURVEY_COUNT);
      break;

    // ---- Quality warning ----
    case OLED_SURVEY_QUALITY_WARN:
      // Click = force measure anyway (bypass quality gate)
      if (OledMenu::onMeasureWithCode) {
        OledMenu::onMeasureWithCode(s_selCod, s_selCodLbl, true);
      } else if (OledMenu::onForceMeasurePoint) {
        OledMenu::onForceMeasurePoint();
      } else if (OledMenu::onMeasurePoint) {
        OledMenu::onMeasurePoint();
      }
      s_state = OLED_SURVEY_MEASURE;
      break;

    // ---- Stakeout sub-menu ----
    case OLED_STAKEOUT_MENU:
      switch (s_cursor) {
        case 0: // Navigate → live nav screen
          s_state = OLED_STAKEOUT_NAV;
          break;
        case 1: // Files → file list
          {
            int cnt = OledMenu::getStakeoutFileCount ? OledMenu::getStakeoutFileCount() : 0;
            enterMenu(OLED_STAKEOUT_FILE_LIST, max(cnt, 1));
          }
          break;
        case 2: // Back
          enterMenu(OLED_MENU_MAIN, MAIN_COUNT, 1);
          break;
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
      // Click = back to stakeout menu
      enterMenu(OLED_STAKEOUT_MENU, STAKEOUT_COUNT);
      break;

    // ---- Settings list ----
    case OLED_MENU_SETTINGS:
      if (SETTINGS_IS_TOGGLE[s_cursor]) {
        toggleSettingsItem(s_cursor);
      } else {
        switch (s_cursor) {
          case SI_ZED_RATE:
            enterMenu(OLED_SUBMENU_RATE, RATE_COUNT);
            break;
          case SI_BRIGHTNESS:
            enterMenu(OLED_SUBMENU_BRIGHT, BRIGHT_COUNT);
            break;
          case SI_REBOOT:
            s_state = OLED_CONFIRM_REBOOT;
            break;
          case SI_SYNC_SD:
            if (OledMenu::onSyncToSD) OledMenu::onSyncToSD();
            break;
          case SI_INFO:
            s_state = OLED_INFO_SCREEN;
            break;
          case SI_BACK:
            enterMenu(OLED_MENU_MAIN, MAIN_COUNT);
            break;
          default:
            break;
        }
      }
      break;

    // ---- ZED Rate sub-menu ----
    case OLED_SUBMENU_RATE:
      if (OledMenu::onSetRate) {
        OledMenu::onSetRate(RATE_MS[s_cursor]);
      }
      enterMenu(OLED_MENU_SETTINGS, SI_COUNT, SI_ZED_RATE);
      break;

    // ---- Brightness sub-menu ----
    case OLED_SUBMENU_BRIGHT:
      // s_cursor: 0=Normal, 1=Dim, 2=Off
      // Brightness control is handled entirely in draw() via display reference;
      // here we just return to settings.
      enterMenu(OLED_MENU_SETTINGS, SI_COUNT, SI_BRIGHTNESS);
      break;

    // ---- Info screen ----
    case OLED_INFO_SCREEN:
      // Click anywhere → back to settings
      enterMenu(OLED_MENU_SETTINGS, SI_COUNT, SI_INFO);
      break;

    // ---- Reboot confirmation ----
    case OLED_CONFIRM_REBOOT:
      // Click = confirm reboot
      if (OledMenu::onReboot) {
        OledMenu::onReboot();
      }
      break;

    default:
      break;
  }
}

// ---------------------------------------------------------------------------
// OledMenu::draw
// ---------------------------------------------------------------------------
void draw(Adafruit_SSD1306& disp) {

  disp.clearDisplay();
  disp.setTextColor(SSD1306_WHITE);
  disp.setTextSize(1);

  // ---- Draw dashed separator ----
  auto dashedLine = [&](int y) {
    for (int x = 0; x < SCREEN_W; x += 4) {
      disp.drawPixel(x,   y, SSD1306_WHITE);
      disp.drawPixel(x+1, y, SSD1306_WHITE);
    }
  };

  // ---- Draw a list item row ----
  // selected items are drawn inverted (white bg, black text)
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

  // ---- Hint bar ----
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
        const char* label;
        if (i == 3) {
          // Raw log toggle: index 3 in main menu
          bool logActive = OledMenu::getRawLogState && OledMenu::getRawLogState();
          label = logActive ? "Stop Raw Log" : "Start Raw Log";
        } else {
          label = MAIN_ITEMS[i];
        }
        drawItem(y, label, nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Settings list ----
    case OLED_MENU_SETTINGS: {
      disp.setCursor(0, TITLE_Y);
      disp.print("* SETTINGS");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < s_listCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        char rightBuf[8] = "";
        if (SETTINGS_IS_TOGGLE[i]) {
          strncpy(rightBuf, settingsItemState(i) ? "ON" : "OFF", sizeof(rightBuf) - 1);
        } else if (i == SI_ZED_RATE && OledMenu::getRateString) {
          strncpy(rightBuf, OledMenu::getRateString(), sizeof(rightBuf) - 1);
        }
        drawItem(y, SETTINGS_LABELS[i], rightBuf, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- ZED Rate sub-menu ----
    case OLED_SUBMENU_RATE: {
      disp.setCursor(0, TITLE_Y);
      disp.print("ZED RATE");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < s_listCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, RATE_LABELS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *set <back");
      break;
    }

    // ---- Brightness sub-menu ----
    case OLED_SUBMENU_BRIGHT: {
      disp.setCursor(0, TITLE_Y);
      disp.print("BRIGHTNESS");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < s_listCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, BRIGHT_LABELS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *set <back");

      // Apply brightness immediately as user scrolls
      if (s_cursor == 0) {
        // Normal
        disp.ssd1306_command(SSD1306_SETCONTRAST);
        disp.ssd1306_command(BRIGHTNESS_NORMAL);
        disp.dim(false);
      } else if (s_cursor == 1) {
        // Dim
        disp.dim(true);
      } else {
        // Off
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
      // Multi-line info provided by main.cpp via getInfoString()
      if (OledMenu::getInfoString) {
        String info = OledMenu::getInfoString();
        // Split on '\n' and draw up to MAX_VISIBLE lines
        int lineIdx = 0;
        int start   = 0;
        while (start < (int)info.length() && lineIdx < MAX_VISIBLE) {
          int end = info.indexOf('\n', start);
          if (end < 0) end = info.length();
          String seg = info.substring(start, end);
          disp.setCursor(0, ITEMS_Y_START + lineIdx * ROW_H);
          disp.print(seg);
          start = end + 1;
          lineIdx++;
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
      disp.print("*=Yes  <=No");
      break;
    }

    // ---- Survey sub-menu ----
    case OLED_SURVEY_MENU: {
      disp.setCursor(0, TITLE_Y);
      disp.print("* SURVEYS");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < s_listCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, SURVEY_ITEMS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Survey list ----
    case OLED_SURVEY_LIST: {
      disp.setCursor(0, TITLE_Y);
      disp.print("SURVEYS");
      dashedLine(SEP_Y);
      int cnt = OledMenu::getSurveyCount ? OledMenu::getSurveyCount() : 0;
      if (cnt == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("No surveys");
      } else {
        for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String lbl = OledMenu::getSurveyLabel ? OledMenu::getSurveyLabel(i) : String(i);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *active <back");
      break;
    }

    // ---- Code category selection ----
    case OLED_SURVEY_CODE_CAT: {
      disp.setCursor(0, TITLE_Y);
      disp.print("CATEGORY");
      dashedLine(SEP_Y);
      int catCount = OledMenu::getCodeCatCount ? OledMenu::getCodeCatCount() : 0;
      if (catCount == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("No categories");
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
      static const int TITLE_MAX_CHARS = 10;  // chars shown in title for category name
      static const int COD_MAX_CHARS   = 9;   // chars shown for cod in measure title
      disp.setCursor(0, TITLE_Y);
      // Show first TITLE_MAX_CHARS chars of category label as title
      if (OledMenu::getCodeCatLabel) {
        String catLbl = OledMenu::getCodeCatLabel(s_selCatIdx);
        if (catLbl.length() > TITLE_MAX_CHARS) catLbl = catLbl.substring(0, TITLE_MAX_CHARS);
        disp.print(catLbl);
      } else {
        disp.print("CODE");
      }
      dashedLine(SEP_Y);
      int codeCount = OledMenu::getCodeCount ? OledMenu::getCodeCount(s_selCatIdx) : 0;
      if (codeCount == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("No codes");
      } else {
        for (int i = s_scrollTop; i < codeCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String cod = OledMenu::getCodeCod   ? OledMenu::getCodeCod(s_selCatIdx, i)   : "";
          String lbl = OledMenu::getCodeLabel ? OledMenu::getCodeLabel(s_selCatIdx, i) : "";
          // Build display: "COD - Label" or just "COD" if too long
          String display = cod + " - " + lbl;
          if (display.length() > MAX_DISP_CHARS) display = cod; // fallback to code only
          if (display.length() > MAX_DISP_CHARS) display = display.substring(0, MAX_DISP_CHARS);
          drawItem(y, display.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Survey measure screen ----
    case OLED_SURVEY_MEASURE: {
      static const int MISURA_PREFIX = 7;    // "MEASURE" label width in snprintf
      static const int COD_MAX_CHARS = 8;    // max cod chars on title line
      disp.setCursor(0, TITLE_Y);
      if (s_selCod.length() > 0) {
        char tbuf[MAX_DISP_CHARS + 1];
        snprintf(tbuf, sizeof(tbuf), "%-*s %s", MISURA_PREFIX, "MEASURE",
                 s_selCod.substring(0, COD_MAX_CHARS).c_str());
        disp.print(tbuf);
      } else {
        disp.print("MEASURE...");
      }
      dashedLine(SEP_Y);
      bool m = OledMenu::isMeasuring ? OledMenu::isMeasuring() : false;
      if (m) {
        String prog = OledMenu::getMeasureProgressStr ? OledMenu::getMeasureProgressStr() : "...";
        // Show progress lines (split on \n)
        int lineIdx = 0, start = 0;
        while (start < (int)prog.length() && lineIdx < MAX_VISIBLE) {
          int end = prog.indexOf('\n', start);
          if (end < 0) end = prog.length();
          disp.setCursor(0, ITEMS_Y_START + lineIdx * ROW_H);
          disp.print(prog.substring(start, end));
          start = end + 1; lineIdx++;
        }
      } else {
        // Auto-advance to result when done
        s_state = OLED_SURVEY_RESULT;
      }
      drawHint("*=ok");
      break;
    }

    // ---- Survey result screen ----
    case OLED_SURVEY_RESULT: {
      disp.setCursor(0, TITLE_Y);
      disp.print("RESULT");
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
      // Show selected code label on last visible line
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
      disp.print("Low quality!");
      disp.setCursor(0, 22);
      disp.print("*=Force  <=Cancel");
      break;
    }

    // ---- Stakeout sub-menu ----
    case OLED_STAKEOUT_MENU: {
      disp.setCursor(0, TITLE_Y);
      disp.print("* STAKEOUT");
      dashedLine(SEP_Y);
      for (int i = s_scrollTop; i < s_listCount && (i - s_scrollTop) < MAX_VISIBLE; i++) {
        int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
        drawItem(y, STAKEOUT_ITEMS[i], nullptr, i == s_cursor);
      }
      drawHint("^ scroll *ok <back");
      break;
    }

    // ---- Stakeout file list ----
    case OLED_STAKEOUT_FILE_LIST: {
      disp.setCursor(0, TITLE_Y);
      disp.print("STAKEOUT FILES");
      dashedLine(SEP_Y);
      int cnt = OledMenu::getStakeoutFileCount ? OledMenu::getStakeoutFileCount() : 0;
      if (cnt == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("No files");
      } else {
        for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String lbl = OledMenu::getStakeoutFileLabel ? OledMenu::getStakeoutFileLabel(i) : String(i);
          if (lbl.length() > MAX_DISP_CHARS) lbl = lbl.substring(0, MAX_DISP_CHARS);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *open <back");
      break;
    }

    // ---- Stakeout point list ----
    case OLED_STAKEOUT_POINT_LIST: {
      disp.setCursor(0, TITLE_Y);
      disp.print("POINTS");
      dashedLine(SEP_Y);
      int cnt = OledMenu::getStakeoutPointCount ? OledMenu::getStakeoutPointCount() : 0;
      if (cnt == 0) {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("No points");
      } else {
        for (int i = s_scrollTop; i < cnt && (i - s_scrollTop) < MAX_VISIBLE; i++) {
          int y = ITEMS_Y_START + (i - s_scrollTop) * ROW_H;
          String lbl = OledMenu::getStakeoutPointLabel ? OledMenu::getStakeoutPointLabel(i) : String(i);
          if (lbl.length() > MAX_DISP_CHARS) lbl = lbl.substring(0, MAX_DISP_CHARS);
          drawItem(y, lbl.c_str(), nullptr, i == s_cursor);
        }
      }
      drawHint("^ scroll *set <back");
      break;
    }

    // ---- Stakeout NAV screen ----
    case OLED_STAKEOUT_NAV: {
      // Title row: fix badge + "STAKEOUT"
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
          start = end + 1;
          lineIdx++;
        }
      } else {
        disp.setCursor(0, ITEMS_Y_START);
        disp.print("No target");
      }
      drawHint("*=menu");
      break;
    }

    default:
      break;
  }

  disp.display();
}

const String& getSelectedCod() {
  return s_selCod;
}

const String& getSelectedCodLabel() {
  return s_selCodLbl;
}

} // namespace OledMenu
