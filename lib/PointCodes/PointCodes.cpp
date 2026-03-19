// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "PointCodes.h"
#include "FlashConfig.h"

// ---------------------------------------------------------------------------
// Storage path
// ---------------------------------------------------------------------------
static const char* PC_FLASH_PATH = "/config/codici_punto.json";

// ---------------------------------------------------------------------------
// In-memory data
// ---------------------------------------------------------------------------
static std::vector<PointCategory> s_categories;

// Flat list cache
struct FlatEntry {
  String label;
  String cod;
  int    catIdx;
  int    codeIdx;  // -1 for category-header rows
};
static std::vector<FlatEntry> s_flat;

// Maximum characters for flat-list labels (OLED display constraint)
static const int MAX_FLAT_LABEL_LENGTH = 18;

static void rebuildFlat() {
  s_flat.clear();
  for (int ci = 0; ci < (int)s_categories.size(); ci++) {
    // Category header row (cod == "")
    FlatEntry hdr;
    hdr.label    = s_categories[ci].label;
    hdr.cod      = "";
    hdr.catIdx   = ci;
    hdr.codeIdx  = -1;
    s_flat.push_back(hdr);
    // Code rows
    for (int ki = 0; ki < (int)s_categories[ci].codici.size(); ki++) {
      FlatEntry e;
      String full = s_categories[ci].label + "/" + s_categories[ci].codici[ki].label;
      if (full.length() > (unsigned)MAX_FLAT_LABEL_LENGTH) full = full.substring(0, MAX_FLAT_LABEL_LENGTH);
      e.label   = full;
      e.cod     = s_categories[ci].codici[ki].cod;
      e.catIdx  = ci;
      e.codeIdx = ki;
      s_flat.push_back(e);
    }
  }
}

// ---------------------------------------------------------------------------
// Hardcoded defaults
// ---------------------------------------------------------------------------
static const char DEFAULT_JSON[] =
  "{"
  "\"categorie\":["
    "{\"id\":\"fabbricati\",\"label\":\"Fabbricati\",\"codici\":["
      "{\"cod\":\"SPIG_FAB\",\"label\":\"Spigolo fabbricato\"},"
      "{\"cod\":\"ANG_MUR\",\"label\":\"Angolo muro\"},"
      "{\"cod\":\"SPIG_MUR\",\"label\":\"Spigolo muro\"},"
      "{\"cod\":\"PORTA\",\"label\":\"Porta / ingresso\"},"
      "{\"cod\":\"FINESTRA\",\"label\":\"Finestra\"},"
      "{\"cod\":\"COLONNA\",\"label\":\"Colonna / pilastro\"},"
      "{\"cod\":\"GRADINO\",\"label\":\"Gradino / soglia\"},"
      "{\"cod\":\"FAB_GEN\",\"label\":\"Fabbricato (generico)\"}"
    "]},"
    "{\"id\":\"recinzioni\",\"label\":\"Recinzioni e confini\",\"codici\":["
      "{\"cod\":\"SPIG_REC\",\"label\":\"Spigolo recinzione\"},"
      "{\"cod\":\"PALO_REC\",\"label\":\"Palo recinzione\"},"
      "{\"cod\":\"CANCEL\",\"label\":\"Cancello / accesso\"},"
      "{\"cod\":\"CONF_PT\",\"label\":\"Punto di confine\"},"
      "{\"cod\":\"CIPPO\",\"label\":\"Cippo / termine\"}"
    "]},"
    "{\"id\":\"viabilita\",\"label\":\"Viabilit\\u00e0 e infrastrutture\",\"codici\":["
      "{\"cod\":\"CIGL_STR\",\"label\":\"Ciglio strada\"},"
      "{\"cod\":\"MARG_STR\",\"label\":\"Margine pavimentazione\"},"
      "{\"cod\":\"AIUOLA\",\"label\":\"Bordo aiuola / isola\"},"
      "{\"cod\":\"GRAD_STR\",\"label\":\"Gradino stradale\"},"
      "{\"cod\":\"CURVA\",\"label\":\"Curva / raccordo\"},"
      "{\"cod\":\"CENTRO\",\"label\":\"Asse strada (centro)\"}"
    "]},"
    "{\"id\":\"impianti\",\"label\":\"Impianti e manufatti\",\"codici\":["
      "{\"cod\":\"POZZETTO\",\"label\":\"Pozzetto / chiusino\"},"
      "{\"cod\":\"IDRANTE\",\"label\":\"Idrante\"},"
      "{\"cod\":\"PALO_ILL\",\"label\":\"Palo illuminazione\"},"
      "{\"cod\":\"SEGNALE\",\"label\":\"Segnale stradale\"},"
      "{\"cod\":\"CABINA\",\"label\":\"Cabina / armadio\"},"
      "{\"cod\":\"BOCCHETT\",\"label\":\"Bocchetta / scarico\"}"
    "]},"
    "{\"id\":\"verde\",\"label\":\"Verde e vegetazione\",\"codici\":["
      "{\"cod\":\"ALBERO\",\"label\":\"Albero (fusto)\"},"
      "{\"cod\":\"CESPUGL\",\"label\":\"Cespuglio / siepe\"},"
      "{\"cod\":\"BORDO_VRD\",\"label\":\"Bordo area verde\"},"
      "{\"cod\":\"SCARP\",\"label\":\"Scarpata (piede)\"},"
      "{\"cod\":\"SCARP_CR\",\"label\":\"Scarpata (cresta)\"}"
    "]},"
    "{\"id\":\"quota\",\"label\":\"Quota terreno e morfologia\",\"codici\":["
      "{\"cod\":\"QT\",\"label\":\"Quota terreno\"},"
      "{\"cod\":\"QT_PIAN\",\"label\":\"Quota terreno (piano)\"},"
      "{\"cod\":\"QT_PEND\",\"label\":\"Quota terreno (pendio)\"},"
      "{\"cod\":\"FOSSO_FD\",\"label\":\"Fondo fosso\"},"
      "{\"cod\":\"FOSSO_SP\",\"label\":\"Sponda fosso\"},"
      "{\"cod\":\"CIGLIO_SC\",\"label\":\"Ciglio scarpata\"},"
      "{\"cod\":\"PIEDE_SC\",\"label\":\"Piede scarpata\"}"
    "]},"
    "{\"id\":\"idrografia\",\"label\":\"Idrografia\",\"codici\":["
      "{\"cod\":\"SPONDA\",\"label\":\"Sponda corso d'acqua\"},"
      "{\"cod\":\"PELO_AQ\",\"label\":\"Pelo libero acqua\"},"
      "{\"cod\":\"ARGINE\",\"label\":\"Argine (cresta)\"},"
      "{\"cod\":\"ARGINE_P\",\"label\":\"Argine (piede)\"}"
    "]},"
    "{\"id\":\"altro\",\"label\":\"Altro\",\"codici\":["
      "{\"cod\":\"STAZ\",\"label\":\"Stazione / base\"},"
      "{\"cod\":\"CTRL\",\"label\":\"Punto di controllo\"},"
      "{\"cod\":\"RIF\",\"label\":\"Punto di riferimento\"},"
      "{\"cod\":\"ALTRO\",\"label\":\"Altro (vedi note)\"}"
    "]}"
  "]}";

// ---------------------------------------------------------------------------
// Minimal JSON helpers (no ArduinoJson — same style as rest of codebase)
// ---------------------------------------------------------------------------

// Extract string value for a key like "key":"value" starting at pos
static String jsonStringVal(const String& s, const String& key, int fromPos = 0) {
  String search = "\"" + key + "\":\"";
  int p = s.indexOf(search, fromPos);
  if (p < 0) return "";
  int start = p + search.length();
  int end   = s.indexOf("\"", start);
  if (end < start) return "";
  // Unescape basic sequences
  String val = s.substring(start, end);
  String out = "";
  for (int i = 0; i < (int)val.length(); i++) {
    if (val[i] == '\\' && i + 1 < (int)val.length()) {
      char next = val[i+1];
      if      (next == 'n')  { out += '\n'; i++; }
      else if (next == 'r')  { out += '\r'; i++; }
      else if (next == 't')  { out += '\t'; i++; }
      else if (next == '"')  { out += '"';  i++; }
      else if (next == '\\') { out += '\\'; i++; }
      else if (next == '/')  { out += '/';  i++; }
      else if (next == 'u' && i + 5 < (int)val.length()) {
        // 4-hex unicode escape — convert simple Latin-1 range
        char hex[5] = { val[i+2], val[i+3], val[i+4], val[i+5], 0 };
        unsigned int cp = (unsigned int)strtoul(hex, nullptr, 16);
        if (cp < 0x80) {
          out += (char)cp;
        } else if (cp < 0x800) {
          out += (char)(0xC0 | (cp >> 6));
          out += (char)(0x80 | (cp & 0x3F));
        } else {
          out += (char)(0xE0 | (cp >> 12));
          out += (char)(0x80 | ((cp >> 6) & 0x3F));
          out += (char)(0x80 | (cp & 0x3F));
        }
        i += 5;
      } else {
        out += next; i++;
      }
    } else {
      out += val[i];
    }
  }
  return out;
}

// JSON-escape a string for output
static String jsonEsc(const String& s) {
  String r = "";
  for (int i = 0; i < (int)s.length(); i++) {
    unsigned char c = (unsigned char)s[i];
    if      (c == '"')  r += "\\\"";
    else if (c == '\\') r += "\\\\";
    else if (c == '\n') r += "\\n";
    else if (c == '\r') r += "\\r";
    else if (c == '\t') r += "\\t";
    else                r += (char)c;
  }
  return r;
}

// Parse all categories+codes from a JSON string
static bool parseJSON(const String& json, std::vector<PointCategory>& out) {
  out.clear();
  // Find "categorie":[...]
  int catArrStart = json.indexOf("\"categorie\"");
  if (catArrStart < 0) return false;
  catArrStart = json.indexOf("[", catArrStart);
  if (catArrStart < 0) return false;

  int pos = catArrStart + 1;
  int depth = 1;

  while (pos < (int)json.length()) {
    // Find start of next category object {
    int objStart = json.indexOf("{", pos);
    if (objStart < 0) break;

    // Find matching closing }
    // Walk forward counting braces
    int d = 0;
    int objEnd = -1;
    for (int i = objStart; i < (int)json.length(); i++) {
      if (json[i] == '{') d++;
      else if (json[i] == '}') { d--; if (d == 0) { objEnd = i; break; } }
    }
    if (objEnd < 0) break;

    String catObj = json.substring(objStart, objEnd + 1);

    PointCategory cat;
    cat.id    = jsonStringVal(catObj, "id");
    cat.label = jsonStringVal(catObj, "label");

    // Find codici array inside catObj
    int codArrStart = catObj.indexOf("\"codici\"");
    if (codArrStart >= 0) {
      codArrStart = catObj.indexOf("[", codArrStart);
      if (codArrStart >= 0) {
        int codPos = codArrStart + 1;
        while (codPos < (int)catObj.length()) {
          int codeStart = catObj.indexOf("{", codPos);
          if (codeStart < 0) break;
          int cd = 0, codeEnd = -1;
          for (int i = codeStart; i < (int)catObj.length(); i++) {
            if (catObj[i] == '{') cd++;
            else if (catObj[i] == '}') { cd--; if (cd == 0) { codeEnd = i; break; } }
          }
          if (codeEnd < 0) break;
          String codeObj = catObj.substring(codeStart, codeEnd + 1);
          PointCode pc;
          pc.cod   = jsonStringVal(codeObj, "cod");
          pc.label = jsonStringVal(codeObj, "label");
          if (pc.cod.length() > 0) cat.codici.push_back(pc);
          codPos = codeEnd + 1;
        }
      }
    }

    if (cat.id.length() > 0) out.push_back(cat);
    pos = objEnd + 1;
    (void)depth;
  }
  return !out.empty();
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

void PointCodes::begin() {
  if (FlashConfig::exists(PC_FLASH_PATH)) {
    if (loadFromFlash()) {
      Serial.println("[PointCodes] Loaded from flash");
      return;
    }
  }
  Serial.println("[PointCodes] No file found, loading defaults");
  resetToDefaults();
}

bool PointCodes::loadFromFlash() {
  String json = FlashConfig::readFile(PC_FLASH_PATH);
  if (json.length() < 10) return false;
  std::vector<PointCategory> tmp;
  if (!parseJSON(json, tmp)) return false;
  s_categories = tmp;
  rebuildFlat();
  return true;
}

bool PointCodes::saveToFlash() {
  String json = toJSON();
  bool ok = FlashConfig::writeFile(PC_FLASH_PATH, json);
  if (ok) FlashConfig::markDirty();
  return ok;
}

void PointCodes::resetToDefaults() {
  String def = String(DEFAULT_JSON);
  std::vector<PointCategory> tmp;
  parseJSON(def, tmp);
  s_categories = tmp;
  rebuildFlat();
  saveToFlash();
}

int PointCodes::getCategoryCount() {
  return (int)s_categories.size();
}

String PointCodes::getCategoryId(int catIdx) {
  if (catIdx < 0 || catIdx >= (int)s_categories.size()) return "";
  return s_categories[catIdx].id;
}

String PointCodes::getCategoryLabel(int catIdx) {
  if (catIdx < 0 || catIdx >= (int)s_categories.size()) return "";
  return s_categories[catIdx].label;
}

int PointCodes::getCodeCount(int catIdx) {
  if (catIdx < 0 || catIdx >= (int)s_categories.size()) return 0;
  return (int)s_categories[catIdx].codici.size();
}

String PointCodes::getCodeCod(int catIdx, int codeIdx) {
  if (catIdx < 0 || catIdx >= (int)s_categories.size()) return "";
  if (codeIdx < 0 || codeIdx >= (int)s_categories[catIdx].codici.size()) return "";
  return s_categories[catIdx].codici[codeIdx].cod;
}

String PointCodes::getCodeLabel(int catIdx, int codeIdx) {
  if (catIdx < 0 || catIdx >= (int)s_categories.size()) return "";
  if (codeIdx < 0 || codeIdx >= (int)s_categories[catIdx].codici.size()) return "";
  return s_categories[catIdx].codici[codeIdx].label;
}

int PointCodes::getFlatCount() {
  return (int)s_flat.size();
}

String PointCodes::getFlatLabel(int flatIdx) {
  if (flatIdx < 0 || flatIdx >= (int)s_flat.size()) return "";
  return s_flat[flatIdx].label;
}

String PointCodes::getFlatCod(int flatIdx) {
  if (flatIdx < 0 || flatIdx >= (int)s_flat.size()) return "";
  return s_flat[flatIdx].cod;
}

int PointCodes::getFlatCatIdx(int flatIdx) {
  if (flatIdx < 0 || flatIdx >= (int)s_flat.size()) return -1;
  return s_flat[flatIdx].catIdx;
}

int PointCodes::getFlatCodeIdx(int flatIdx) {
  if (flatIdx < 0 || flatIdx >= (int)s_flat.size()) return -1;
  return s_flat[flatIdx].codeIdx;
}

String PointCodes::toJSON() {
  String out = "{\"categorie\":[\n";
  for (int ci = 0; ci < (int)s_categories.size(); ci++) {
    const PointCategory& cat = s_categories[ci];
    out += "  {\"id\":\"" + jsonEsc(cat.id) + "\",";
    out += "\"label\":\"" + jsonEsc(cat.label) + "\",";
    out += "\"codici\":[\n";
    for (int ki = 0; ki < (int)cat.codici.size(); ki++) {
      out += "    {\"cod\":\"" + jsonEsc(cat.codici[ki].cod) + "\",";
      out += "\"label\":\"" + jsonEsc(cat.codici[ki].label) + "\"}";
      if (ki + 1 < (int)cat.codici.size()) out += ",";
      out += "\n";
    }
    out += "  ]}";
    if (ci + 1 < (int)s_categories.size()) out += ",";
    out += "\n";
  }
  out += "]}";
  return out;
}

bool PointCodes::fromJSON(const String& json) {
  std::vector<PointCategory> tmp;
  if (!parseJSON(json, tmp)) return false;
  s_categories = tmp;
  rebuildFlat();
  return true;
}
