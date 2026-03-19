// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "Stakeout.h"
#include "FlashConfig.h"
#include "SurveyPoints.h"
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <math.h>
#include <time.h>

// ---------------------------------------------------------------------------
// Storage layout on LittleFS
// ---------------------------------------------------------------------------
#define STAKEOUT_DIR         "/stakeout"
#define STAKEOUT_ACTIVE_FILE "/stakeout/_active.json"
#define STAKEOUT_SD_DIR      "/stakeout"   // mirrored on SD

// ArduinoJson budget for one imported file (GeoJSON)
#define STAKEOUT_JSON_BUF    32768

static const double DEG2RAD = M_PI / 180.0;
static const double WGS84_A = 6378137.0;

// ---------------------------------------------------------------------------
// Namespace globals
// ---------------------------------------------------------------------------
namespace Stakeout {
    PositionFn       positionProvider = nullptr;
    SdFat*           _sd              = nullptr;
    SemaphoreHandle_t _sdMutex        = nullptr;
    bool*            _sdOK            = nullptr;
} // namespace Stakeout

// Dirty flag — set by any mutating operation, cleared by consumer
static volatile bool s_dirty = false;

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------

static String stakeoutGenerateId() {
    time_t now = time(nullptr);
    char buf[20];
    snprintf(buf, sizeof(buf), "sk%lu", (unsigned long)now);
    return String(buf);
}

static String stakeoutJsonEscape(const String& s) {
    String out;
    out.reserve(s.length() + 4);
    for (size_t i = 0; i < s.length(); i++) {
        char c = s[i];
        if      (c == '"')  out += "\\\"";
        else if (c == '\\') out += "\\\\";
        else if (c == '\n') out += "\\n";
        else if (c == '\r') out += "\\r";
        else                out += c;
    }
    return out;
}

// Haversine great-circle distance (metres)
static double haversine2D(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG2RAD;
    double dLon = (lon2 - lon1) * DEG2RAD;
    double a = sin(dLat / 2) * sin(dLat / 2)
             + cos(lat1 * DEG2RAD) * cos(lat2 * DEG2RAD)
               * sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return WGS84_A * c;
}

// Forward azimuth from (lat1,lon1) to (lat2,lon2), degrees from North (0-360)
static double forwardAzimuth(double lat1, double lon1, double lat2, double lon2) {
    double lat1r = lat1 * DEG2RAD;
    double lat2r = lat2 * DEG2RAD;
    double dLon  = (lon2 - lon1) * DEG2RAD;
    double y = sin(dLon) * cos(lat2r);
    double x = cos(lat1r) * sin(lat2r) - sin(lat1r) * cos(lat2r) * cos(dLon);
    double az = atan2(y, x) / DEG2RAD;
    if (az < 0) az += 360.0;
    return az;
}

// Build the normalised JSON for a set of points and save to LittleFS.
// Returns the fileId on success, empty on failure.
static String saveNormalisedFile(const String& name,
                                 const std::vector<StakeoutPoint>& pts) {
    if (pts.empty()) return "";

    String fileId = stakeoutGenerateId();
    String json;
    json.reserve(256 + pts.size() * 100);
    json  = "{\"id\":\"" + fileId + "\",";
    json += "\"name\":\"" + stakeoutJsonEscape(name) + "\",";
    json += "\"count\":" + String((int)pts.size()) + ",";
    json += "\"points\":[";
    for (size_t i = 0; i < pts.size(); i++) {
        if (i > 0) json += ",";
        json += "{\"id\":\"" + stakeoutJsonEscape(pts[i].id) + "\",";
        json += "\"name\":\"" + stakeoutJsonEscape(pts[i].name) + "\",";
        char coord[80];
        if (isnan(pts[i].h)) {
            snprintf(coord, sizeof(coord), "\"lat\":%.9f,\"lon\":%.9f,\"h\":null",
                     pts[i].lat, pts[i].lon);
        } else {
            snprintf(coord, sizeof(coord), "\"lat\":%.9f,\"lon\":%.9f,\"h\":%.4f",
                     pts[i].lat, pts[i].lon, pts[i].h);
        }
        json += String(coord) + "}";
    }
    json += "]}";

    String path = String(STAKEOUT_DIR) + "/" + fileId + ".json";
    if (!FlashConfig::writeFile(path.c_str(), json)) {
        Serial.printf("[Stakeout] Failed to save file %s\n", fileId.c_str());
        return "";
    }
    Serial.printf("[Stakeout] Saved %s (%d pts): %s\n",
                  fileId.c_str(), (int)pts.size(), name.c_str());
    s_dirty = true;
    return fileId;
}

// ---------------------------------------------------------------------------
// CSV parser
// Accepts a raw byte buffer (content of the uploaded file).
// Header row must exist; columns detected case-insensitively.
// Recognised column names:
//   id/name → point name (if both present, name takes precedence)
//   lat / latitude
//   lon / longitude / lng
//   h / height / alt / altitude / elev / elevation / altHAE / altMSL
//
// If StakeoutCSVHints are provided, those column names override auto-detect.
// If no columns match by name, a value-range heuristic tries to detect
// lat ([-90,90]) and lon ([-180,180]) from the first data row.
// ---------------------------------------------------------------------------
static String parseCSV(const String& name, const uint8_t* data, size_t len,
                       const StakeoutCSVHints& hints) {
    // Convert to String for easy line parsing
    String content;
    content.reserve(len + 1);
    for (size_t i = 0; i < len; i++) content += (char)data[i];

    // Split into lines
    std::vector<String> lines;
    int start = 0;
    while (start < (int)content.length()) {
        int end = content.indexOf('\n', start);
        if (end < 0) end = content.length();
        String ln = content.substring(start, end);
        ln.trim();
        if (ln.length() > 0) lines.push_back(ln);
        start = end + 1;
    }
    if (lines.size() < 2) return "";  // need at least header + one data row

    // Detect delimiter (comma or semicolon) by counting occurrences in header
    char delim = ',';
    {
        int commaCount = 0, semicolonCount = 0, tabCount = 0;
        for (size_t i = 0; i < lines[0].length(); i++) {
            if (lines[0][i] == ',')  commaCount++;
            if (lines[0][i] == ';')  semicolonCount++;
            if (lines[0][i] == '\t') tabCount++;
        }
        if (semicolonCount > commaCount && semicolonCount >= tabCount) delim = ';';
        else if (tabCount > commaCount && tabCount >= semicolonCount)  delim = '\t';
    }

    // Split a line by delimiter, respecting quoted fields
    auto splitLine = [&](const String& row) -> std::vector<String> {
        std::vector<String> cells;
        int s = 0;
        while (s <= (int)row.length()) {
            int e = row.indexOf(delim, s);
            if (e < 0) e = row.length();
            String cell = row.substring(s, e);
            cell.trim();
            cell.replace("\"", "");
            cells.push_back(cell);
            s = e + 1;
        }
        return cells;
    };

    // Parse header
    std::vector<String> headers;
    {
        String hdr = lines[0];
        // Strip BOM
        hdr.replace("\xef\xbb\xbf", "");
        std::vector<String> raw = splitLine(hdr);
        for (auto& h : raw) {
            h.toLowerCase();
            headers.push_back(h);
        }
    }

    // ---- Column index resolution ----
    // Priority: hints (user-specified) > auto-detect by header name > value-range heuristic
    int colId   = -1;  // id
    int colName = -1;  // name (preferred over id)
    int colLat  = -1;
    int colLon  = -1;
    int colH    = -1;

    // Helper: find column index by header name (case-insensitive match)
    auto findCol = [&](const String& target) -> int {
        String t = target;
        t.toLowerCase();
        t.trim();
        if (t.isEmpty()) return -1;
        for (int i = 0; i < (int)headers.size(); i++) {
            if (headers[i] == t) return i;
        }
        return -1;
    };

    // Step 1: apply user hints if provided
    if (hints.latCol.length()  > 0) colLat  = findCol(hints.latCol);
    if (hints.lonCol.length()  > 0) colLon  = findCol(hints.lonCol);
    if (hints.hCol.length()    > 0) colH    = findCol(hints.hCol);
    if (hints.nameCol.length() > 0) colName = findCol(hints.nameCol);

    // Step 2: auto-detect remaining columns by well-known header names
    for (int i = 0; i < (int)headers.size(); i++) {
        String h = headers[i];
        // Name / ID
        if (colName < 0 && colId < 0) {
            if (h == "id")   colId = i;
        }
        if (colName < 0) {
            if (h == "name" || h == "nome" || h == "point" || h == "punto"
                || h == "label" || h == "descrizione" || h == "desc"
                || h == "marker" || h == "station" || h == "stazione")
                colName = i;
        }
        // Latitude
        if (colLat < 0) {
            if (h == "lat" || h == "latitude" || h == "latitudine"
                || h == "y" || h == "nord" || h == "north")
                colLat = i;
        }
        // Longitude
        if (colLon < 0) {
            if (h == "lon" || h == "longitude" || h == "longitudine"
                || h == "lng" || h == "long"
                || h == "x" || h == "est" || h == "east")
                colLon = i;
        }
        // Height / altitude
        if (colH < 0) {
            if (h == "h" || h == "height" || h == "alt" || h == "altitude"
                || h == "elev" || h == "elevation" || h == "quota"
                || h == "altitudine" || h == "althae" || h == "altmsl"
                || h == "ellheight" || h == "orthoheight"
                || h == "z" || h == "hae" || h == "hgt"
                || h == "quota_hae" || h == "quota_slm"
                || h == "up" || h == "ele")
                colH = i;
        }
    }

    // Step 3: value-range heuristic if lat or lon still missing
    // Scan first data row: values in [-90,90] → lat candidate; [-180,180] → lon candidate
    if (colLat < 0 || colLon < 0) {
        // Parse first data row
        std::vector<String> firstRow = splitLine(lines[1]);
        int latCandidate = -1, lonCandidate = -1;
        for (int i = 0; i < (int)firstRow.size(); i++) {
            // Skip columns already assigned
            if (i == colName || i == colId || i == colH) continue;
            if (i == colLat || i == colLon) continue;
            double v = firstRow[i].toDouble();
            if (firstRow[i].length() == 0 || (v == 0.0 && firstRow[i] != "0" && firstRow[i] != "0.0"))
                continue;  // not a number
            // Check if looks like lat/lon (must have decimals for geographic coords)
            if (firstRow[i].indexOf('.') < 0) continue;
            if (v >= -90.0 && v <= 90.0 && latCandidate < 0)  latCandidate = i;
            else if (v >= -180.0 && v <= 180.0 && lonCandidate < 0) lonCandidate = i;
        }
        // Also try: if we have two numeric columns in range, the first [-90,90] is lat,
        // the other is lon.  Validate with second data row if available.
        if (latCandidate >= 0 && lonCandidate >= 0) {
            if (colLat < 0) colLat = latCandidate;
            if (colLon < 0) colLon = lonCandidate;
            Serial.printf("[Stakeout] CSV heuristic: lat=col%d lon=col%d\n", colLat, colLon);
        }
    }

    if (colLat < 0 || colLon < 0) {
        Serial.println("[Stakeout] CSV: cannot detect lat/lon columns");
        return "";  // mandatory columns missing
    }
    int nameCol = (colName >= 0) ? colName : colId;  // prefer name

    Serial.printf("[Stakeout] CSV columns: lat=%d lon=%d h=%d name=%d\n",
                  colLat, colLon, colH, nameCol);

    std::vector<StakeoutPoint> pts;
    pts.reserve(lines.size() - 1);

    for (size_t li = 1; li < lines.size(); li++) {
        String row = lines[li];
        if (row.startsWith("#")) continue;  // comment line

        std::vector<String> cells = splitLine(row);
        if ((int)cells.size() <= max(colLat, colLon)) continue;

        StakeoutPoint pt;
        pt.lat = cells[colLat].toDouble();
        pt.lon = cells[colLon].toDouble();
        pt.h   = (colH >= 0 && colH < (int)cells.size() && cells[colH].length() > 0)
                 ? cells[colH].toDouble()
                 : NAN;
        pt.name = (nameCol >= 0 && nameCol < (int)cells.size())
                  ? cells[nameCol]
                  : String("P") + String((int)pts.size() + 1);
        if (pt.name.isEmpty()) pt.name = String("P") + String((int)pts.size() + 1);

        // Generate a stable ID from row index
        pt.id = String((int)pts.size() + 1);

        if (pt.lat == 0.0 && pt.lon == 0.0) continue;  // skip invalid
        pts.push_back(pt);
    }

    if (pts.empty()) return "";
    return saveNormalisedFile(name, pts);
}

// ---------------------------------------------------------------------------
// GeoJSON parser (uses ArduinoJson v6)
// ---------------------------------------------------------------------------
static String parseGeoJSON(const String& name, const uint8_t* data, size_t len) {
    DynamicJsonDocument doc(STAKEOUT_JSON_BUF);
    DeserializationError err = deserializeJson(doc, data, len);
    if (err) {
        Serial.printf("[Stakeout] GeoJSON parse error: %s\n", err.c_str());
        return "";
    }

    JsonArray features;
    if (doc.containsKey("features")) {
        features = doc["features"].as<JsonArray>();
    } else {
        return "";  // not a FeatureCollection
    }

    std::vector<StakeoutPoint> pts;
    pts.reserve(features.size());
    int idx = 0;
    for (JsonObject feature : features) {
        String geomType = feature["geometry"]["type"] | "";
        if (geomType != "Point") continue;

        JsonArray coords = feature["geometry"]["coordinates"];
        if (coords.size() < 2) continue;

        StakeoutPoint pt;
        pt.lon = coords[0].as<double>();
        pt.lat = coords[1].as<double>();
        pt.h   = (coords.size() >= 3 && !coords[2].isNull()) ? coords[2].as<double>() : NAN;

        // Try to enrich h from properties if still NaN
        // (don't override a valid non-zero coordinate h)
        JsonObject props = feature["properties"];
        if (isnan(pt.h)) {
            // Walk through known property names for height
            if (!props.isNull()) {
                JsonObject hppos = props["HPPOSLLH"];
                if (!hppos.isNull() && hppos.containsKey("altHAE")) {
                    pt.h = hppos["altHAE"].as<double>();
                } else if (props.containsKey("altHAE")) {
                    pt.h = props["altHAE"].as<double>();
                } else if (props.containsKey("altitude")) {
                    pt.h = props["altitude"].as<double>();
                } else if (props.containsKey("height")) {
                    pt.h = props["height"].as<double>();
                } else if (props.containsKey("quota")) {
                    pt.h = props["quota"].as<double>();
                } else if (props.containsKey("elev")) {
                    pt.h = props["elev"].as<double>();
                } else if (props.containsKey("ele")) {
                    pt.h = props["ele"].as<double>();
                } else if (props.containsKey("z")) {
                    pt.h = props["z"].as<double>();
                } else if (props.containsKey("hae")) {
                    pt.h = props["hae"].as<double>();
                }
            }
        }

        // Name: try name, nome, codice, id — in order of preference
        pt.name = props["name"] | props["nome"] | props["codice"] | props["id"] | "";
        pt.id   = props["id"]   | "";
        if (pt.id.isEmpty())   pt.id   = String(idx + 1);
        if (pt.name.isEmpty()) pt.name = pt.id;

        if (pt.lat == 0.0 && pt.lon == 0.0) continue;
        pts.push_back(pt);
        idx++;
    }

    if (pts.empty()) return "";
    return saveNormalisedFile(name, pts);
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
namespace Stakeout {

void begin(SdFat& sd, SemaphoreHandle_t sdMutex, bool& sdOK) {
    _sd      = &sd;
    _sdMutex = sdMutex;
    _sdOK    = &sdOK;
    FlashConfig::mkdir(STAKEOUT_DIR);
    s_dirty  = true;  // force initial load
    Serial.println("[Stakeout] Initialized");
}

bool listFiles(std::vector<StakeoutFile>& out) {
    out.clear();
    File root = LittleFS.open(STAKEOUT_DIR);
    if (!root) return false;
    File f = root.openNextFile();
    while (f) {
        if (!f.isDirectory()) {
            String fname = f.name();
            if (fname.endsWith(".json") && !fname.startsWith("_")) {
                String fileId = fname.substring(0, fname.length() - 5);
                String path   = String(STAKEOUT_DIR) + "/" + fname;
                String json   = FlashConfig::readFile(path.c_str());
                StakeoutFile sf;
                sf.fileId = fileId;
                sf.count  = 0;
                // Extract name
                int ni = json.indexOf("\"name\":\"");
                if (ni >= 0) {
                    int ns = ni + 8;
                    int ne = json.indexOf("\"", ns);
                    if (ne > ns) sf.name = json.substring(ns, ne);
                }
                if (sf.name.isEmpty()) sf.name = fileId;
                // Extract count
                int ci = json.indexOf("\"count\":");
                if (ci >= 0) {
                    int cs = ci + 8;
                    int ce = json.indexOf(',', cs);
                    if (ce < 0) ce = json.indexOf('}', cs);
                    if (ce > cs) sf.count = json.substring(cs, ce).toInt();
                }
                out.push_back(sf);
            }
        }
        f = root.openNextFile();
    }
    root.close();
    return true;
}

bool loadFilePoints(const String& fileId, std::vector<StakeoutPoint>& out) {
    out.clear();
    if (fileId.isEmpty()) return false;
    String path = String(STAKEOUT_DIR) + "/" + fileId + ".json";
    String json = FlashConfig::readFile(path.c_str());
    if (json.isEmpty()) return false;

    // Find the "points" array
    int pStart = json.indexOf("\"points\":[");
    if (pStart < 0) return false;
    pStart += 10; // skip past "points":[ 

    // Iterate objects: {"id":...,"name":...,"lat":...,"lon":...,"h":...}
    int pos = pStart;
    while (pos < (int)json.length()) {
        int objStart = json.indexOf('{', pos);
        if (objStart < 0) break;
        int objEnd = json.indexOf('}', objStart);
        if (objEnd < 0) break;
        String obj = json.substring(objStart + 1, objEnd);

        StakeoutPoint pt;
        // Extract each field
        auto extractStr = [&](const String& key) -> String {
            String search = "\"" + key + "\":\"";
            int ki = obj.indexOf(search);
            if (ki < 0) return "";
            int ks = ki + search.length();
            int ke = obj.indexOf("\"", ks);
            if (ke < ks) return "";
            return obj.substring(ks, ke);
        };
        auto extractDbl = [&](const String& key) -> double {
            String search = "\"" + key + "\":";
            int ki = obj.indexOf(search);
            if (ki < 0) return NAN;
            int ks = ki + search.length();
            // Check for "null"
            if (ks + 4 <= (int)obj.length() && obj.substring(ks, ks + 4) == "null") return NAN;
            int ke = ks;
            while (ke < (int)obj.length() &&
                   (obj[ke] == '-' || obj[ke] == '.' ||
                    (obj[ke] >= '0' && obj[ke] <= '9'))) ke++;
            if (ke == ks) return NAN;
            return obj.substring(ks, ke).toDouble();
        };

        pt.id   = extractStr("id");
        pt.name = extractStr("name");
        pt.lat  = extractDbl("lat");
        pt.lon  = extractDbl("lon");
        pt.h    = extractDbl("h");

        if (pt.id.isEmpty())   pt.id   = String((int)out.size() + 1);
        if (pt.name.isEmpty()) pt.name = pt.id;

        out.push_back(pt);
        pos = objEnd + 1;
    }
    return !out.empty();
}

bool deleteFile(const String& fileId) {
    if (fileId.isEmpty()) return false;
    // If this is the active file, clear the selection
    StakeoutActive act = getActive();
    if (act.fileId == fileId) {
        FlashConfig::remove(STAKEOUT_ACTIVE_FILE);
    }
    String path = String(STAKEOUT_DIR) + "/" + fileId + ".json";
    bool ok = FlashConfig::remove(path.c_str());
    s_dirty = true;
    return ok;
}

String importCSV(const String& name, const uint8_t* data, size_t len,
                 const StakeoutCSVHints& hints) {
    return parseCSV(name, data, len, hints);
}

String importGeoJSON(const String& name, const uint8_t* data, size_t len) {
    return parseGeoJSON(name, data, len);
}

String importFromSurvey(const String& surveyId, const String& datasetName) {
    String geojson = SurveyPoints::getSurveyGeoJSON(surveyId);
    if (geojson.isEmpty()) {
        Serial.printf("[Stakeout] No GeoJSON for survey %s\n", surveyId.c_str());
        return "";
    }
    String dsName = datasetName;
    if (dsName.isEmpty()) dsName = "Survey " + surveyId;
    const uint8_t* buf = (const uint8_t*)geojson.c_str();
    size_t len2 = geojson.length();
    return parseGeoJSON(dsName, buf, len2);
}

StakeoutActive getActive() {
    StakeoutActive act;
    String json = FlashConfig::readFile(STAKEOUT_ACTIVE_FILE);
    if (json.isEmpty()) return act;
    // Parse {"fileId":"...","pointId":"..."}
    int fi = json.indexOf("\"fileId\":\"");
    if (fi >= 0) {
        int fs = fi + 10;
        int fe = json.indexOf("\"", fs);
        if (fe > fs) act.fileId = json.substring(fs, fe);
    }
    int pi = json.indexOf("\"pointId\":\"");
    if (pi >= 0) {
        int ps = pi + 11;
        int pe = json.indexOf("\"", ps);
        if (pe > ps) act.pointId = json.substring(ps, pe);
    }
    return act;
}

bool setActive(const String& fileId, const String& pointId) {
    if (fileId.isEmpty() || pointId.isEmpty()) return false;
    String json = "{\"fileId\":\"" + stakeoutJsonEscape(fileId) +
                  "\",\"pointId\":\"" + stakeoutJsonEscape(pointId) + "\"}";
    bool ok = FlashConfig::writeFile(STAKEOUT_ACTIVE_FILE, json);
    s_dirty = true;
    return ok;
}

bool getActivePoint(StakeoutPoint& out) {
    StakeoutActive act = getActive();
    if (!act.valid()) return false;
    std::vector<StakeoutPoint> pts;
    if (!loadFilePoints(act.fileId, pts)) return false;
    for (auto& p : pts) {
        if (p.id == act.pointId) { out = p; return true; }
    }
    return false;
}

StakeoutStatus getStatus() {
    StakeoutStatus st;
    st.valid = false;
    st.d2d   = 0; st.dH = 0; st.az = 0;
    st.roverCarrSoln = 0; st.roverFixQuality = 0;

    if (!positionProvider) return st;

    // Get rover position
    double rLat, rLon, rH;
    uint8_t carrSoln, fixQuality;
    positionProvider(rLat, rLon, rH, carrSoln, fixQuality);
    st.roverCarrSoln   = carrSoln;
    st.roverFixQuality = fixQuality;

    if (rLat == 0.0 && rLon == 0.0) return st;  // no fix yet

    // Get active target point
    StakeoutPoint tpt;
    if (!getActivePoint(tpt)) return st;

    st.targetName = tpt.name;
    st.targetId   = tpt.id;
    st.targetLat  = tpt.lat;
    st.targetLon  = tpt.lon;
    st.targetH    = tpt.h;

    st.d2d  = haversine2D(rLat, rLon, tpt.lat, tpt.lon);
    st.dH   = isnan(tpt.h) ? NAN : (tpt.h - rH);
    st.az   = forwardAzimuth(rLat, rLon, tpt.lat, tpt.lon);
    st.valid = true;
    return st;
}

// ---- Dirty flag -----------------------------------------------------------
bool isDirty()  { return s_dirty; }
void clearDirty() { s_dirty = false; }

void syncToSD() {
    if (!_sd || !_sdOK || !*_sdOK || !_sdMutex) return;
    if (xSemaphoreTake(_sdMutex, pdMS_TO_TICKS(2000)) != pdTRUE) return;

    // Ensure SD directory exists
    _sd->mkdir(STAKEOUT_SD_DIR);

    // Copy every file from LittleFS /stakeout/ to SD /stakeout/
    File root = LittleFS.open(STAKEOUT_DIR);
    if (root) {
        File f = root.openNextFile();
        while (f) {
            String fname = f.name();
            String srcPath = String(STAKEOUT_DIR) + "/" + fname;
            String dstPath = String(STAKEOUT_SD_DIR) + "/" + fname;

            String content = FlashConfig::readFile(srcPath.c_str());
            if (content.length() > 0) {
                FsFile out = _sd->open(dstPath.c_str(), O_WRITE | O_CREAT | O_TRUNC);
                if (out) {
                    out.print(content);
                    out.close();
                }
            }
            f = root.openNextFile();
        }
        root.close();
    }

    xSemaphoreGive(_sdMutex);
    Serial.println("[Stakeout] Synced to SD");
}

} // namespace Stakeout
