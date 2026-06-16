// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "SurveyPoints.h"
#include "FlashConfig.h"
#include "config.h"
#include <LittleFS.h>
#include <math.h>
#include <time.h>
#include <algorithm>

#define SURVEYS_DIR        "/surveys"
#define ACTIVE_SURVEY_FILE "/surveys/_active"
#define ROBUST_SIGMA       2.0f
#define ROBUST_TRIM_Q      0.10f
#define ROBUST_MIN_KEEP    5
#define MAX_SAMPLES        120
#define WGS84_A            6378137.0
#define SYNC_INTERVAL_MS   (5UL * 60UL * 1000UL)
static const double DEG2RAD = M_PI / 180.0;

// ============================================================
// Namespace globals
// ============================================================
namespace SurveyPoints {
float            maxHAcc          = 0.10f;
float            maxPDOP          = 3.0f;
int              minSV            = 8;
SnapshotFn       snapshotProvider = nullptr;
SdFat*           _sd              = nullptr;
SemaphoreHandle_t _sdMutex        = nullptr;
bool*            _sdOK            = nullptr;
} // namespace SurveyPoints

static SemaphoreHandle_t g_measureMutex = nullptr;
static MeasureProgress   g_measureProg;
static MeasureParams     g_measureParams;
static TaskHandle_t      g_measureTask  = nullptr;
static uint32_t          g_lastSyncMs   = 0;

// EXTINT marker flag defined in main.cpp
extern volatile bool g_extintMarkerEnabled;

// ============================================================
// EXTINT / PPK event marker helper
// ============================================================
//
// ZED-F9P EXTINT is an input. RTKino must generate a short pulse
// only when a survey sample is actually acquired. The receiver
// timestamps the rising/falling edge and reports it through UBX-TIM-TM2.
//
// Idle level is LOW. The pin must never be kept HIGH for the whole
// measurement, otherwise TIM-TM2 marks the measurement start/end rather
// than the individual samples.
static constexpr uint32_t EXTINT_MARKER_PULSE_US = 5000;

static inline void extintMarkerBeginPulse()
{
    digitalWrite(EXTINT_GPIO, LOW);
    delayMicroseconds(50);
    digitalWrite(EXTINT_GPIO, HIGH);
}

static inline void extintMarkerEndPulse()
{
    delayMicroseconds(EXTINT_MARKER_PULSE_US);
    digitalWrite(EXTINT_GPIO, LOW);
}

// ============================================================
// Static helpers
// ============================================================
static String currentTimestamp() {
    time_t now = time(nullptr);
    struct tm t;
    localtime_r(&now, &t);
    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &t);
    return String(buf);
}

static String generateSurveyId() {
    time_t now = time(nullptr);
    char buf[20];
    snprintf(buf, sizeof(buf), "%lu", (unsigned long)now);
    return String(buf);
}

static String jsonEscape(const String& s) {
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

// ============================================================
// Robust averaging — float
// ============================================================
static float robustAvgF(std::vector<float>& vals,
                        float* sigma_out = nullptr,
                        int*   n_kept_out = nullptr)
{
    std::vector<float> v;
    v.reserve(vals.size());
    for (float x : vals) {
        if (!isnan(x) && !isinf(x)) v.push_back(x);
    }
    int n = (int)v.size();
    if (n == 0) {
        if (sigma_out)  *sigma_out  = 0;
        if (n_kept_out) *n_kept_out = 0;
        return NAN;
    }
    if (n < 3) {
        float sum = 0;
        for (float x : v) sum += x;
        float m = sum / n;
        if (sigma_out)  *sigma_out  = 0;
        if (n_kept_out) *n_kept_out = n;
        return m;
    }

    float sum = 0;
    for (float x : v) sum += x;
    float mean = sum / n;
    float sq   = 0;
    for (float x : v) sq += (x - mean) * (x - mean);
    float sd = sqrtf(sq / n);

    if (sd > 0) {
        std::vector<float> kept;
        for (float x : v) {
            if (fabsf(x - mean) <= ROBUST_SIGMA * sd) kept.push_back(x);
        }
        int minKeep = max(ROBUST_MIN_KEEP, n / 2);
        if ((int)kept.size() >= minKeep) {
            float ks = 0;
            for (float x : kept) ks += x;
            float km = ks / (float)kept.size();
            if (sigma_out) {
                float ksq = 0;
                for (float x : kept) ksq += (x - km) * (x - km);
                *sigma_out = sqrtf(ksq / (float)kept.size());
            }
            if (n_kept_out) *n_kept_out = (int)kept.size();
            return km;
        }
    }

    // Trim fallback
    std::sort(v.begin(), v.end());
    int trim = max(1, (int)(n * ROBUST_TRIM_Q));
    int end_ = n - trim;
    if (end_ > trim && (end_ - trim) >= max(ROBUST_MIN_KEEP, n / 2)) {
        float ks = 0;
        int   cnt = 0;
        for (int i = trim; i < end_; i++) { ks += v[i]; cnt++; }
        float km = ks / cnt;
        if (sigma_out) {
            float ksq = 0;
            for (int i = trim; i < end_; i++) ksq += (v[i] - km) * (v[i] - km);
            *sigma_out = cnt > 1 ? sqrtf(ksq / cnt) : 0.0f;
        }
        if (n_kept_out) *n_kept_out = cnt;
        return km;
    }

    // Median fallback
    int   mid = n / 2;
    float med = (n % 2 == 0) ? (v[mid - 1] + v[mid]) / 2.0f : v[mid];
    if (sigma_out)  *sigma_out  = 0;
    if (n_kept_out) *n_kept_out = 1;
    return med;
}

// ============================================================
// Robust averaging — double
// ============================================================
static double robustAvgD(std::vector<double>& vals,
                         double* sigma_out  = nullptr,
                         int*    n_kept_out = nullptr)
{
    std::vector<double> v;
    v.reserve(vals.size());
    for (double x : vals) {
        if (!isnan(x) && !isinf(x)) v.push_back(x);
    }
    int n = (int)v.size();
    if (n == 0) {
        if (sigma_out)  *sigma_out  = 0;
        if (n_kept_out) *n_kept_out = 0;
        return NAN;
    }
    if (n < 3) {
        double sum = 0;
        for (double x : v) sum += x;
        double m = sum / n;
        if (sigma_out)  *sigma_out  = 0;
        if (n_kept_out) *n_kept_out = n;
        return m;
    }

    double sum = 0;
    for (double x : v) sum += x;
    double mean = sum / n;
    double sq   = 0;
    for (double x : v) sq += (x - mean) * (x - mean);
    double sd = sqrt(sq / n);

    if (sd > 0) {
        std::vector<double> kept;
        for (double x : v) {
            if (fabs(x - mean) <= ROBUST_SIGMA * sd) kept.push_back(x);
        }
        int minKeep = max(ROBUST_MIN_KEEP, n / 2);
        if ((int)kept.size() >= minKeep) {
            double ks = 0;
            for (double x : kept) ks += x;
            double km = ks / (double)kept.size();
            if (sigma_out) {
                double ksq = 0;
                for (double x : kept) ksq += (x - km) * (x - km);
                *sigma_out = sqrt(ksq / (double)kept.size());
            }
            if (n_kept_out) *n_kept_out = (int)kept.size();
            return km;
        }
    }

    std::sort(v.begin(), v.end());
    int trim = max(1, (int)(n * ROBUST_TRIM_Q));
    int end_ = n - trim;
    if (end_ > trim && (end_ - trim) >= max(ROBUST_MIN_KEEP, n / 2)) {
        double ks = 0;
        int    cnt = 0;
        for (int i = trim; i < end_; i++) { ks += v[i]; cnt++; }
        double km = ks / cnt;
        if (sigma_out) {
            double ksq = 0;
            for (int i = trim; i < end_; i++) ksq += (v[i] - km) * (v[i] - km);
            *sigma_out = cnt > 1 ? sqrt(ksq / cnt) : 0.0;
        }
        if (n_kept_out) *n_kept_out = cnt;
        return km;
    }

    int    mid = n / 2;
    double med = (n % 2 == 0) ? (v[mid - 1] + v[mid]) / 2.0 : v[mid];
    if (sigma_out)  *sigma_out  = 0;
    if (n_kept_out) *n_kept_out = 1;
    return med;
}

// ============================================================
// Forward declaration for the measurement task helper
// ============================================================
static bool buildAndSavePoint(const MeasureParams& params,
                              const GNSSSnapshot*  snaps,
                              int                  n);
static void measureTask(void* pvParam);

// ============================================================
// namespace SurveyPoints — public API
// ============================================================
namespace SurveyPoints {

void begin(SdFat& sd, SemaphoreHandle_t sdMutex, bool& sdOK) {
    _sd      = &sd;
    _sdMutex = sdMutex;
    _sdOK    = &sdOK;

    g_measureMutex = xSemaphoreCreateMutex();
    g_measureProg  = MeasureProgress();

    FlashConfig::mkdir(SURVEYS_DIR);
    Serial.println("[SurveyPts] Initialized");
}

// ---- createSurvey ----
String createSurvey(const String& title, const String& desc) {
    String sid = generateSurveyId();
    String ts  = currentTimestamp();

    String json = "{\"type\":\"FeatureCollection\",";
    json += "\"name\":\"" + jsonEscape(title) + "\",";
    json += "\"properties\":{";
    json += "\"id\":\""      + sid                   + "\",";
    json += "\"title\":\""   + jsonEscape(title)      + "\",";
    json += "\"desc\":\""    + jsonEscape(desc)        + "\",";
    json += "\"created\":\"" + ts                     + "\",";
    json += "\"app\":\"RTKino\",";
    json += "\"version\":\"multi-pt-2\"";
    json += "},\"features\":[]}";

    String path = String(SURVEYS_DIR) + "/" + sid + ".json";
    if (!FlashConfig::writeFile(path.c_str(), json)) {
        Serial.println("[SurveyPts] Failed to create survey");
        return "";
    }
    Serial.printf("[SurveyPts] Created survey %s: %s\n", sid.c_str(), title.c_str());
    return sid;
}

// ---- deleteSurvey ----
bool deleteSurvey(const String& sid) {
    if (sid.isEmpty()) return false;
    String path = String(SURVEYS_DIR) + "/" + sid + ".json";
    bool ok = FlashConfig::remove(path.c_str());
    if (getActiveSurveyId() == sid) {
        FlashConfig::remove(ACTIVE_SURVEY_FILE);
    }
    return ok;
}

// ---- listSurveyIds ----
bool listSurveyIds(std::vector<String>& ids) {
    ids.clear();
    File root = LittleFS.open(SURVEYS_DIR);
    if (!root) return false;

    File f = root.openNextFile();
    while (f) {
        if (!f.isDirectory()) {
            String name = f.name();  // filename only (no path prefix)
            if (name.endsWith(".json") && !name.startsWith("_")) {
                // Remove .json extension to get sid
                ids.push_back(name.substring(0, name.length() - 5));
            }
        }
        f = root.openNextFile();
    }
    root.close();
    return true;
}

// ---- loadSurveyJSON ----
String loadSurveyJSON(const String& sid) {
    if (sid.isEmpty()) return "";
    String path = String(SURVEYS_DIR) + "/" + sid + ".json";
    return FlashConfig::readFile(path.c_str());
}

// ---- saveSurveyJSON ----
bool saveSurveyJSON(const String& sid, const String& json) {
    if (sid.isEmpty()) return false;
    String path = String(SURVEYS_DIR) + "/" + sid + ".json";
    return FlashConfig::writeFile(path.c_str(), json);
}

// ---- getActiveSurveyId ----
String getActiveSurveyId() {
    String s = FlashConfig::readFile(ACTIVE_SURVEY_FILE);
    s.trim();
    return s;
}

// ---- setActiveSurveyId ----
bool setActiveSurveyId(const String& sid) {
    return FlashConfig::writeFile(ACTIVE_SURVEY_FILE, sid + "\n");
}

// ---- getSurveyPointCount ----
int getSurveyPointCount(const String& sid) {
    String json = loadSurveyJSON(sid);
    if (json.isEmpty()) return 0;
    int count = 0;
    int pos   = 0;
    while ((pos = json.indexOf("\"type\":\"Feature\"", pos)) != -1) {
        count++;
        pos++;
    }
    return count;
}

// ---- checkQuality ----
QualityWarning checkQuality() {
    QualityWarning w;
    if (!snapshotProvider) { w.stale = true; w.details = "No GNSS provider"; return w; }

    GNSSSnapshot snap;
    snapshotProvider(snap);

    float hAcc    = snap.hpValid ? snap.hAccHP : snap.hAcc;
    w.hAccBad     = hAcc > maxHAcc;
    w.pdopBad     = snap.pdop > maxPDOP;
    w.svBad       = snap.numSV < (uint8_t)minSV;
    w.stale       = (snap.fixMode < 3);   // no 3D fix
    w.noRtk       = (snap.carrSoln == 0); // no RTK float/fixed

    // Build human-readable details string
    String d;
    if (w.stale) {
        d += "No fix 3D (fixMode=";
        d += snap.fixMode;
        d += ")";
    }
    if (w.hAccBad) {
        if (d.length()) d += "; ";
        char buf[40];
        snprintf(buf, sizeof(buf), "hAcc %.2fm > %.2fm", hAcc, maxHAcc);
        d += buf;
    }
    if (w.pdopBad) {
        if (d.length()) d += "; ";
        char buf[32];
        snprintf(buf, sizeof(buf), "PDOP %.1f > %.1f", snap.pdop, maxPDOP);
        d += buf;
    }
    if (w.svBad) {
        if (d.length()) d += "; ";
        d += "SV ";
        d += snap.numSV;
        d += " < ";
        d += minSV;
    }
    if (w.noRtk) {
        if (d.length()) d += "; ";
        d += "No RTK fix";
    }
    w.details = d;
    return w;
}

// ---- startMeasure ----
bool startMeasure(const MeasureParams& params) {
    if (!g_measureMutex) return false;
    if (xSemaphoreTake(g_measureMutex, pdMS_TO_TICKS(100)) != pdTRUE) return false;

    bool already = (g_measureTask != nullptr);
    if (!already) {
        g_measureParams     = params;
        g_measureProg       = MeasureProgress();
        g_measureProg.status = MS_RUNNING;
    }
    xSemaphoreGive(g_measureMutex);

    if (already) return false;

    BaseType_t res = xTaskCreatePinnedToCore(
        measureTask,
        "surveyMeas",
        8192,
        nullptr,
        1,
        &g_measureTask,
        1  // app CPU
    );
    return res == pdPASS;
}

// ---- getMeasureProgress ----
MeasureProgress getMeasureProgress() {
    MeasureProgress p;
    if (g_measureMutex && xSemaphoreTake(g_measureMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        p = g_measureProg;
        xSemaphoreGive(g_measureMutex);
    }
    return p;
}

// ---- isMeasuring ----
bool isMeasuring() {
    return g_measureTask != nullptr;
}

// ---- deletePoint ----
bool deletePoint(const String& sid, const String& pid) {
    String json = loadSurveyJSON(sid);
    if (json.isEmpty()) return false;

    // Find the feature block with matching "id":"pid"
    // Strategy: find "\"id\":\"<pid>\"" then walk back to the preceding '{' for a feature
    // and forward to the matching '}', removing it (plus trailing comma if any)
    String needle = "\"id\":\"" + pid + "\"";
    int    idx    = json.indexOf(needle);
    if (idx < 0) return false;

    // Walk back to find the start of this feature object
    int featureStart = -1;
    for (int i = idx; i >= 1; i--) {
        if (json[i] == '{') { featureStart = i; break; }
    }
    if (featureStart < 0) return false;

    // Walk forward counting braces to find end of object
    int depth = 0;
    int featureEnd = -1;
    for (int i = featureStart; i < (int)json.length(); i++) {
        if (json[i] == '{') depth++;
        else if (json[i] == '}') {
            depth--;
            if (depth == 0) { featureEnd = i; break; }
        }
    }
    if (featureEnd < 0) return false;

    // Remove the feature (and a trailing comma + optional whitespace if present)
    int removeStart = featureStart;
    int removeEnd   = featureEnd + 1;

    // Eat a leading comma that may precede this feature
    if (removeStart > 0 && json[removeStart - 1] == ',') {
        removeStart--;
    } else {
        // Or a trailing comma
        int after = removeEnd;
        while (after < (int)json.length() && json[after] == ' ') after++;
        if (after < (int)json.length() && json[after] == ',') removeEnd = after + 1;
    }

    String newJson = json.substring(0, removeStart) + json.substring(removeEnd);
    return saveSurveyJSON(sid, newJson);
}

// ---- getSurveyGeoJSON ----
String getSurveyGeoJSON(const String& sid) {
    return loadSurveyJSON(sid);
}

// ---- getSurveyCSV ----
String getSurveyCSV(const String& sid) {
    String json = loadSurveyJSON(sid);
    if (json.isEmpty()) return "";

    String csv = "id,name,codice,desc,timestamp,lat,lon,altHAE,altMSL,"
                 "hAcc,vAcc,pdop,hdop,vdop,ndop,edop,gdop,tdop,numSV,rtk,"
                 "ecefX,ecefY,ecefZ,pAcc,"
                 "covNN,covEE,covDD,covNE,covND,covED,"
                 "relN,relE,relD,relSN,relSE,relSD,baseline,horiz,bearingDeg,slopeDeg,"
                 "n_samples,duration_s,interval_s,sigma_N,sigma_E,sigma_U,n_kept\n";

    // Parse feature blocks — find each "type":"Feature"
    int pos = 0;
    while (true) {
        int fi = json.indexOf("\"type\":\"Feature\"", pos);
        if (fi < 0) break;

        // Find the "id" field after this marker
        auto extractStr = [&](const String& key, int from) -> String {
            String needle = "\"" + key + "\":\"";
            int    ki     = json.indexOf(needle, from);
            if (ki < 0) return "";
            int vs = ki + needle.length();
            int ve = json.indexOf("\"", vs);
            if (ve < 0) return "";
            return json.substring(vs, ve);
        };
        auto extractNum = [&](const String& key, int from) -> String {
            String needle = "\"" + key + "\":";
            int    ki     = json.indexOf(needle, from);
            if (ki < 0) return "0";
            int vs = ki + needle.length();
            // skip whitespace
            while (vs < (int)json.length() && json[vs] == ' ') vs++;
            int ve = vs;
            while (ve < (int)json.length() &&
                   (isdigit(json[ve]) || json[ve] == '.' || json[ve] == '-' || json[ve] == 'e' || json[ve] == 'E' || json[ve] == '+'))
                ve++;
            return json.substring(vs, ve);
        };

        // Find end of this feature object
        int featureStart = -1;
        for (int i = fi; i >= 1; i--) {
            if (json[i] == '{') { featureStart = i; break; }
        }
        if (featureStart < 0) { pos = fi + 1; continue; }

        int depth = 0, featureEnd = -1;
        for (int i = featureStart; i < (int)json.length(); i++) {
            if (json[i] == '{') depth++;
            else if (json[i] == '}') { if (--depth == 0) { featureEnd = i; break; } }
        }
        if (featureEnd < 0) break;

        // Extract fields from properties section
        int propsStart = json.indexOf("\"properties\":", featureStart);
        int from       = (propsStart > 0) ? propsStart : featureStart;

        csv += extractStr("id",        featureStart) + ",";
        csv += "\"" + extractStr("name",      from) + "\",";
        csv += "\"" + extractStr("codice",    from) + "\",";
        csv += "\"" + extractStr("desc",      from) + "\",";
        csv += extractStr("timestamp", from)         + ",";
        // geometry coordinates
        int geomPos = json.indexOf("\"coordinates\":", featureStart);
        String lat_s = "0", lon_s = "0", alt_s = "0";
        if (geomPos > 0) {
            int bracketPos = json.indexOf("[", geomPos);
            if (bracketPos > 0) {
                int endB = json.indexOf("]", bracketPos);
                String coords = json.substring(bracketPos + 1, endB);
                int c1 = coords.indexOf(",");
                int c2 = coords.indexOf(",", c1 + 1);
                lon_s = coords.substring(0, c1);
                lat_s = coords.substring(c1 + 1, c2 > 0 ? c2 : coords.length());
                if (c2 > 0) alt_s = coords.substring(c2 + 1);
                lat_s.trim(); lon_s.trim(); alt_s.trim();
            }
        }
        csv += lat_s + "," + lon_s + "," + alt_s + ",";
        csv += extractNum("altMSL", from) + ",";
        csv += extractNum("hAcc",   from) + ",";
        csv += extractNum("vAcc",   from) + ",";
        csv += extractNum("pdop",   from) + ",";
        csv += extractNum("hdop",   from) + ",";
        csv += extractNum("vdop",   from) + ",";
        csv += extractNum("ndop",   from) + ",";
        csv += extractNum("edop",   from) + ",";
        csv += extractNum("gdop",   from) + ",";
        csv += extractNum("tdop",   from) + ",";
        csv += extractNum("numSV",  from) + ",";
        csv += "\"" + extractStr("rtk", from) + "\",";
        csv += extractNum("X",    from) + ",";
        csv += extractNum("Y",    from) + ",";
        csv += extractNum("Z",    from) + ",";
        csv += extractNum("pAcc", from) + ",";
        csv += extractNum("NN", from) + ",";
        csv += extractNum("EE", from) + ",";
        csv += extractNum("DD", from) + ",";
        csv += extractNum("NE", from) + ",";
        csv += extractNum("ND", from) + ",";
        csv += extractNum("ED", from) + ",";
        csv += extractNum("N",           from) + ",";
        csv += extractNum("E",           from) + ",";
        csv += extractNum("D",           from) + ",";
        csv += extractNum("sN",          from) + ",";
        csv += extractNum("sE",          from) + ",";
        csv += extractNum("sD",          from) + ",";
        csv += extractNum("baseline",    from) + ",";
        csv += extractNum("horiz",       from) + ",";
        csv += extractNum("bearingDeg",  from) + ",";
        csv += extractNum("slopeDeg",    from) + ",";
        csv += extractNum("n_samples",   from) + ",";
        csv += extractNum("duration_s",  from) + ",";
        csv += extractNum("interval_s",  from) + ",";
        csv += extractNum("sigma_N",     from) + ",";
        csv += extractNum("sigma_E",     from) + ",";
        csv += extractNum("sigma_U",     from) + ",";
        csv += extractNum("n_kept",      from) + "\n";

        pos = featureEnd + 1;
    }
    return csv;
}

// ---- syncToSD ----
void syncToSD() {
    if (!_sd || !_sdMutex || !_sdOK || !*_sdOK) return;
    if (xSemaphoreTake(_sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;

    _sd->mkdir("/surveys");

    // List all survey files and copy them
    File root = LittleFS.open(SURVEYS_DIR);
    if (root) {
        File f = root.openNextFile();
        while (f) {
            if (!f.isDirectory()) {
                String name    = f.name();
                String sdPath  = "/surveys/" + name;
                String content = FlashConfig::readFile((String(SURVEYS_DIR) + "/" + name).c_str());
                if (!content.isEmpty()) {
                    FsFile sdFile = _sd->open(sdPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
                    if (sdFile) {
                        sdFile.print(content);
                        sdFile.close();
                    }
                }
            }
            f = root.openNextFile();
        }
        root.close();
    }

    xSemaphoreGive(_sdMutex);
}

// ---- periodicSync ----
void periodicSync(bool forceNow) {
    uint32_t now = millis();
    if (!forceNow && (now - g_lastSyncMs < SYNC_INTERVAL_MS)) return;
    g_lastSyncMs = now;
    syncToSD();
}

} // namespace SurveyPoints

// ============================================================
// Measurement task helpers (file-scope)
// ============================================================

// Format a double with given decimal places into a String
static String fmtD(double v, int decimals = 9) {
    char buf[32];
    snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    return String(buf);
}
static String fmtF(float v, int decimals = 4) {
    char buf[24];
    snprintf(buf, sizeof(buf), "%.*f", decimals, v);
    return String(buf);
}

// Generate next point ID given existing count
static String nextPointId(int existingCount) {
    char buf[8];
    snprintf(buf, sizeof(buf), "P%03d", existingCount + 1);
    return String(buf);
}

// RTK label from carrSoln
static String rtkLabel(uint8_t carrSoln) {
    if (carrSoln == 2) return "RTK fixed";
    if (carrSoln == 1) return "RTK float";
    return "No RTK";
}

// Build GeoJSON feature string and append it to the survey file
static bool buildAndSavePoint(const MeasureParams& params,
                              const GNSSSnapshot*  snaps,
                              int                  n)
{
    if (n <= 0) return false;

    String sid = SurveyPoints::getActiveSurveyId();
    if (sid.isEmpty()) {
        Serial.println("[SurveyPts] No active survey");
        return false;
    }

    // ---- Collect field vectors ----
    std::vector<double> v_lat, v_lon, v_altHAE, v_altMSL;
    std::vector<double> v_ecefX, v_ecefY, v_ecefZ;
    std::vector<double> v_relN, v_relE, v_relD;
    std::vector<float>  v_hAcc, v_vAcc, v_pAcc;
    std::vector<float>  v_gdop, v_pdop, v_hdop, v_vdop, v_ndop, v_edop, v_tdop;
    std::vector<float>  v_covNN, v_covEE, v_covDD, v_covNE, v_covND, v_covED;
    std::vector<float>  v_relSN, v_relSE, v_relSD;

    for (int i = 0; i < n; i++) {
        const GNSSSnapshot& s = snaps[i];
        if (s.hpValid) {
            v_lat.push_back(s.latHP);
            v_lon.push_back(s.lonHP);
            v_altHAE.push_back(s.altHAE);
            v_altMSL.push_back(s.altMSLHP);
            v_hAcc.push_back(s.hAccHP);
            v_vAcc.push_back(s.vAccHP);
        } else {
            v_lat.push_back(s.lat);
            v_lon.push_back(s.lon);
            v_altHAE.push_back((double)s.altMSL); // fallback
            v_altMSL.push_back((double)s.altMSL);
            v_hAcc.push_back(s.hAcc);
            v_vAcc.push_back(s.vAcc);
        }
        if (s.ecefValid) {
            v_ecefX.push_back(s.ecefX);
            v_ecefY.push_back(s.ecefY);
            v_ecefZ.push_back(s.ecefZ);
            v_pAcc.push_back(s.pAcc);
        }
        v_gdop.push_back(s.gdop);
        v_pdop.push_back(s.pdop);
        v_hdop.push_back(s.hdop);
        v_vdop.push_back(s.vdop);
        v_ndop.push_back(s.ndop);
        v_edop.push_back(s.edop);
        v_tdop.push_back(s.tdop);
        if (s.covValid) {
            v_covNN.push_back(s.covNN);
            v_covEE.push_back(s.covEE);
            v_covDD.push_back(s.covDD);
            v_covNE.push_back(s.covNE);
            v_covND.push_back(s.covND);
            v_covED.push_back(s.covED);
        }
        if (s.relValid) {
            v_relN.push_back(s.relN);
            v_relE.push_back(s.relE);
            v_relD.push_back(s.relD);
            v_relSN.push_back(s.relSN);
            v_relSE.push_back(s.relSE);
            v_relSD.push_back(s.relSD);
        }
    }

    // ---- Robust averages ----
    double sigma_lat = 0, sigma_lon = 0, sigma_altHAE = 0;
    int    n_kept_lat = 0;
    double mean_lat    = robustAvgD(v_lat,    &sigma_lat,    &n_kept_lat);
    double mean_lon    = robustAvgD(v_lon,    &sigma_lon,    nullptr);
    double mean_altHAE = robustAvgD(v_altHAE, &sigma_altHAE, nullptr);
    double mean_altMSL = robustAvgD(v_altMSL, nullptr,       nullptr);

    float mean_hAcc = robustAvgF(v_hAcc, nullptr, nullptr);
    float mean_vAcc = robustAvgF(v_vAcc, nullptr, nullptr);

    double mean_ecefX = v_ecefX.empty() ? 0.0 : robustAvgD(v_ecefX, nullptr, nullptr);
    double mean_ecefY = v_ecefY.empty() ? 0.0 : robustAvgD(v_ecefY, nullptr, nullptr);
    double mean_ecefZ = v_ecefZ.empty() ? 0.0 : robustAvgD(v_ecefZ, nullptr, nullptr);
    float  mean_pAcc  = v_pAcc.empty()  ? 0.0f : robustAvgF(v_pAcc, nullptr, nullptr);

    float mean_gdop = robustAvgF(v_gdop, nullptr, nullptr);
    float mean_pdop = robustAvgF(v_pdop, nullptr, nullptr);
    float mean_hdop = robustAvgF(v_hdop, nullptr, nullptr);
    float mean_vdop = robustAvgF(v_vdop, nullptr, nullptr);
    float mean_ndop = robustAvgF(v_ndop, nullptr, nullptr);
    float mean_edop = robustAvgF(v_edop, nullptr, nullptr);
    float mean_tdop = robustAvgF(v_tdop, nullptr, nullptr);

    float mean_covNN = v_covNN.empty() ? 0.0f : robustAvgF(v_covNN, nullptr, nullptr);
    float mean_covEE = v_covEE.empty() ? 0.0f : robustAvgF(v_covEE, nullptr, nullptr);
    float mean_covDD = v_covDD.empty() ? 0.0f : robustAvgF(v_covDD, nullptr, nullptr);
    float mean_covNE = v_covNE.empty() ? 0.0f : robustAvgF(v_covNE, nullptr, nullptr);
    float mean_covND = v_covND.empty() ? 0.0f : robustAvgF(v_covND, nullptr, nullptr);
    float mean_covED = v_covED.empty() ? 0.0f : robustAvgF(v_covED, nullptr, nullptr);

    double mean_relN = v_relN.empty() ? 0.0 : robustAvgD(v_relN, nullptr, nullptr);
    double mean_relE = v_relE.empty() ? 0.0 : robustAvgD(v_relE, nullptr, nullptr);
    double mean_relD = v_relD.empty() ? 0.0 : robustAvgD(v_relD, nullptr, nullptr);
    float  mean_relSN = v_relSN.empty() ? 0.0f : robustAvgF(v_relSN, nullptr, nullptr);
    float  mean_relSE = v_relSE.empty() ? 0.0f : robustAvgF(v_relSE, nullptr, nullptr);
    float  mean_relSD = v_relSD.empty() ? 0.0f : robustAvgF(v_relSD, nullptr, nullptr);

    // ---- Sigma N/E/U in metres ----
    double sigma_N = sigma_lat * DEG2RAD * WGS84_A;
    double sigma_E = sigma_lon * DEG2RAD * WGS84_A * cos(mean_lat * DEG2RAD);
    double sigma_U = sigma_altHAE;

    // ---- RELPOSNED derived ----
    double horiz      = hypot(mean_relN, mean_relE);
    double baseline   = sqrt(horiz * horiz + mean_relD * mean_relD);
    double bearingDeg = atan2(mean_relE, mean_relN) * 180.0 / M_PI;
    if (bearingDeg < 0) bearingDeg += 360.0;
    double slopeDeg   = atan2(-mean_relD, horiz) * 180.0 / M_PI;

    // ---- TPV from last sample ----
    const GNSSSnapshot& last = snaps[n - 1];
    uint8_t fixMode  = last.fixMode;
    uint8_t carrSoln = last.carrSoln;
    uint8_t numSV    = last.numSV;

    // ---- Point ID ----
    int    existing = SurveyPoints::getSurveyPointCount(sid);
    String pid      = nextPointId(existing);

    // ---- Build GeoJSON Feature ----
    String feat = "{";
    feat += "\"type\":\"Feature\",";
    feat += "\"id\":\"" + pid + "\",";
    feat += "\"geometry\":{\"type\":\"Point\",\"coordinates\":[";
    feat += fmtD(mean_lon) + "," + fmtD(mean_lat) + "," + fmtD(mean_altHAE) + "]},";
    feat += "\"properties\":{";
    feat += "\"name\":\""      + jsonEscape(params.name.isEmpty() ? pid : params.name) + "\",";
    feat += "\"codice\":\""    + jsonEscape(params.codice) + "\",";
    feat += "\"desc\":\""      + jsonEscape(params.desc)   + "\",";
    feat += "\"timestamp\":\"" + currentTimestamp()        + "\",";

    // TPV
    feat += "\"TPV\":{\"mode\":"     + String(fixMode) + ",\"rtk\":\""
          + rtkLabel(carrSoln)       + "\",\"numSV\":"  + String(numSV) + "},";

    // HPPOSLLH
    feat += "\"HPPOSLLH\":{";
    feat += "\"lat\":"    + fmtD(mean_lat) + ",";
    feat += "\"lon\":"    + fmtD(mean_lon) + ",";
    feat += "\"altHAE\":" + fmtD(mean_altHAE, 4) + ",";
    feat += "\"altMSL\":" + fmtD(mean_altMSL, 4) + ",";
    feat += "\"hAcc\":"   + fmtF(mean_hAcc) + ",";
    feat += "\"vAcc\":"   + fmtF(mean_vAcc) + "},";

    // HPPOSECEF
    feat += "\"HPPOSECEF\":{";
    feat += "\"X\":"    + fmtD(mean_ecefX, 4) + ",";
    feat += "\"Y\":"    + fmtD(mean_ecefY, 4) + ",";
    feat += "\"Z\":"    + fmtD(mean_ecefZ, 4) + ",";
    feat += "\"pAcc\":" + fmtF(mean_pAcc) + "},";

    // DOP
    feat += "\"DOP\":{";
    feat += "\"gdop\":" + fmtF(mean_gdop, 2) + ",";
    feat += "\"pdop\":" + fmtF(mean_pdop, 2) + ",";
    feat += "\"hdop\":" + fmtF(mean_hdop, 2) + ",";
    feat += "\"vdop\":" + fmtF(mean_vdop, 2) + ",";
    feat += "\"ndop\":" + fmtF(mean_ndop, 2) + ",";
    feat += "\"edop\":" + fmtF(mean_edop, 2) + ",";
    feat += "\"tdop\":" + fmtF(mean_tdop, 2) + "},";

    // COV
    feat += "\"COV\":{";
    feat += "\"NN\":" + fmtF(mean_covNN) + ",";
    feat += "\"EE\":" + fmtF(mean_covEE) + ",";
    feat += "\"DD\":" + fmtF(mean_covDD) + ",";
    feat += "\"NE\":" + fmtF(mean_covNE) + ",";
    feat += "\"ND\":" + fmtF(mean_covND) + ",";
    feat += "\"ED\":" + fmtF(mean_covED) + "},";

    // RELPOSNED
    feat += "\"RELPOSNED\":{";
    feat += "\"N\":"          + fmtD(mean_relN, 4)  + ",";
    feat += "\"E\":"          + fmtD(mean_relE, 4)  + ",";
    feat += "\"D\":"          + fmtD(mean_relD, 4)  + ",";
    feat += "\"sN\":"         + fmtF(mean_relSN)    + ",";
    feat += "\"sE\":"         + fmtF(mean_relSE)    + ",";
    feat += "\"sD\":"         + fmtF(mean_relSD)    + ",";
    feat += "\"baseline\":"   + fmtD(baseline, 4)   + ",";
    feat += "\"horiz\":"      + fmtD(horiz, 4)      + ",";
    feat += "\"bearingDeg\":" + fmtD(bearingDeg, 4) + ",";
    feat += "\"slopeDeg\":"   + fmtD(slopeDeg, 4)   + "},";

    // sampling
    feat += "\"sampling\":{";
    feat += "\"duration_s\":"  + fmtF(params.durationSec, 1)  + ",";
    feat += "\"interval_s\":"  + fmtF(params.intervalSec, 2)  + ",";
    feat += "\"n_samples\":"   + String(n)                    + ",";
    feat += "\"sigma_N\":"     + fmtD(sigma_N, 6)             + ",";
    feat += "\"sigma_E\":"     + fmtD(sigma_E, 6)             + ",";
    feat += "\"sigma_U\":"     + fmtD(sigma_U, 6)             + ",";
    feat += "\"n_kept\":"      + String(n_kept_lat)            + "}";

    feat += "}}";

    // ---- Append feature to survey JSON ----
    String surveyJson = SurveyPoints::loadSurveyJSON(sid);
    if (surveyJson.isEmpty()) {
        Serial.println("[SurveyPts] Empty survey JSON");
        return false;
    }

    // Find "features":[ ... ] end bracket
    int featArrayEnd = surveyJson.lastIndexOf(']');
    if (featArrayEnd < 0) {
        Serial.println("[SurveyPts] Malformed survey JSON");
        return false;
    }

    // Check if we need a comma (already has features)
    bool needComma = false;
    for (int i = featArrayEnd - 1; i >= 0; i--) {
        char c = surveyJson[i];
        if (c == ' ' || c == '\n' || c == '\r' || c == '\t') continue;
        if (c == '[') break;           // empty array
        needComma = true;
        break;
    }

    String newJson = surveyJson.substring(0, featArrayEnd)
                   + (needComma ? "," : "")
                   + feat
                   + surveyJson.substring(featArrayEnd);

    bool ok = SurveyPoints::saveSurveyJSON(sid, newJson);
    if (ok) {
        if (g_measureMutex && xSemaphoreTake(g_measureMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
            g_measureProg.lastPointId = pid;
            xSemaphoreGive(g_measureMutex);
        }
        Serial.printf("[SurveyPts] Saved point %s to survey %s (%d samples)\n",
                      pid.c_str(), sid.c_str(), n);
    }
    return ok;
}

// Allocate sample buffer: tries PSRAM first, then heap, shrinks if needed
static GNSSSnapshot* allocateSampleBuffer(int& maxSamples) {
    GNSSSnapshot* buf = (GNSSSnapshot*)ps_malloc(maxSamples * sizeof(GNSSSnapshot));
    if (!buf) buf     = (GNSSSnapshot*)malloc(maxSamples * sizeof(GNSSSnapshot));
    if (!buf) {
        maxSamples = min(maxSamples, 60);
        buf = (GNSSSnapshot*)malloc(maxSamples * sizeof(GNSSSnapshot));
    }
    return buf;
}

// ============================================================
// FreeRTOS measurement task
// ============================================================
static void measureTask(void* pvParam) {
    // Copy params under lock
    MeasureParams params;
    if (g_measureMutex && xSemaphoreTake(g_measureMutex, portMAX_DELAY) == pdTRUE) {
        params = g_measureParams;
        xSemaphoreGive(g_measureMutex);
    }

    // Snapshot the marker flag once — avoids races with WebUI toggling mid-task
    const bool markerEnabled = g_extintMarkerEnabled;

    // Quality gate
    if (!params.forceQuality) {
        QualityWarning w = SurveyPoints::checkQuality();
        if (w.anyBad()) {
            if (g_measureMutex && xSemaphoreTake(g_measureMutex, portMAX_DELAY) == pdTRUE) {
                g_measureProg.status   = MS_ERROR;
                g_measureProg.errorMsg = w.details.isEmpty() ? "Quality check failed" : w.details;
                xSemaphoreGive(g_measureMutex);
            }
            g_measureTask = nullptr;
            vTaskDelete(nullptr);
            return;
        }
    }

    // Allocate sample buffer
    int maxSamples = (int)(params.durationSec / params.intervalSec) + 1;
    if (maxSamples > MAX_SAMPLES) maxSamples = MAX_SAMPLES;

    GNSSSnapshot* snaps = allocateSampleBuffer(maxSamples);
    if (!snaps) {
        if (g_measureMutex && xSemaphoreTake(g_measureMutex, portMAX_DELAY) == pdTRUE) {
            g_measureProg.status   = MS_ERROR;
            g_measureProg.errorMsg = "Out of memory";
            xSemaphoreGive(g_measureMutex);
        }
        g_measureTask = nullptr;
        vTaskDelete(nullptr);
        return;
    }

    // ---- Sampling loop ----
    int      nCollected  = 0;
    uint32_t startMs     = millis();
    uint32_t durationMs  = (uint32_t)(params.durationSec * 1000.0f);
    uint32_t intervalMs  = (uint32_t)(params.intervalSec * 1000.0f);
    uint32_t lastSample  = startMs - intervalMs; // take first sample immediately

    if (markerEnabled) {
        // Make sure EXTINT starts from the idle level before the first sample.
        digitalWrite(EXTINT_GPIO, LOW);
    }

    while ((millis() - startMs) < durationMs && nCollected < maxSamples) {
        uint32_t now = millis();
        if ((now - lastSample) >= intervalMs) {
            if (SurveyPoints::snapshotProvider) {
                if (markerEnabled) {
                    // Rising edge: actual PPK marker for this survey sample.
                    extintMarkerBeginPulse();
                }

                SurveyPoints::snapshotProvider(snaps[nCollected]);
                nCollected++;
                lastSample = now;

                if (markerEnabled) {
                    // Finish the short pulse after the sample copy.
                    extintMarkerEndPulse();
                }
            }
        }
        if (g_measureMutex && xSemaphoreTake(g_measureMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
            float elapsed         = (float)(millis() - startMs) / 1000.0f;
            g_measureProg.elapsed  = elapsed;
            g_measureProg.nSamples = nCollected;
            g_measureProg.pct      = (int)min(100.0f, elapsed / params.durationSec * 100.0f);
            if (nCollected > 0) g_measureProg.curHAcc = snaps[nCollected - 1].hAcc;
            xSemaphoreGive(g_measureMutex);
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }

    // ---- Average and save ----
    if (markerEnabled) {
        // Keep EXTINT in the idle state after the measurement.
        digitalWrite(EXTINT_GPIO, LOW);
    }
    bool ok = buildAndSavePoint(params, snaps, nCollected);
    free(snaps);

    if (g_measureMutex && xSemaphoreTake(g_measureMutex, portMAX_DELAY) == pdTRUE) {
        g_measureProg.status = ok ? MS_DONE : MS_ERROR;
        g_measureProg.pct    = 100;
        if (!ok && g_measureProg.errorMsg.isEmpty()) {
            g_measureProg.errorMsg = "Failed to save point";
        }
        g_measureTask = nullptr;  // clear under mutex to prevent race with startMeasure()
        xSemaphoreGive(g_measureMutex);
    } else {
        g_measureTask = nullptr;
    }

    vTaskDelete(nullptr);
}
