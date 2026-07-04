// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "TrackRecorder.h"
#include "FlashConfig.h"
#include <LittleFS.h>
#include <math.h>
#include <time.h>

// ---------------------------------------------------------------------------
// Storage layout
//   LittleFS (primary, always):  /tracks/<id>.trk  (append-only CSV points)
//                                 /tracks/<id>.json (small metadata sidecar)
//   SD (backup only, best-effort, mirrors SurveyPoints'/Stakeout's role for SD)
// ---------------------------------------------------------------------------
#define TRACKS_DIR             "/tracks"
#define TRACKS_SD_DIR          "/tracks"
#define TRACK_MIN_FREE_BYTES   (256UL * 1024UL)
#define TRACK_BUFFER_CAPACITY  4096                 // ~140 KB, PSRAM if available
#define SAMPLE_TICK_MS         250
#define FLASH_FLUSH_MIN_POINTS 50
#define FLASH_FLUSH_MAX_MS     30000UL              // 30s — batched to protect flash endurance
#define METADATA_CHECKPOINT_MS 60000UL
#define SD_SYNC_INTERVAL_MS    (5UL * 60UL * 1000UL) // mirrors SurveyPoints::SYNC_INTERVAL_MS

static const double DEG2RAD = M_PI / 180.0;
static const double WGS84_A = 6378137.0;

namespace TrackRecorder {
    PositionFn        positionProvider = nullptr;
    SdFat*            _sd              = nullptr;
    SemaphoreHandle_t _sdMutex         = nullptr;
    bool*             _sdOK            = nullptr;
} // namespace TrackRecorder

// ---------------------------------------------------------------------------
// Point buffer (PSRAM ring buffer — mirrors SurveyPoints::allocateSampleBuffer)
// ---------------------------------------------------------------------------
struct TrackPoint {
    uint32_t epochSec;
    double   lat, lon;
    float    altHAE;
    uint8_t  fixQuality;
    uint8_t  carrSoln;
};

static TrackPoint* g_buf         = nullptr;
static int         g_bufCapacity = 0;
static int         g_bufHead     = 0;   // next write index
static int         g_bufCount    = 0;   // unflushed points currently buffered
static uint32_t    g_bufDropped  = 0;

static bool allocateTrackBuffer() {
    if (g_buf) return true;
    int capacity = TRACK_BUFFER_CAPACITY;
    TrackPoint* buf = (TrackPoint*)ps_malloc(capacity * sizeof(TrackPoint));
    if (!buf) buf = (TrackPoint*)malloc(capacity * sizeof(TrackPoint));
    if (!buf) {
        capacity = min(capacity, 256);
        buf = (TrackPoint*)malloc(capacity * sizeof(TrackPoint));
    }
    g_buf         = buf;
    g_bufCapacity = buf ? capacity : 0;
    return buf != nullptr;
}

static void bufPush(const TrackPoint& p) {
    if (!g_buf || g_bufCapacity == 0) return;
    g_buf[g_bufHead] = p;
    g_bufHead = (g_bufHead + 1) % g_bufCapacity;
    if (g_bufCount < g_bufCapacity) g_bufCount++;
    else                            g_bufDropped++;  // overwrote the oldest unflushed point
}

// ---------------------------------------------------------------------------
// Shared status/config — short-timeout mutex (mirrors positionMutex/getPosition)
// ---------------------------------------------------------------------------
static SemaphoreHandle_t g_trackMutex = nullptr;
static TrackStatus       g_status;
static String            g_trackName;
static uint32_t          g_trackStartEpoch = 0;
static uint32_t          g_startMs         = 0;

static File g_flashFile;
static bool g_fileOpen = false;

static uint32_t g_lastSampleMs    = 0;
static double   g_lastSampleLat   = NAN, g_lastSampleLon = NAN;
static uint32_t g_lastFlashFlushMs = 0;
static uint32_t g_lastCheckpointMs = 0;
static uint32_t g_lastSyncMs       = 0;

// ---------------------------------------------------------------------------
// Static helpers
// ---------------------------------------------------------------------------
static String trackGenerateId() {
    time_t now = time(nullptr);
    char buf[20];
    snprintf(buf, sizeof(buf), "tk%lu", (unsigned long)now);
    return String(buf);
}

static String trackJsonEscape(const String& s) {
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

// Haversine great-circle distance (metres) — private copy, mirrors Stakeout.cpp's own
static double haversineMeters(double lat1, double lon1, double lat2, double lon2) {
    double dLat = (lat2 - lat1) * DEG2RAD;
    double dLon = (lon2 - lon1) * DEG2RAD;
    double a = sin(dLat / 2) * sin(dLat / 2)
             + cos(lat1 * DEG2RAD) * cos(lat2 * DEG2RAD)
               * sin(dLon / 2) * sin(dLon / 2);
    return WGS84_A * 2 * atan2(sqrt(a), sqrt(1 - a));
}

static void setError(const String& msg) {
    if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_status.lastError = msg;
        xSemaphoreGive(g_trackMutex);
    }
}

static void writeMetadata(const String& id, bool finished) {
    TrackConfig cfg;
    uint32_t pointCount; double distanceM;
    if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        cfg = g_status.cfg;
        pointCount = g_status.pointCount;
        distanceM  = g_status.distanceM;
        xSemaphoreGive(g_trackMutex);
    } else return;

    String json = "{";
    json += "\"id\":\""         + id + "\",";
    json += "\"name\":\""       + trackJsonEscape(g_trackName.isEmpty() ? id : g_trackName) + "\",";
    json += "\"startEpoch\":"   + String((unsigned long)g_trackStartEpoch) + ",";
    json += "\"endEpoch\":"     + String(finished ? (unsigned long)time(nullptr) : 0UL) + ",";
    json += "\"triggerMode\":"  + String((int)cfg.triggerMode) + ",";
    json += "\"intervalSec\":"  + String(cfg.intervalSec, 2) + ",";
    json += "\"distanceThresholdM\":" + String(cfg.distanceM, 2) + ",";
    json += "\"pointCount\":"   + String(pointCount) + ",";
    json += "\"distanceM\":"    + String(distanceM, 2);
    json += "}";

    String path = String(TRACKS_DIR) + "/" + id + ".json";
    FlashConfig::writeFile(path.c_str(), json);
}

// Append every currently-buffered point to the open working file on LittleFS.
// No mutex needed — LittleFS operations never touch sdMutex, so this can
// never contend with or delay sdWriterTask.
static void flushBufferToFlash() {
    if (g_bufCount == 0 || !g_fileOpen) return;
    int tail = (g_bufHead - g_bufCount + g_bufCapacity) % g_bufCapacity;
    char line[96];
    for (int i = 0; i < g_bufCount; i++) {
        int idx = (tail + i) % g_bufCapacity;
        const TrackPoint& p = g_buf[idx];
        int n = snprintf(line, sizeof(line), "%lu,%.8f,%.8f,%.3f,%u,%u\n",
                          (unsigned long)p.epochSec, p.lat, p.lon, (double)p.altHAE,
                          (unsigned)p.fixQuality, (unsigned)p.carrSoln);
        g_flashFile.write((const uint8_t*)line, n);
    }
    g_flashFile.flush();
    g_bufCount = 0;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
namespace TrackRecorder {

void begin(SdFat& sd, SemaphoreHandle_t sdMutex, bool& sdOK) {
    _sd      = &sd;
    _sdMutex = sdMutex;
    _sdOK    = &sdOK;
    if (!g_trackMutex) g_trackMutex = xSemaphoreCreateMutex();
    FlashConfig::mkdir(TRACKS_DIR);
    Serial.println("[TrackRecorder] Initialized");
}

bool start(const String& name) {
    if (g_status.recording) { setError("Already recording"); return false; }
    if (!allocateTrackBuffer()) { setError("Buffer allocation failed"); return false; }

    size_t used = FlashConfig::usedBytes(), total = FlashConfig::totalBytes();
    if (total > used && (total - used) < TRACK_MIN_FREE_BYTES) {
        setError("Insufficient flash space");
        return false;
    }

    String id = trackGenerateId();
    String path = String(TRACKS_DIR) + "/" + id + ".trk";
    g_flashFile = LittleFS.open(path, "w");
    if (!g_flashFile) { setError("Failed to create track file"); return false; }
    g_fileOpen = true;

    g_bufHead = 0; g_bufCount = 0; g_bufDropped = 0;
    g_lastSampleMs = 0; g_lastSampleLat = NAN; g_lastSampleLon = NAN;
    g_lastFlashFlushMs = millis();
    g_lastCheckpointMs = millis();
    g_startMs = millis();
    g_trackStartEpoch = (uint32_t)time(nullptr);
    g_trackName = name;

    if (xSemaphoreTake(g_trackMutex, portMAX_DELAY) == pdTRUE) {
        g_status.recording     = true;
        g_status.trackId       = id;
        g_status.pointCount    = 0;
        g_status.pointsDropped = 0;
        g_status.distanceM     = 0.0;
        g_status.durationSec   = 0;
        g_status.lastError     = "";
        xSemaphoreGive(g_trackMutex);
    }

    writeMetadata(id, false);
    Serial.printf("[TrackRecorder] Started track %s\n", id.c_str());
    return true;
}

bool stop() {
    if (!g_status.recording) return false;
    flushBufferToFlash();
    if (g_fileOpen) { g_flashFile.close(); g_fileOpen = false; }

    String id = g_status.trackId;
    if (xSemaphoreTake(g_trackMutex, portMAX_DELAY) == pdTRUE) {
        g_status.recording = false;
        xSemaphoreGive(g_trackMutex);
    }
    writeMetadata(id, true);
    periodicSync(true);   // final SD backup, best-effort
    Serial.printf("[TrackRecorder] Stopped track %s\n", id.c_str());
    return true;
}

bool isRecording() { return g_status.recording; }

void setTriggerMode(TrackTriggerMode mode) {
    if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_status.cfg.triggerMode = mode;
        xSemaphoreGive(g_trackMutex);
    }
    g_lastSampleMs = 0; g_lastSampleLat = NAN; g_lastSampleLon = NAN;
}

void setIntervalSec(float sec) {
    if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_status.cfg.intervalSec = sec;
        xSemaphoreGive(g_trackMutex);
    }
    g_lastSampleMs = 0;
}

void setDistanceM(float metres) {
    if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        g_status.cfg.distanceM = metres;
        xSemaphoreGive(g_trackMutex);
    }
    g_lastSampleLat = NAN; g_lastSampleLon = NAN;
}

TrackConfig getConfig() {
    TrackConfig cfg;
    if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        cfg = g_status.cfg;
        xSemaphoreGive(g_trackMutex);
    }
    return cfg;
}

TrackStatus getStatus() {
    TrackStatus st;
    if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
        st = g_status;
        xSemaphoreGive(g_trackMutex);
    }
    return st;
}

void backgroundTask(void* pv) {
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(SAMPLE_TICK_MS));

        bool recording; TrackConfig cfg;
        if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
            recording = g_status.recording;
            cfg       = g_status.cfg;
            xSemaphoreGive(g_trackMutex);
        } else continue;
        if (!recording || !positionProvider) continue;

        double lat, lon, h; uint8_t carrSoln, fixQuality;
        positionProvider(lat, lon, h, carrSoln, fixQuality);
        if (fixQuality == 0) continue;   // no fix at all — skip this tick

        uint32_t nowMs = millis();
        bool doSample;
        if (cfg.triggerMode == TRACK_TRIGGER_TIME) {
            doSample = (g_lastSampleMs == 0) ||
                       (nowMs - g_lastSampleMs >= (uint32_t)(cfg.intervalSec * 1000.0f));
        } else {
            doSample = isnan(g_lastSampleLat) ||
                       haversineMeters(g_lastSampleLat, g_lastSampleLon, lat, lon) >= cfg.distanceM;
        }

        if (doSample) {
            double deltaM = isnan(g_lastSampleLat) ? 0.0
                          : haversineMeters(g_lastSampleLat, g_lastSampleLon, lat, lon);
            TrackPoint p{ (uint32_t)time(nullptr), lat, lon, (float)h, fixQuality, carrSoln };
            bufPush(p);
            g_lastSampleMs = nowMs; g_lastSampleLat = lat; g_lastSampleLon = lon;

            if (xSemaphoreTake(g_trackMutex, pdMS_TO_TICKS(50)) == pdTRUE) {
                g_status.pointCount++;
                g_status.distanceM    += deltaM;
                g_status.pointsDropped = g_bufDropped;
                g_status.durationSec   = (millis() - g_startMs) / 1000;
                xSemaphoreGive(g_trackMutex);
            }
        }

        // ---- Flush to flash: batched, no mutex, cannot delay the raw logger ----
        bool enoughPoints = g_bufCount >= FLASH_FLUSH_MIN_POINTS;
        bool timeUp       = (nowMs - g_lastFlashFlushMs) >= FLASH_FLUSH_MAX_MS;
        if (g_bufCount > 0 && (enoughPoints || timeUp)) {
            flushBufferToFlash();
            g_lastFlashFlushMs = nowMs;
        }

        // ---- Metadata checkpoint (LittleFS only, infrequent) ----
        if (nowMs - g_lastCheckpointMs >= METADATA_CHECKPOINT_MS) {
            g_lastCheckpointMs = nowMs;
            writeMetadata(g_status.trackId, false);
        }
    }
}

bool listTracks(std::vector<TrackInfo>& out) {
    out.clear();
    File root = LittleFS.open(TRACKS_DIR);
    if (!root) return false;
    File f = root.openNextFile();
    while (f) {
        if (!f.isDirectory()) {
            String fname = f.name();
            if (fname.endsWith(".json")) {
                String id   = fname.substring(0, fname.length() - 5);
                String path = String(TRACKS_DIR) + "/" + fname;
                String json = FlashConfig::readFile(path.c_str());

                TrackInfo info;
                info.trackId = id;
                info.name = id; info.startEpoch = 0; info.endEpoch = 0;
                info.pointCount = 0; info.distanceM = 0.0;

                auto extractStr = [&](const char* key) -> String {
                    String nd = String("\"") + key + "\":\"";
                    int ki = json.indexOf(nd);
                    if (ki < 0) return "";
                    int vs = ki + nd.length();
                    int ve = json.indexOf("\"", vs);
                    return ve > vs ? json.substring(vs, ve) : "";
                };
                auto extractNum = [&](const char* key) -> double {
                    String nd = String("\"") + key + "\":";
                    int ki = json.indexOf(nd);
                    if (ki < 0) return 0;
                    int vs = ki + nd.length();
                    int ve = vs;
                    while (ve < (int)json.length() &&
                           (isdigit(json[ve]) || json[ve] == '.' || json[ve] == '-')) ve++;
                    return json.substring(vs, ve).toDouble();
                };

                String nm = extractStr("name");
                if (!nm.isEmpty()) info.name = nm;
                info.startEpoch = (uint32_t)extractNum("startEpoch");
                info.endEpoch   = (uint32_t)extractNum("endEpoch");
                info.pointCount = (uint32_t)extractNum("pointCount");
                info.distanceM  = extractNum("distanceM");

                out.push_back(info);
            }
        }
        f = root.openNextFile();
    }
    root.close();
    return true;
}

bool deleteTrack(const String& trackId) {
    if (trackId.isEmpty()) return false;
    if (g_status.recording && g_status.trackId == trackId) return false;  // stop it first
    String trkPath  = String(TRACKS_DIR) + "/" + trackId + ".trk";
    String jsonPath = String(TRACKS_DIR) + "/" + trackId + ".json";
    bool ok1 = FlashConfig::remove(trkPath.c_str());
    bool ok2 = FlashConfig::remove(jsonPath.c_str());
    return ok1 || ok2;
}

// ---- SD backup sync — mirrors SurveyPoints::syncToSD/periodicSync exactly ----
static void syncToSD() {
    if (!_sd || !_sdMutex || !_sdOK || !*_sdOK) return;
    if (xSemaphoreTake(_sdMutex, pdMS_TO_TICKS(200)) != pdTRUE) return;   // busy — skip, retry later

    _sd->mkdir(TRACKS_SD_DIR);
    File root = LittleFS.open(TRACKS_DIR);
    if (root) {
        File f = root.openNextFile();
        while (f) {
            if (!f.isDirectory()) {
                String name    = f.name();
                String sdPath  = String(TRACKS_SD_DIR) + "/" + name;
                String content = FlashConfig::readFile((String(TRACKS_DIR) + "/" + name).c_str());
                if (!content.isEmpty()) {
                    FsFile sdFile = _sd->open(sdPath.c_str(), O_WRONLY | O_CREAT | O_TRUNC);
                    if (sdFile) { sdFile.print(content); sdFile.close(); }
                }
            }
            f = root.openNextFile();
        }
        root.close();
    }

    xSemaphoreGive(_sdMutex);
}

void periodicSync(bool forceNow) {
    uint32_t now = millis();
    if (!forceNow && (now - g_lastSyncMs < SD_SYNC_INTERVAL_MS)) return;
    g_lastSyncMs = now;
    syncToSD();
}

// ---------------------------------------------------------------------------
// Export — streams the .trk CSV back, transformed into GPX/KML, never
// materializing the whole track in RAM.
// ---------------------------------------------------------------------------
static bool loadMeta(const String& trackId, TrackInfo& out) {
    std::vector<TrackInfo> all;
    if (!listTracks(all)) return false;
    for (auto& t : all) if (t.trackId == trackId) { out = t; return true; }
    return false;
}

static String isoUtc(uint32_t epochSec) {
    time_t t = (time_t)epochSec;
    struct tm tmv;
    gmtime_r(&t, &tmv);
    char buf[24];
    strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tmv);
    return String(buf);
}

bool exportGPX(const String& trackId, EmitFn emit, void* ctx) {
    TrackInfo info;
    if (!loadMeta(trackId, info)) return false;

    String path = String(TRACKS_DIR) + "/" + trackId + ".trk";
    File f = LittleFS.open(path, "r");
    if (!f) return false;

    String header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                     "<gpx version=\"1.1\" creator=\"RTKino\" xmlns=\"http://www.topografix.com/GPX/1/1\">\n"
                     "<trk><name>" + trackJsonEscape(info.name) + "</name><trkseg>\n";
    emit(header, ctx);

    const int BATCH_LINES = 100;
    int inBatch = 0;
    String batch;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) continue;

        int c1 = line.indexOf(','); if (c1 < 0) continue;
        int c2 = line.indexOf(',', c1 + 1); if (c2 < 0) continue;
        int c3 = line.indexOf(',', c2 + 1); if (c3 < 0) continue;

        uint32_t epochSec = (uint32_t)line.substring(0, c1).toInt();
        String   lat      = line.substring(c1 + 1, c2);
        String   lon      = line.substring(c2 + 1, c3);
        int c4 = line.indexOf(',', c3 + 1);
        String   ele      = (c4 > 0) ? line.substring(c3 + 1, c4) : line.substring(c3 + 1);

        batch += "<trkpt lat=\"" + lat + "\" lon=\"" + lon + "\"><ele>" + ele + "</ele><time>"
               + isoUtc(epochSec) + "</time></trkpt>\n";
        if (++inBatch >= BATCH_LINES) {
            emit(batch, ctx);
            batch = "";
            inBatch = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    if (!batch.isEmpty()) emit(batch, ctx);
    f.close();

    emit(String("</trkseg></trk>\n</gpx>\n"), ctx);
    return true;
}

bool exportKML(const String& trackId, EmitFn emit, void* ctx) {
    TrackInfo info;
    if (!loadMeta(trackId, info)) return false;

    String path = String(TRACKS_DIR) + "/" + trackId + ".trk";
    File f = LittleFS.open(path, "r");
    if (!f) return false;

    String header = "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n"
                     "<kml xmlns=\"http://www.opengis.net/kml/2.2\">\n<Document>\n"
                     "<name>" + trackJsonEscape(info.name) + "</name>\n"
                     "<Placemark><name>" + trackJsonEscape(info.name) + "</name>\n"
                     "<LineString><altitudeMode>absolute</altitudeMode>\n<coordinates>\n";
    emit(header, ctx);

    const int BATCH_LINES = 100;
    int inBatch = 0;
    String batch;
    while (f.available()) {
        String line = f.readStringUntil('\n');
        line.trim();
        if (line.isEmpty()) continue;

        int c1 = line.indexOf(','); if (c1 < 0) continue;
        int c2 = line.indexOf(',', c1 + 1); if (c2 < 0) continue;
        int c3 = line.indexOf(',', c2 + 1); if (c3 < 0) continue;

        String lat = line.substring(c1 + 1, c2);
        String lon = line.substring(c2 + 1, c3);
        int c4 = line.indexOf(',', c3 + 1);
        String ele = (c4 > 0) ? line.substring(c3 + 1, c4) : line.substring(c3 + 1);

        batch += lon + "," + lat + "," + ele + "\n";
        if (++inBatch >= BATCH_LINES) {
            emit(batch, ctx);
            batch = "";
            inBatch = 0;
            vTaskDelay(pdMS_TO_TICKS(1));
        }
    }
    if (!batch.isEmpty()) emit(batch, ctx);
    f.close();

    emit(String("</coordinates></LineString></Placemark>\n</Document>\n</kml>\n"), ctx);
    return true;
}

} // namespace TrackRecorder
