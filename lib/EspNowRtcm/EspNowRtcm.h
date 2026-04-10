// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#pragma once

#include <Arduino.h>
#include <functional>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

// ============================================================================
// ESP-NOW RTCM MESH — Packet protocol
// ============================================================================

#define ESPNOW_MAGIC    0x52544B4EUL  // "RTKN"
#define ESPNOW_VERSION  1

#define PKT_RTCM_FRAG   0x01  // RTCM fragment (continuous, broadcast)
#define PKT_TELEMETRY   0x02  // node telemetry (every 5s, broadcast)
#define PKT_COMMAND     0x03  // remote command (unicast)
#define PKT_ACK         0x04  // command response (unicast)

// Remote commands
#define CMD_REBOOT          0x01
#define CMD_ZED_RESET_HOT   0x02
#define CMD_ZED_RESET_COLD  0x03
#define CMD_LOG_START       0x04
#define CMD_LOG_STOP        0x05
#define CMD_BASE_STOP       0x06
#define CMD_ESPNOW_STOP     0x07
#define CMD_STATUS_REQ      0x08

// ---- Packet structures (packed, no padding) ----

struct __attribute__((packed)) EspNowRtcmPacket {
    uint32_t magic;
    uint8_t  version;
    uint8_t  pkt_type;        // PKT_RTCM_FRAG
    uint32_t network_id;
    uint16_t node_id;         // sender MAC low 2 bytes
    uint16_t seq;             // global stream sequence
    uint8_t  ttl;             // remaining hops
    uint8_t  hop_count;       // hops performed
    uint32_t packet_uid;      // dedup ID: seq ^ (node_id << 16)
    uint32_t timestamp_ms;    // millis() at generation
    int8_t   last_rssi;       // RSSI of last hop (0 at origin)
    uint8_t  frag_index;      // fragment index (0-based)
    uint8_t  frag_count;      // total fragments in this RTCM frame
    uint16_t payload_len;     // valid payload bytes
    uint8_t  payload[200];    // RTCM data
    uint16_t crc;             // CRC16 of entire packet except crc field
};

struct __attribute__((packed)) EspNowTelemPacket {
    uint32_t magic;
    uint8_t  version;
    uint8_t  pkt_type;        // PKT_TELEMETRY
    uint32_t network_id;
    uint16_t node_id;
    uint8_t  node_role;       // 0=rover, 1=base, 2=relay
    int32_t  lat_e7;
    int32_t  lon_e7;
    int16_t  alt_dm;          // altitude in decimetres
    uint8_t  fix_quality;
    uint8_t  carr_soln;       // 0=no RTK, 1=float, 2=fixed
    uint8_t  num_sv;
    uint16_t h_acc_mm;
    int8_t   last_rssi;
    uint8_t  pkt_loss_pct;
    uint16_t rtcm_age_ms;
    uint8_t  hop_count;
    uint16_t uptime_min;
    uint16_t free_heap_kb;
    uint32_t timestamp_ms;
    uint16_t crc;
};

struct __attribute__((packed)) EspNowCmdPacket {
    uint32_t magic;
    uint8_t  version;
    uint8_t  pkt_type;        // PKT_COMMAND
    uint32_t network_id;
    uint16_t src_node_id;
    uint16_t dst_node_id;     // 0xFFFF = broadcast
    uint32_t cmd_uid;
    uint8_t  command;
    uint8_t  param;
    uint16_t crc;
};

struct __attribute__((packed)) EspNowAckPacket {
    uint32_t magic;
    uint8_t  version;
    uint8_t  pkt_type;        // PKT_ACK
    uint32_t network_id;
    uint16_t src_node_id;
    uint32_t ack_cmd_uid;
    uint8_t  result;          // 0=OK, 1=FAIL, 2=UNKNOWN_CMD
    uint16_t crc;
};

// ============================================================================
// EspNowRtcm class
// ============================================================================

class EspNowRtcm {
public:
    // Public packet type alias for callbacks
    using TelemPacket = EspNowTelemPacket;

    // Callbacks set by main.cpp
    std::function<void(const uint8_t*, size_t)>              onRtcmReceived;
    std::function<void(const EspNowTelemPacket&)>            onTelemReceived;
    std::function<void(uint8_t cmd, uint8_t param, uint16_t src)> onCommandReceived;

    bool begin(uint8_t channel);
    void stop();

    // TX base: fragment raw RTCM buffer and broadcast via ESP-NOW
    // Fire and forget: drops if radio busy, never buffers
    void broadcastRtcm(const uint8_t* data, size_t len);

    // TX telemetry: build EspNowTelemPacket from current data and broadcast
    void sendTelemetry(uint8_t role,
                       int32_t lat_e7, int32_t lon_e7, int16_t alt_dm,
                       uint8_t fix, uint8_t carr, uint8_t sv, uint16_t hacc_mm,
                       int8_t rssi, uint8_t loss_pct, uint16_t rtcm_age_ms,
                       uint8_t hops, uint16_t uptime_min, uint16_t heap_kb);

    // TX command: send to dst_node_id (unicast via broadcast MAC if peer unknown)
    bool sendCommand(uint16_t dst_node_id, uint8_t cmd, uint8_t param = 0);

    // TX ack
    void sendAck(uint16_t dst_node_id, uint32_t cmd_uid, uint8_t result);

    // Statistics
    uint32_t getRxPkts()      const { return _rxPkts; }
    uint32_t getDropDedup()   const { return _dropDedup; }
    uint32_t getDropOld()     const { return _dropOld; }
    uint32_t getDropCrc()     const { return _dropCrc; }
    uint32_t getTxPkts()      const { return _txPkts; }
    uint32_t getRtcmBytesRx() const { return _rtcmBytesRx; }
    int8_t   getLastRssi()    const { return _lastRssi; }
    uint16_t getNodeId()      const { return _nodeId; }

    // Peer map (telemetry received from other nodes)
    struct PeerInfo {
        uint16_t node_id;
        uint8_t  role;
        int32_t  lat_e7, lon_e7;
        int16_t  alt_dm;
        uint8_t  fix_quality, carr_soln, num_sv;
        uint16_t h_acc_mm;
        int8_t   last_rssi;
        uint8_t  pkt_loss_pct;
        uint16_t rtcm_age_ms;
        uint32_t last_seen_ms;
    };
    static const int MAX_PEERS = 8;
    PeerInfo peers[MAX_PEERS];
    int peerCount = 0;
    PeerInfo* findOrAddPeer(uint16_t node_id);

private:
    bool _active = false;
    uint16_t _nodeId = 0;
    uint16_t _seq = 0;

    // Dedup cache: ring buffer of recent packet_uids
    static const int DEDUP_SIZE = 64;
    uint32_t _dedupCache[DEDUP_SIZE];
    int _dedupHead = 0;
    bool isDuplicate(uint32_t uid);
    void markSeen(uint32_t uid);

    // Pending command tracking (for ACK timeout)
    uint32_t _pendingCmdUid = 0;
    uint32_t _pendingCmdSentMs = 0;
    static const uint32_t CMD_ACK_TIMEOUT_MS = 2000;

    // Confirm pattern for destructive commands
    uint8_t  _pendingConfirmCmd = 0;
    uint32_t _pendingConfirmMs = 0;
    static const uint32_t CMD_CONFIRM_WINDOW_MS = 3000;

    // Stats
    uint32_t _rxPkts = 0, _dropDedup = 0, _dropOld = 0, _dropCrc = 0;
    uint32_t _txPkts = 0, _rtcmBytesRx = 0;
    int8_t   _lastRssi = 0;

    // Receive callback (static, called by IDF on Core 0)
    // Signature is version-dependent: IDF >= 5.0 uses esp_now_recv_info_t*
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    static void onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len);
#else
    static void onRecv(const uint8_t* mac_addr, const uint8_t* data, int len);
#endif
    static EspNowRtcm* _instance;

    // FreeRTOS RX queue (Core 0 → Core 1)
    static const int RX_QUEUE_SIZE = 8;
    struct RxItem {
        uint8_t data[250];
        int     len;
        int8_t  rssi;
    };
    QueueHandle_t _rxQueue = nullptr;

    // RX task on Core 1
    TaskHandle_t _rxTaskHandle = nullptr;
    static void rxTask(void* pv);
    void processPacket(const uint8_t* data, int len, int8_t rssi);

    // CRC16-CCITT
    static uint16_t crc16(const uint8_t* data, size_t len);
};