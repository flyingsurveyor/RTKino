// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "EspNowRtcm.h"
#include "config.h"

// Broadcast MAC address (used for all ESP-NOW sends)
static const uint8_t ESPNOW_BROADCAST_MAC[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

// ---- Singleton instance (needed for static callback) ----
EspNowRtcm* EspNowRtcm::_instance = nullptr;

// ============================================================================
// CRC16-CCITT (polynomial 0x1021, init 0xFFFF)
// ============================================================================
uint16_t EspNowRtcm::crc16(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; j++)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

// ============================================================================
// Dedup helpers
// ============================================================================
bool EspNowRtcm::isDuplicate(uint32_t uid) {
    for (int i = 0; i < DEDUP_SIZE; i++) {
        if (_dedupCache[i] == uid) return true;
    }
    return false;
}

void EspNowRtcm::markSeen(uint32_t uid) {
    _dedupCache[_dedupHead] = uid;
    _dedupHead = (_dedupHead + 1) % DEDUP_SIZE;
}

// ============================================================================
// Peer map
// ============================================================================
EspNowRtcm::PeerInfo* EspNowRtcm::findOrAddPeer(uint16_t node_id) {
    for (int i = 0; i < peerCount; i++) {
        if (peers[i].node_id == node_id) return &peers[i];
    }
    if (peerCount < MAX_PEERS) {
        PeerInfo& p = peers[peerCount++];
        memset(&p, 0, sizeof(p));
        p.node_id = node_id;
        return &p;
    }
    // Evict oldest (lowest last_seen_ms)
    int oldest = 0;
    for (int i = 1; i < peerCount; i++) {
        if (peers[i].last_seen_ms < peers[oldest].last_seen_ms) oldest = i;
    }
    memset(&peers[oldest], 0, sizeof(peers[oldest]));
    peers[oldest].node_id = node_id;
    return &peers[oldest];
}

// ============================================================================
// begin() — initialise ESP-NOW
// ============================================================================
bool EspNowRtcm::begin(uint8_t channel) {
    if (_active) return true;

    _instance = this;
    memset(_dedupCache, 0, sizeof(_dedupCache));
    _dedupHead = 0;
    peerCount  = 0;

    // Read MAC and compute node_id (low 2 bytes of AP MAC)
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    _nodeId = ((uint16_t)mac[4] << 8) | mac[5];

    // Init ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("[ESPNOW] esp_now_init failed");
        return false;
    }
    esp_now_register_recv_cb(onRecv);

    // Add broadcast peer
    esp_now_peer_info_t peer = {};
    memset(peer.peer_addr, 0xFF, 6);
    peer.channel = channel;
    peer.encrypt = false;
    esp_now_add_peer(&peer);

    // Create RX queue and task on Core 1
    _rxQueue = xQueueCreate(RX_QUEUE_SIZE, sizeof(RxItem));
    if (!_rxQueue) {
        Serial.println("[ESPNOW] Failed to create RX queue");
        esp_now_deinit();
        return false;
    }
    xTaskCreatePinnedToCore(rxTask, "espnow_rx", 4096, this, 1, &_rxTaskHandle, 1);

    _active = true;
    Serial.printf("[ESPNOW] Started, node_id=0x%04X, ch=%u\n", _nodeId, channel);
    return true;
}

// ============================================================================
// stop()
// ============================================================================
void EspNowRtcm::stop() {
    if (!_active) return;
    _active = false;

    if (_rxTaskHandle) {
        vTaskDelete(_rxTaskHandle);
        _rxTaskHandle = nullptr;
    }
    if (_rxQueue) {
        vQueueDelete(_rxQueue);
        _rxQueue = nullptr;
    }
    esp_now_deinit();
    _instance = nullptr;
    Serial.println("[ESPNOW] Stopped");
}

// ============================================================================
// Static RX callback — Core 0, ISR context; must be minimal
// ============================================================================
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
void EspNowRtcm::onRecv(const esp_now_recv_info_t* info, const uint8_t* data, int len) {
    if (!_instance || !_instance->_rxQueue) return;
    if (len <= 0 || len > 250) return;

    RxItem item;
    memcpy(item.data, data, len);
    item.len  = len;
    item.rssi = info->rx_ctrl ? (int8_t)info->rx_ctrl->rssi : 0;

    BaseType_t woken = pdFALSE;
    xQueueSendToBackFromISR(_instance->_rxQueue, &item, &woken);
    if (woken) portYIELD_FROM_ISR();
}
#else
void EspNowRtcm::onRecv(const uint8_t* /*mac_addr*/, const uint8_t* data, int len) {
    if (!_instance || !_instance->_rxQueue) return;
    if (len <= 0 || len > 250) return;

    RxItem item;
    memcpy(item.data, data, len);
    item.len  = len;
    item.rssi = 0;  // RSSI not available in IDF 4.x callback signature

    BaseType_t woken = pdFALSE;
    xQueueSendToBackFromISR(_instance->_rxQueue, &item, &woken);
    if (woken) portYIELD_FROM_ISR();
}
#endif

// ============================================================================
// rxTask — Core 1, picks from queue and calls processPacket
// ============================================================================
void EspNowRtcm::rxTask(void* pv) {
    EspNowRtcm* self = static_cast<EspNowRtcm*>(pv);
    RxItem item;
    while (self->_active) {
        if (xQueueReceive(self->_rxQueue, &item, pdMS_TO_TICKS(20)) == pdTRUE) {
            self->processPacket(item.data, item.len, item.rssi);
        }
        // Yield to allow loop() and other tasks to run
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    vTaskDelete(nullptr);
}

// ============================================================================
// processPacket — validate and dispatch
// ============================================================================
void EspNowRtcm::processPacket(const uint8_t* data, int len, int8_t rssi) {
    if (len < 8) return;

    // Peek at magic + version (common header prefix for all packet types)
    uint32_t magic;
    memcpy(&magic, data, 4);
    if (magic != ESPNOW_MAGIC) return;
    if (data[4] != ESPNOW_VERSION) return;

    uint8_t pkt_type = data[5];

    // Network ID check (offset 6 for all packet types)
    uint32_t net_id;
    memcpy(&net_id, data + 6, 4);
    if (net_id != _networkId) return;

    _rxPkts++;

    switch (pkt_type) {

    // ------------------------------------------------------------------
    case PKT_RTCM_FRAG: {
        if (len < (int)sizeof(EspNowRtcmPacket)) return;
        const EspNowRtcmPacket* pkt = reinterpret_cast<const EspNowRtcmPacket*>(data);

        // CRC check (over everything except the last 2 bytes)
        uint16_t calc = crc16(data, sizeof(EspNowRtcmPacket) - 2);
        if (calc != pkt->crc) { _dropCrc++; return; }

        // Dedup
        if (isDuplicate(pkt->packet_uid)) { _dropDedup++; return; }

        // Age check
        uint32_t nowMs = (uint32_t)millis();
        if (nowMs > pkt->timestamp_ms && (nowMs - pkt->timestamp_ms) > ESPNOW_MAX_AGE_MS) {
            _dropOld++;
            return;
        }

        markSeen(pkt->packet_uid);
        _lastRssi = rssi;
        _rtcmBytesRx += pkt->payload_len;

        if (onRtcmReceived && pkt->payload_len > 0) {
            onRtcmReceived(pkt->payload, pkt->payload_len);
        }
        break;
    }

    // ------------------------------------------------------------------
    case PKT_TELEMETRY: {
        if (len < (int)sizeof(EspNowTelemPacket)) return;
        const EspNowTelemPacket* pkt = reinterpret_cast<const EspNowTelemPacket*>(data);

        uint16_t calc = computeAuth(data, sizeof(EspNowTelemPacket) - 2);
        if (calc != pkt->crc) { _dropAuth++; return; }  // crc field doubles as auth tag when PSK enabled

        PeerInfo* peer = findOrAddPeer(pkt->node_id);
        if (peer) {
            peer->role          = pkt->node_role;
            peer->lat_e7        = pkt->lat_e7;
            peer->lon_e7        = pkt->lon_e7;
            peer->alt_dm        = pkt->alt_dm;
            peer->fix_quality   = pkt->fix_quality;
            peer->carr_soln     = pkt->carr_soln;
            peer->num_sv        = pkt->num_sv;
            peer->h_acc_mm      = pkt->h_acc_mm;
            peer->last_rssi     = rssi;
            peer->pkt_loss_pct  = pkt->pkt_loss_pct;
            peer->rtcm_age_ms   = pkt->rtcm_age_ms;
            peer->last_seen_ms  = (uint32_t)millis();
            peer->upstream_node_id  = pkt->upstream_node_id;
            peer->relay_for_node_id = pkt->relay_for_node_id;
        }

        if (onTelemReceived) onTelemReceived(*pkt);
        break;
    }

    // ------------------------------------------------------------------
    case PKT_COMMAND: {
        if (len < (int)sizeof(EspNowCmdPacket)) return;
        const EspNowCmdPacket* pkt = reinterpret_cast<const EspNowCmdPacket*>(data);

        uint16_t calc = computeAuth(data, sizeof(EspNowCmdPacket) - 2);
        if (calc != pkt->crc) { _dropAuth++; return; }

        // Check destination
        if (pkt->dst_node_id != _nodeId && pkt->dst_node_id != 0xFFFF) return;

        // Whitelist: only accept commands from known peers (seen via telemetry)
        bool knownSrc = false;
        for (int i = 0; i < peerCount; i++) {
            if (peers[i].node_id == pkt->src_node_id) { knownSrc = true; break; }
        }
        if (!knownSrc && pkt->dst_node_id != 0xFFFF) {
            Serial.printf("[ESPNOW] CMD from unknown peer 0x%04X — ignored\n", pkt->src_node_id);
            return;
        }

        // Confirm pattern for destructive commands
        if (pkt->command == CMD_REBOOT || pkt->command == CMD_ZED_RESET_COLD) {
            uint32_t nowMs = (uint32_t)millis();
            if (_pendingConfirmCmd == pkt->command &&
                (nowMs - _pendingConfirmMs) < CMD_CONFIRM_WINDOW_MS) {
                // Second identical command within window → execute
                _pendingConfirmCmd = 0;
            } else {
                // First time → prime confirm, send FAIL ack
                _pendingConfirmCmd = pkt->command;
                _pendingConfirmMs  = nowMs;
                sendAck(pkt->src_node_id, pkt->cmd_uid, 1 /*FAIL — needs confirm*/);
                Serial.printf("[ESPNOW] CMD 0x%02X needs confirm (send again within 3s)\n", pkt->command);
                return;
            }
        }

        sendAck(pkt->src_node_id, pkt->cmd_uid, 0 /*OK*/);

        if (onCommandReceived) {
            onCommandReceived(pkt->command, pkt->param, pkt->src_node_id);
        }
        break;
    }

    // ------------------------------------------------------------------
    case PKT_ACK: {
        if (len < (int)sizeof(EspNowAckPacket)) return;
        const EspNowAckPacket* pkt = reinterpret_cast<const EspNowAckPacket*>(data);

        uint16_t calc = computeAuth(data, sizeof(EspNowAckPacket) - 2);
        if (calc != pkt->crc) { _dropAuth++; return; }

        if (pkt->ack_cmd_uid == _pendingCmdUid) {
            Serial.printf("[ESPNOW] ACK received for cmd_uid=0x%08X result=%u\n",
                          pkt->ack_cmd_uid, pkt->result);
            _pendingCmdUid = 0;
        }
        break;
    }

    default:
        break;
    }
}

// ============================================================================
// broadcastRtcm — fire and forget fragmentation
// ============================================================================
void EspNowRtcm::broadcastRtcm(const uint8_t* data, size_t len) {
    if (!_active || !data || len == 0) return;

    const size_t payloadSize = ESPNOW_PAYLOAD_SIZE;
    uint8_t fragCount = (uint8_t)((len + payloadSize - 1) / payloadSize);
    if (fragCount == 0) return;

    uint32_t ts = (uint32_t)millis();
    uint32_t uid = (uint32_t)_seq ^ ((uint32_t)_nodeId << 16);

    for (uint8_t fi = 0; fi < fragCount; fi++) {
        EspNowRtcmPacket pkt = {};
        pkt.magic        = ESPNOW_MAGIC;
        pkt.version      = ESPNOW_VERSION;
        pkt.pkt_type     = PKT_RTCM_FRAG;
        pkt.network_id   = _networkId;
        pkt.node_id      = _nodeId;
        pkt.seq          = _seq;
        pkt.ttl          = ESPNOW_TTL;
        pkt.hop_count    = 0;
        pkt.packet_uid   = uid;
        pkt.timestamp_ms = ts;
        pkt.last_rssi    = 0;
        pkt.frag_index   = fi;
        pkt.frag_count   = fragCount;

        size_t offset = fi * payloadSize;
        size_t chunk  = (offset + payloadSize > len) ? (len - offset) : payloadSize;
        pkt.payload_len = (uint16_t)chunk;
        memcpy(pkt.payload, data + offset, chunk);

        // Compute CRC over all fields except the last 2 bytes (crc itself)
        pkt.crc = crc16((const uint8_t*)&pkt, sizeof(pkt) - 2);

        esp_err_t err = esp_now_send(ESPNOW_BROADCAST_MAC, (const uint8_t*)&pkt, sizeof(pkt));
        if (err == ESP_OK) {
            _txPkts++;
        } else if (err == ESP_ERR_ESPNOW_NO_MEM) {
            _dropNoMem++;
        }
        // Duplicate each fragment: the receiver dedup cache handles the duplicate silently.
        // Halves the probability of a fragment loss due to transient radio contention (WiFi beacon, etc.).
        vTaskDelay(pdMS_TO_TICKS(3));
        esp_now_send(ESPNOW_BROADCAST_MAC, (const uint8_t*)&pkt, sizeof(pkt));
    }

    _seq++;
}

// ============================================================================
// sendTelemetry
// ============================================================================
void EspNowRtcm::sendTelemetry(uint8_t role,
                                int32_t lat_e7, int32_t lon_e7, int16_t alt_dm,
                                uint8_t fix, uint8_t carr, uint8_t sv, uint16_t hacc_mm,
                                int8_t rssi, uint8_t loss_pct, uint16_t rtcm_age_ms,
                                uint8_t hops, uint16_t uptime_min, uint16_t heap_kb) {
    if (!_active) return;

    EspNowTelemPacket pkt = {};
    pkt.magic         = ESPNOW_MAGIC;
    pkt.version       = ESPNOW_VERSION;
    pkt.pkt_type      = PKT_TELEMETRY;
    pkt.network_id    = _networkId;
    pkt.node_id       = _nodeId;
    pkt.node_role     = role;
    pkt.lat_e7        = lat_e7;
    pkt.lon_e7        = lon_e7;
    pkt.alt_dm        = alt_dm;
    pkt.fix_quality   = fix;
    pkt.carr_soln     = carr;
    pkt.num_sv        = sv;
    pkt.h_acc_mm      = hacc_mm;
    pkt.last_rssi     = rssi;
    pkt.pkt_loss_pct  = loss_pct;
    pkt.rtcm_age_ms   = rtcm_age_ms;
    pkt.hop_count     = hops;
    pkt.uptime_min    = uptime_min;
    pkt.free_heap_kb  = heap_kb;
    pkt.upstream_node_id  = 0;
    pkt.relay_for_node_id = 0;
    pkt.timestamp_ms  = (uint32_t)millis();
    pkt.crc           = computeAuth((const uint8_t*)&pkt, sizeof(pkt) - 2);

    if (esp_now_send(ESPNOW_BROADCAST_MAC, (const uint8_t*)&pkt, sizeof(pkt)) == ESP_OK) {
        _txPkts++;
    }
}

// ============================================================================
// configure — set runtime network_id and PSK
// ============================================================================
void EspNowRtcm::configure(uint32_t network_id, const char* psk) {
    _networkId = network_id;
    if (psk && psk[0] != '\0') {
        if (strlen(psk) >= sizeof(_psk)) {
            Serial.printf("[ESPNOW] WARNING: PSK truncated to %u chars\n", (unsigned)(sizeof(_psk) - 1));
        }
        strncpy(_psk, psk, sizeof(_psk) - 1);
        _psk[sizeof(_psk) - 1] = '\0';
        _pskEnabled = true;
    } else {
        memset(_psk, 0, sizeof(_psk));
        _pskEnabled = false;
    }
}

// ============================================================================
// computeHmac4 — HMAC-SHA256 keyed by _psk; returns first 4 bytes big-endian
// ============================================================================
uint32_t EspNowRtcm::computeHmac4(const uint8_t* data, size_t len) const {
    uint8_t digest[32];
    mbedtls_md_hmac(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256),
                    (const uint8_t*)_psk, strlen(_psk),
                    data, len, digest);
    return ((uint32_t)digest[0] << 24) |
           ((uint32_t)digest[1] << 16) |
           ((uint32_t)digest[2] <<  8) |
            (uint32_t)digest[3];
}

// ============================================================================
// computeAuth — returns auth field: HMAC low-2 bytes if PSK enabled, else CRC16
// ============================================================================
uint16_t EspNowRtcm::computeAuth(const uint8_t* data, size_t len) const {
    if (_pskEnabled) {
        return (uint16_t)(computeHmac4(data, len) & 0xFFFF);
    }
    return crc16(data, len);
}

// ============================================================================
// sendCommand
// ============================================================================
bool EspNowRtcm::sendCommand(uint16_t dst_node_id, uint8_t cmd, uint8_t param) {
    if (!_active) return false;

    EspNowCmdPacket pkt = {};
    pkt.magic       = ESPNOW_MAGIC;
    pkt.version     = ESPNOW_VERSION;
    pkt.pkt_type    = PKT_COMMAND;
    pkt.network_id  = _networkId;
    pkt.src_node_id = _nodeId;
    pkt.dst_node_id = dst_node_id;
    pkt.cmd_uid     = (uint32_t)millis() ^ ((uint32_t)_nodeId << 16);
    pkt.command     = cmd;
    pkt.param       = param;
    pkt.crc         = computeAuth((const uint8_t*)&pkt, sizeof(pkt) - 2);

    _pendingCmdUid    = pkt.cmd_uid;
    _pendingCmdSentMs = (uint32_t)millis();

    if (esp_now_send(ESPNOW_BROADCAST_MAC, (const uint8_t*)&pkt, sizeof(pkt)) == ESP_OK) {
        _txPkts++;
        return true;
    }
    return false;
}

// ============================================================================
// sendAck
// ============================================================================
void EspNowRtcm::sendAck(uint16_t dst_node_id, uint32_t cmd_uid, uint8_t result) {
    if (!_active) return;
    // dst_node_id is kept in the signature for API symmetry with sendCommand()
    // and for future unicast support once per-peer MAC tracking is added.
    // Currently ACKs are sent via broadcast MAC (all registered peers receive them).
    (void)dst_node_id;

    EspNowAckPacket pkt = {};
    pkt.magic       = ESPNOW_MAGIC;
    pkt.version     = ESPNOW_VERSION;
    pkt.pkt_type    = PKT_ACK;
    pkt.network_id  = _networkId;
    pkt.src_node_id = _nodeId;
    pkt.ack_cmd_uid = cmd_uid;
    pkt.result      = result;
    pkt.crc         = computeAuth((const uint8_t*)&pkt, sizeof(pkt) - 2);

    if (esp_now_send(ESPNOW_BROADCAST_MAC, (const uint8_t*)&pkt, sizeof(pkt)) == ESP_OK) {
        _txPkts++;
    }
}
