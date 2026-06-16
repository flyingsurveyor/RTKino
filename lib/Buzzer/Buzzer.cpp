// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#include "Buzzer.h"
#include <ArduinoJson.h>

// JSON document size for melody loading
#define BUZZER_JSON_BUFFER_SIZE 2048

// Predefined melodies
const std::vector<BuzzerTone> Buzzer::MELODY_RTK_FIXED = {
    {880, 100},   // A5 - short beep
    {0, 50},      // pause
    {1318, 150}   // E6 - longer ascending beep
};

const std::vector<BuzzerTone> Buzzer::MELODY_RTK_LOST = {
    {440, 150},   // A4 - low beep
    {0, 50},      // pause
    {392, 150},   // G4 - lower beep
    {0, 50},      // pause
    {349, 150}    // F4 - lowest beep
};

Buzzer::Buzzer(uint8_t gpio, uint8_t ledcChannel)
    : m_gpio(gpio)
    , m_ledcChannel(ledcChannel)
    , m_initialized(false)
    , m_enabled(true)
    , m_playing(false)
    , m_currentToneIndex(0)
    , m_toneStartTime(0)
{
}

Buzzer::~Buzzer() {
    end();
}

bool Buzzer::begin() {
    if (m_initialized) {
        return true;
    }
    
    // Configure LEDC for PWM tone generation.
    // A non-zero seed frequency is required by the ESP-IDF LEDC peripheral;
    // passing 0 Hz causes an IDF error log on serial at startup.
    ledcSetup(m_ledcChannel, 1000, 8);  // 1 kHz seed, 8-bit resolution
    ledcAttachPin(m_gpio, m_ledcChannel);
    ledcWrite(m_ledcChannel, 0);  // Start silent (0% duty cycle)
    
    m_initialized = true;
    return true;
}

void Buzzer::end() {
    if (!m_initialized) {
        return;
    }
    
    stop();
    ledcDetachPin(m_gpio);
    m_initialized = false;
}

void Buzzer::playEvent(BuzzerEvent event) {
    if (!m_enabled || !m_initialized) {
        return;
    }
    
    switch (event) {
        case BuzzerEvent::RTK_FIXED:
            playMelody(MELODY_RTK_FIXED);
            break;
        case BuzzerEvent::RTK_LOST:
            playMelody(MELODY_RTK_LOST);
            break;
        case BuzzerEvent::CUSTOM:
            if (!m_customMelody.empty()) playMelody(m_customMelody);
            break;
    }
}

void Buzzer::playMelody(const std::vector<BuzzerTone>& melody) {
    if (!m_enabled || !m_initialized || melody.empty()) {
        return;
    }
    
    stop();  // Stop any current playback
    
    m_currentMelody = melody;
    m_currentToneIndex = 0;
    m_toneStartTime = millis();
    m_playing = true;
    
    // Start first tone
    const BuzzerTone& tone = m_currentMelody[0];
    if (tone.freq > 0) {
        startTone(tone.freq);
    }
}

void Buzzer::stop() {
    if (!m_initialized) {
        return;
    }
    
    stopTone();
    m_playing = false;
    m_currentMelody.clear();
    m_currentToneIndex = 0;
}

void Buzzer::loop() {
    if (!m_playing || !m_initialized || m_currentMelody.empty()) {
        return;
    }
    
    uint32_t now = millis();
    const BuzzerTone& currentTone = m_currentMelody[m_currentToneIndex];
    
    // Check if current tone has finished
    if (now - m_toneStartTime >= currentTone.duration) {
        m_currentToneIndex++;
        
        // Check if melody is complete
        if (m_currentToneIndex >= m_currentMelody.size()) {
            stop();
            return;
        }
        
        // Start next tone
        const BuzzerTone& nextTone = m_currentMelody[m_currentToneIndex];
        m_toneStartTime = now;
        
        if (nextTone.freq > 0) {
            startTone(nextTone.freq);
        } else {
            stopTone();  // Silence/pause
        }
    }
}

void Buzzer::setEnabled(bool enabled) {
    m_enabled = enabled;
    if (!enabled && m_playing) {
        stop();
    }
}

bool Buzzer::loadMelodyFromJson(const String& json, std::vector<BuzzerTone>& melody) {
    DynamicJsonDocument doc(BUZZER_JSON_BUFFER_SIZE);
    DeserializationError error = deserializeJson(doc, json);
    
    if (error) {
        Serial.printf("[Buzzer] JSON parse error: %s\n", error.c_str());
        return false;
    }
    
    if (!doc.containsKey("tones")) {
        Serial.println("[Buzzer] JSON missing 'tones' array");
        return false;
    }
    
    JsonArray tones = doc["tones"].as<JsonArray>();
    if (tones.size() == 0) {
        Serial.println("[Buzzer] Empty tones array");
        return false;
    }
    
    melody.clear();
    for (JsonObject tone : tones) {
        if (!tone.containsKey("freq") || !tone.containsKey("duration")) {
            Serial.println("[Buzzer] Tone missing freq or duration");
            return false;
        }
        
        BuzzerTone t;
        t.freq = tone["freq"].as<uint16_t>();
        t.duration = tone["duration"].as<uint16_t>();
        melody.push_back(t);
    }
    
    return true;
}

void Buzzer::setSD(SdFat* sd, SemaphoreHandle_t sdMutex) {
    m_sd      = sd;
    m_sdMutex = sdMutex;
}

bool Buzzer::setCustomMelody(const String& jsonContent) {
    std::vector<BuzzerTone> melody;
    if (!loadMelodyFromJson(jsonContent, melody)) return false;
    m_customMelody = melody;
    Serial.printf("[Buzzer] Custom melody set: %d tones\n", (int)m_customMelody.size());
    return true;
}

bool Buzzer::loadCustomMelody(const char* path) {
    if (!m_sd || !m_sdMutex) {
        Serial.println("[Buzzer] loadCustomMelody: SD not available");
        return false;
    }
    bool locked = (xSemaphoreTake(m_sdMutex, pdMS_TO_TICKS(2000)) == pdTRUE);
    if (!locked) {
        Serial.println("[Buzzer] loadCustomMelody: SD busy");
        return false;
    }
    FsFile f = m_sd->open(path, O_RDONLY);
    if (!f) {
        xSemaphoreGive(m_sdMutex);
        Serial.printf("[Buzzer] loadCustomMelody: file not found: %s\n", path);
        return false;
    }
    String content;
    content.reserve(f.size());
    while (f.available()) content += (char)f.read();
    f.close();
    xSemaphoreGive(m_sdMutex);
    return setCustomMelody(content);
}

void Buzzer::startTone(uint16_t freq) {
    if (!m_initialized || freq == 0) {
        return;
    }
    
    // Change frequency on the already-attached channel (no re-attach needed)
    ledcSetup(m_ledcChannel, freq, 8);
    ledcWrite(m_ledcChannel, 128);  // 50% duty cycle
}

void Buzzer::stopTone() {
    if (!m_initialized) {
        return;
    }
    
    ledcWrite(m_ledcChannel, 0);  // 0% duty cycle = silence
}
