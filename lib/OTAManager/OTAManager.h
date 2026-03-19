// SPDX-License-Identifier: AGPL-3.0-or-later
// Copyright (C) 2025-2026 FlyingSurveyor
#ifndef OTAMANAGER_H
#define OTAMANAGER_H

#include <Arduino.h>
#include <Update.h>
#include <esp_ota_ops.h>

/**
 * OTAManager - Gestione aggiornamenti OTA via Web Upload
 * 
 * Supporta aggiornamento firmware tramite upload da browser web.
 * 
 * Features:
 * - Rollback automatico in caso di errori
 * - Verifica MD5 del firmware
 * - Progress callback per feedback utente
 * - Gestione sicura delle partizioni OTA
 */

class OTAManager {
public:
    // Callback per il progresso dell'aggiornamento (0-100)
    typedef std::function<void(size_t progress, size_t total)> ProgressCallback;
    
    // Callback per gli errori
    typedef std::function<void(const char* error)> ErrorCallback;
    
    /**
     * Inizializza l'OTA Manager
     * @return true se inizializzazione OK
     */
    static bool begin();
    
    /**
     * Imposta la callback per il progresso
     */
    static void setProgressCallback(ProgressCallback cb);
    
    /**
     * Imposta la callback per gli errori
     */
    static void setErrorCallback(ErrorCallback cb);
    
    /**
     * Inizia l'aggiornamento da web upload
     * @param updateSize Dimensione totale del firmware in bytes
     * @return true se inizializzazione OK
     */
    static bool beginWebUpdate(size_t updateSize);
    
    /**
     * Scrive un chunk di dati durante il web update
     * @param data Puntatore ai dati
     * @param len Lunghezza dei dati
     * @return true se scrittura OK
     */
    static bool writeWebUpdate(uint8_t* data, size_t len);
    
    /**
     * Finalizza il web update
     * @return true se update completato con successo
     */
    static bool endWebUpdate();
    
    /**
     * Ottieni informazioni sulla partizione corrente
     * @return Stringa con info sulla partizione (es: "app0 @ 0x10000")
     */
    static String getCurrentPartitionInfo();
    
    /**
     * Ottieni informazioni sulla prossima partizione OTA
     * @return Stringa con info sulla prossima partizione
     */
    static String getNextPartitionInfo();
    
    /**
     * Verifica se c'è abbastanza spazio per l'aggiornamento
     * @param updateSize Dimensione del nuovo firmware
     * @return true se c'è spazio sufficiente
     */
    static bool hasSpaceForUpdate(size_t updateSize);
    
    /**
     * Ottieni lo stato dell'ultimo aggiornamento
     * @return Stringa descrittiva dello stato
     */
    static String getLastUpdateStatus();
    
    /**
     * Riavvia il dispositivo
     */
    static void reboot();

private:
    static ProgressCallback _progressCallback;
    static ErrorCallback _errorCallback;
    static bool _initialized;
    static String _lastUpdateStatus;
    static size_t _updateProgress;
    static size_t _updateTotal;
};

#endif // OTAMANAGER_H
