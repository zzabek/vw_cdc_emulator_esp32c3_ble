// ============================================================================
//  btn_map.cpp — Dynamic Command Mapping, Learn Mode & Keepalive NVS Store
// ============================================================================

#include "btn_map.h"
#include "ble_hid.h"
#include "config.h"

#include <Preferences.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

static constexpr const char* NVS_NS_CONFIG   = "cdccfg";
static constexpr const char* NVS_KEY_BLOB    = "cfg";
static constexpr const char* NVS_LEGACY_NS_1 = "btncfg";
static constexpr const char* NVS_LEGACY_NS_2 = "btncfg2";

static constexpr uint32_t CFG_MAGIC          = 0x43444334UL; // "CDC4"
static constexpr uint16_t CFG_VERSION        = 2;

static constexpr uint32_t KEEPALIVE_MIN_MS   = 1000U;
static constexpr uint32_t KEEPALIVE_MAX_MS   = 60000U;

struct PersistedConfig {
    uint32_t        magic;
    uint16_t        version;
    uint8_t         reserved;
    uint8_t         entryCount;
    KeepaliveConfig keepalive;
    CmdEntry        entries[CMD_MAP_MAX_ENTRIES];
};

static SemaphoreHandle_t s_mapMutex = nullptr;
static SemaphoreHandle_t s_nvsMutex = nullptr;

static PersistedConfig s_cfg    = {};
static LastRxInfo      s_lastRx = {};
static volatile bool   s_dirty  = false;

// ============================================================================
//  INTERNAL HELPERS
// ============================================================================

static void sanitizeEntry(CmdEntry& entry) {
    // Ensure null-terminated safe name (strip control characters)
    entry.name[CMD_NAME_MAX_LEN - 1] = '\0';
    for (size_t i = 0; i < CMD_NAME_MAX_LEN - 1 && entry.name[i] != '\0'; i++) {
        uint8_t ch = (uint8_t)entry.name[i];
        if (ch < 0x20 || ch == 0x7F) {
            entry.name[i] = '_';
        }
    }
    if (entry.name[0] == '\0') {
        snprintf(entry.name, sizeof(entry.name), "CMD_0x%02X", entry.cmdCode);
    }

    if (!ble_hid_is_valid_code(entry.hidCode, true)) {
        entry.hidCode = 0x0000;
    }

    if ((uint8_t)entry.radioAction > (uint8_t)RadioAction::RANDOM_TOGGLE) {
        entry.radioAction = RadioAction::NONE;
    }

    if (entry.radioAction == RadioAction::DISC_STICKY ||
        entry.radioAction == RadioAction::DISC_MOMENTARY) {
        if (entry.actionParam < 1 || entry.actionParam > 6) {
            entry.actionParam = 1;
        }
    } else {
        entry.actionParam = 0;
    }
}

static KeepaliveConfig sanitizeKeepalive(KeepaliveConfig cfg) {
    if (cfg.intervalMs < KEEPALIVE_MIN_MS) cfg.intervalMs = KEEPALIVE_MIN_MS;
    if (cfg.intervalMs > KEEPALIVE_MAX_MS) cfg.intervalMs = KEEPALIVE_MAX_MS;
    if (!ble_hid_is_valid_code(cfg.hidCode, !cfg.enabled)) {
        cfg.hidCode = KEEPALIVE_DEFAULT_HID_CODE;
    }
    return cfg;
}

static void populateDefaultConfigLocked() {
    memset(&s_cfg, 0, sizeof(s_cfg));
    s_cfg.magic      = CFG_MAGIC;
    s_cfg.version    = CFG_VERSION;
    s_cfg.entryCount = 0; // Start empty; unknown packets are captured automatically
    s_cfg.keepalive  = sanitizeKeepalive({
        KEEPALIVE_DEFAULT_ENABLED,
        KEEPALIVE_DEFAULT_INTERVAL_MS,
        KEEPALIVE_DEFAULT_HID_CODE
    });
}

static int findEntryIndexLocked(uint8_t cmdCode) {
    for (size_t i = 0; i < s_cfg.entryCount; i++) {
        if (s_cfg.entries[i].cmdCode == cmdCode) {
            return (int)i;
        }
    }
    return -1;
}

static void appendJsonEscaped(String& out, const char* text) {
    for (size_t i = 0; text[i] != '\0'; i++) {
        char ch = text[i];
        if (ch == '"' || ch == '\\') {
            out += '\\';
            out += ch;
        } else if ((uint8_t)ch >= 0x20) {
            out += ch;
        }
    }
}

static void clearLegacyNamespace(const char* ns) {
    Preferences oldPref;
    if (oldPref.begin(ns, false)) {
        oldPref.clear();
        oldPref.end();
    }
}

// ============================================================================
//  PUBLIC API
// ============================================================================

void btnMap_init() {
    if (!s_mapMutex) s_mapMutex = xSemaphoreCreateMutex();
    if (!s_nvsMutex) s_nvsMutex = xSemaphoreCreateMutex();

    bool loadedFromNvs = false;
    PersistedConfig loaded = {};

    if (s_nvsMutex) xSemaphoreTake(s_nvsMutex, portMAX_DELAY);
    Preferences pref;
    if (pref.begin(NVS_NS_CONFIG, true)) {
        size_t len = pref.getBytesLength(NVS_KEY_BLOB);
        if (len == sizeof(PersistedConfig)) {
            size_t readLen = pref.getBytes(NVS_KEY_BLOB, &loaded, sizeof(PersistedConfig));
            if (readLen == sizeof(PersistedConfig) &&
                loaded.magic == CFG_MAGIC &&
                loaded.version == CFG_VERSION &&
                loaded.entryCount <= CMD_MAP_MAX_ENTRIES) {
                loadedFromNvs = true;
            }
        }
        pref.end();
    }

    if (!loadedFromNvs) {
        clearLegacyNamespace(NVS_LEGACY_NS_1);
        clearLegacyNamespace(NVS_LEGACY_NS_2);
    }
    if (s_nvsMutex) xSemaphoreGive(s_nvsMutex);

    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    if (loadedFromNvs) {
        s_cfg = loaded;
        s_cfg.keepalive = sanitizeKeepalive(s_cfg.keepalive);
        for (size_t i = 0; i < s_cfg.entryCount; i++) {
            sanitizeEntry(s_cfg.entries[i]);
        }
        s_dirty = false;
    } else {
        populateDefaultConfigLocked();
        s_dirty = true;
    }
    memset(&s_lastRx, 0, sizeof(s_lastRx));
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);

    Serial.printf("[BTNM] Loaded %u button mappings (%s)\n",
                  (unsigned)s_cfg.entryCount,
                  loadedFromNvs ? "NVS" : "empty / auto-learn");
}

bool btnMap_isDirty() {
    return s_dirty;
}

bool btnMap_saveIfDirty() {
    if (!s_dirty) return true;

    PersistedConfig snapshot = {};
    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    if (!s_dirty) {
        if (s_mapMutex) xSemaphoreGive(s_mapMutex);
        return true;
    }
    snapshot = s_cfg;
    s_dirty = false;
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);

    bool ok = false;
    if (s_nvsMutex) xSemaphoreTake(s_nvsMutex, portMAX_DELAY);
    Preferences pref;
    if (pref.begin(NVS_NS_CONFIG, false)) {
        size_t written = pref.putBytes(NVS_KEY_BLOB, &snapshot, sizeof(PersistedConfig));
        pref.end();
        ok = (written == sizeof(PersistedConfig));
    }
    if (s_nvsMutex) xSemaphoreGive(s_nvsMutex);

    if (!ok) {
        if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
        s_dirty = true;
        if (s_mapMutex) xSemaphoreGive(s_mapMutex);
        Serial.println("[BTNM] ERROR: Failed to commit config to NVS");
    } else {
        Serial.printf("[BTNM] Saved %u mappings + config to NVS\n",
                      (unsigned)snapshot.entryCount);
    }
    return ok;
}

bool btnMap_handleRxCommand(uint8_t cmdCode, CmdEntry& outEntry, bool& outNewlyLearned) {
    outNewlyLearned = false;
    bool matched = false;

    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);

    int idx = findEntryIndexLocked(cmdCode);
    if (idx >= 0) {
        outEntry = s_cfg.entries[idx];
        matched = true;
    } else if (s_cfg.entryCount < CMD_MAP_MAX_ENTRIES) {
        CmdEntry learned = {};
        learned.cmdCode     = cmdCode;
        learned.hidCode     = 0x0000;
        learned.radioAction = RadioAction::NONE;
        learned.actionParam = 0;
        snprintf(learned.name, sizeof(learned.name), "CMD_0x%02X", cmdCode);

        s_cfg.entries[s_cfg.entryCount++] = learned;
        s_dirty = true;

        outEntry        = learned;
        outNewlyLearned = true;
        matched         = true;
    }

    s_lastRx.cmdCode     = cmdCode;
    s_lastRx.timestampMs = millis();
    s_lastRx.valid       = true;
    if (matched) {
        strncpy(s_lastRx.matchedName, outEntry.name, sizeof(s_lastRx.matchedName) - 1);
        s_lastRx.matchedName[sizeof(s_lastRx.matchedName) - 1] = '\0';
    } else {
        strncpy(s_lastRx.matchedName, "(unmapped)", sizeof(s_lastRx.matchedName) - 1);
        s_lastRx.matchedName[sizeof(s_lastRx.matchedName) - 1] = '\0';
    }

    if (s_mapMutex) xSemaphoreGive(s_mapMutex);
    return matched;
}

LastRxInfo btnMap_getLastRx() {
    LastRxInfo info = {};
    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    info = s_lastRx;
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);
    return info;
}

bool btnMap_replaceAll(const CmdEntry* entries, size_t count) {
    if (count > 0 && entries == nullptr) return false;
    if (count > CMD_MAP_MAX_ENTRIES) count = CMD_MAP_MAX_ENTRIES;

    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);

    s_cfg.entryCount = 0;
    for (size_t i = 0; i < count; i++) {
        CmdEntry sanitized = entries[i];
        sanitizeEntry(sanitized);

        int existingIdx = findEntryIndexLocked(sanitized.cmdCode);
        if (existingIdx >= 0) {
            s_cfg.entries[existingIdx] = sanitized;
        } else if (s_cfg.entryCount < CMD_MAP_MAX_ENTRIES) {
            s_cfg.entries[s_cfg.entryCount++] = sanitized;
        }
    }
    s_dirty = true;

    if (s_mapMutex) xSemaphoreGive(s_mapMutex);
    return true;
}

bool btnMap_upsert(const CmdEntry& entry) {
    CmdEntry sanitized = entry;
    sanitizeEntry(sanitized);

    bool ok = false;
    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    int idx = findEntryIndexLocked(sanitized.cmdCode);
    if (idx >= 0) {
        s_cfg.entries[idx] = sanitized;
        s_dirty = true;
        ok = true;
    } else if (s_cfg.entryCount < CMD_MAP_MAX_ENTRIES) {
        s_cfg.entries[s_cfg.entryCount++] = sanitized;
        s_dirty = true;
        ok = true;
    }
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);
    return ok;
}

bool btnMap_remove(uint8_t cmdCode) {
    bool removed = false;
    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    int idx = findEntryIndexLocked(cmdCode);
    if (idx >= 0) {
        for (size_t i = (size_t)idx; i + 1 < s_cfg.entryCount; i++) {
            s_cfg.entries[i] = s_cfg.entries[i + 1];
        }
        s_cfg.entryCount--;
        memset(&s_cfg.entries[s_cfg.entryCount], 0, sizeof(CmdEntry));
        s_dirty = true;
        removed = true;
    }
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);
    return removed;
}

void btnMap_clearAll() {
    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    s_cfg.entryCount = 0;
    memset(s_cfg.entries, 0, sizeof(s_cfg.entries));
    s_dirty = true;
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);
}

KeepaliveConfig btnMap_getKeepalive() {
    KeepaliveConfig cfg = {};
    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    cfg = s_cfg.keepalive;
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);
    return cfg;
}

bool btnMap_setKeepalive(const KeepaliveConfig& cfg) {
    if (cfg.intervalMs < KEEPALIVE_MIN_MS || cfg.intervalMs > KEEPALIVE_MAX_MS) {
        return false;
    }
    if (!ble_hid_is_valid_code(cfg.hidCode, !cfg.enabled)) {
        return false;
    }

    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    s_cfg.keepalive = cfg;
    s_dirty = true;
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);
    return true;
}

String btnMap_entriesJson() {
    CmdEntry snapshot[CMD_MAP_MAX_ENTRIES];
    size_t   count = 0;

    if (s_mapMutex) xSemaphoreTake(s_mapMutex, portMAX_DELAY);
    count = s_cfg.entryCount;
    if (count > 0) {
        memcpy(snapshot, s_cfg.entries, count * sizeof(CmdEntry));
    }
    if (s_mapMutex) xSemaphoreGive(s_mapMutex);

    String json = "[";
    char codeBuf[8];
    char hidBuf[8];
    for (size_t i = 0; i < count; i++) {
        if (i > 0) json += ",";
        snprintf(codeBuf, sizeof(codeBuf), "0x%02X", snapshot[i].cmdCode);
        snprintf(hidBuf,  sizeof(hidBuf),  "0x%04X", snapshot[i].hidCode);

        json += "{\"code\":\"";
        json += codeBuf;
        json += "\",\"name\":\"";
        appendJsonEscaped(json, snapshot[i].name);
        json += "\",\"hid\":\"";
        json += hidBuf;
        json += "\",\"act\":";
        json += String((unsigned)snapshot[i].radioAction);
        json += ",\"param\":";
        json += String((unsigned)snapshot[i].actionParam);
        json += "}";
    }
    json += "]";
    return json;
}
