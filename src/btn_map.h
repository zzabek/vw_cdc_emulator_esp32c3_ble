#pragma once

#include <Arduino.h>

// ============================================================================
//  btn_map.h — Dynamic Radio Command Mapping, Learn Mode & Keepalive Config
//
//  Maps raw VW CDC protocol command bytes (<cmd> from `53 2C <cmd> <~cmd>`)
//  to BLE HID actions and head-unit display behaviors (sticky vs momentary
//  disc selection, track stepping, scan/random toggles).
//  Persisted as a versioned binary blob in NVS namespace "cdccfg".
// ============================================================================

static constexpr size_t CMD_MAP_MAX_ENTRIES = 24;
static constexpr size_t CMD_NAME_MAX_LEN    = 16; // 15 visible chars + NUL

/**
 * @brief Local radio display/state behavior triggered alongside BLE HID output.
 */
enum class RadioAction : uint8_t {
    NONE           = 0, // Do not change radio disc/track/mode display
    TRACK_NEXT     = 1, // Increment displayed track (+1)
    TRACK_PREV     = 2, // Decrement displayed track (-1)
    DISC_STICKY    = 3, // Switch and stay on disc `actionParam` (1..6)
    DISC_MOMENTARY = 4, // Flash disc `actionParam` (1..6) briefly, then return to sticky disc
    SCAN_TOGGLE    = 5, // Toggle radio SCAN indicator
    RANDOM_TOGGLE  = 6, // Toggle radio MIX indicator
};

struct CmdEntry {
    uint8_t     cmdCode;
    char        name[CMD_NAME_MAX_LEN];
    uint16_t    hidCode;
    RadioAction radioAction;
    uint8_t     actionParam; // Disc number 1..6 for DISC_STICKY / DISC_MOMENTARY
};

struct KeepaliveConfig {
    bool     enabled;
    uint32_t intervalMs;
    uint16_t hidCode;
};

struct LastRxInfo {
    uint8_t  cmdCode;
    uint32_t timestampMs;
    bool     valid;
    char     matchedName[CMD_NAME_MAX_LEN];
};

// Lifecycle & NVS persistence
void btnMap_init();
bool btnMap_isDirty();
bool btnMap_saveIfDirty(); // Caller MUST wrap with cdc_pause(true/false)

// RX command lookup & Learn Mode auto-registration
bool btnMap_handleRxCommand(uint8_t cmdCode, CmdEntry& outEntry, bool& outNewlyLearned);
LastRxInfo btnMap_getLastRx();

// Mapping table management
bool btnMap_replaceAll(const CmdEntry* entries, size_t count);
bool btnMap_upsert(const CmdEntry& entry);
bool btnMap_remove(uint8_t cmdCode);
void btnMap_clearAll();
String btnMap_entriesJson();

// Keepalive configuration
KeepaliveConfig btnMap_getKeepalive();
bool            btnMap_setKeepalive(const KeepaliveConfig& cfg);
