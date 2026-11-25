#pragma once

#include <Arduino.h>

// ============================================================================
//  ble_hid.h — Bluetooth Low Energy HID-over-GATT (HOGP) Media & Key Driver
// ============================================================================

// Valid HID usage code ranges accepted by ble_hid_send_button()
static constexpr uint16_t HID_MEDIA_MIN = 0x0001;
static constexpr uint16_t HID_MEDIA_MAX = 0x03FF;
static constexpr uint16_t HID_FKEY_MIN  = 0xF001; // F1 (pseudo-range mapped to Keyboard page)
static constexpr uint16_t HID_FKEY_MAX  = 0xF018; // F24

// Standard HID Consumer Control usage codes (Usage Page 0x0C)
enum HidConsumerCode : uint16_t {
    HID_CC_PLAY        = 0x00B0,
    HID_CC_PAUSE       = 0x00B1,
    HID_CC_FAST_FWD    = 0x00B3,
    HID_CC_REWIND      = 0x00B4,
    HID_CC_NEXT_TRACK  = 0x00B5,
    HID_CC_PREV_TRACK  = 0x00B6,
    HID_CC_STOP        = 0x00B7,
    HID_CC_EJECT       = 0x00B8,
    HID_CC_PLAY_PAUSE  = 0x00CD,
    HID_CC_STOP_EJECT  = 0x00CE,
    HID_CC_MUTE        = 0x00E2,
    HID_CC_VOLUME_UP   = 0x00E9,
    HID_CC_VOLUME_DOWN = 0x00EA,
};

// Diagnostic event IDs stored in the BLE event ring buffer
enum class BleEventType : uint8_t {
    CONNECT     = 1,
    DISCONNECT  = 2,
    MAP_READ    = 3,
    SUB_MEDIA   = 5,
    SUB_KEYS    = 6,
    UNSUB_MEDIA = 7,
    UNSUB_KEYS  = 8,
};

// Snapshot of BLE HID runtime statistics for diagnostics and Web UI
struct BleHidStats {
    uint32_t connectCount;
    uint32_t disconnectCount;
    uint32_t sendCount;
    uint32_t rejectedCount;
    uint32_t mapReadCount;
    uint8_t  mediaSubs;
    uint8_t  keySubs;
    uint16_t lastCode;
    uint32_t lastReportMs;
    bool     prefixId;
    char     lastReportHex[16];
};

/**
 * @brief Validate whether a 16-bit code is a supported Consumer Media or F1..F24 code.
 * @param code      Code to validate.
 * @param allowZero If true, 0x0000 ("disabled") is accepted as valid.
 */
inline bool ble_hid_is_valid_code(uint16_t code, bool allowZero = false) {
    if (allowZero && code == 0x0000) return true;
    return (code >= HID_MEDIA_MIN && code <= HID_MEDIA_MAX) ||
           (code >= HID_FKEY_MIN  && code <= HID_FKEY_MAX);
}

void ble_hid_init();
void ble_hid_suspend(bool suspend);
bool ble_hid_ever_connected();
bool ble_hid_is_connected();
bool ble_hid_is_advertising();
void ble_hid_start_advertising();
void ble_hid_stop_advertising();

/**
 * @brief Delete all stored BLE pairing bonds from NVS.
 */
bool ble_hid_delete_bonds();

/**
 * @brief Send a press + 50 ms hold + release HID report for a Media or F1..F24 code.
 */
void ble_hid_send_button(uint16_t usageCode);

// Diagnostics & Report ID prefix settings
BleHidStats ble_hid_get_stats();
bool        ble_hid_get_prefix();
bool        ble_hid_set_prefix(bool on);
String      ble_hid_events_json();
