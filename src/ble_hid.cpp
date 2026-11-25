// ============================================================================
//  ble_hid.cpp — NimBLE HID-over-GATT (HOGP) Combo Device Implementation
//
//  Exposes two HID Input Reports:
//    - Report ID 1 (2 bytes): Consumer Control (0x0001..0x03FF, media keys)
//    - Report ID 2 (1 byte) : Keyboard F1..F24 (0xF001..0xF018, Key Mapper)
//  Note: Vendor usage page 0xFF00 is intentionally omitted because Android
//  ignores vendor-defined HID usages without a dedicated native driver.
// ============================================================================

#include "ble_hid.h"
#include "config.h"

#include <NimBLEDevice.h>
#include <Preferences.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define BLE_VERBOSE_LOG 0

static constexpr uint8_t MEDIA_REPORT_ID   = 0x01;
static constexpr uint8_t KEYS_REPORT_ID    = 0x02;
static constexpr uint16_t HID_APPEARANCE_KEYBOARD = 961;
static constexpr uint32_t KEY_HOLD_MS      = 50;
static constexpr size_t  EVT_RING_SIZE     = 10;
static constexpr const char* HID_NVS_NS    = "hidsvc";
static constexpr const char* HID_NVS_PFX   = "pfx";

#if BLE_VERBOSE_LOG
  #define BLE_LOG(...) Serial.printf(__VA_ARGS__)
  #define BLE_LOGS(s)  Serial.println(s)
#else
  #define BLE_LOG(...) do {} while (0)
  #define BLE_LOGS(s)  do {} while (0)
#endif

// Combined HID Report Descriptor (Report 1: Consumer 16-bit, Report 2: Keyboard 8-bit)
static const uint8_t s_reportMap[] = {
    // ---------------- REPORT 1: Consumer Media ----------------
    0x05, 0x0C,        // Usage Page (Consumer)
    0x09, 0x01,        // Usage (Consumer Control)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x01,        //   Report ID (1)
    0x16, 0x00, 0x00,  //   Logical Minimum (0)        - 16-bit
    0x26, 0xFF, 0x03,  //   Logical Maximum (0x03FF)   - 16-bit
    0x1A, 0x00, 0x00,  //   Usage Minimum (0)          - 16-bit
    0x2A, 0xFF, 0x03,  //   Usage Maximum (0x03FF)     - 16-bit
    0x75, 0x10,        //   Report Size (16)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data, Array, Absolute)
    0xC0,              // End Collection

    // ---------------- REPORT 2: Keyboard F1..F24 ----------------
    0x05, 0x01,        // Usage Page (Generic Desktop)
    0x09, 0x06,        // Usage (Keyboard)
    0xA1, 0x01,        // Collection (Application)
    0x85, 0x02,        //   Report ID (2)
    0x05, 0x07,        //   Usage Page (Keyboard/Keypad)
    0x15, 0x00,        //   Logical Minimum (0)        - 8-bit
    0x25, 0x73,        //   Logical Maximum (0x73)     - 8-bit
    0x19, 0x00,        //   Usage Minimum (0)          - 8-bit
    0x29, 0x73,        //   Usage Maximum (0x73 = F24) - 8-bit
    0x75, 0x08,        //   Report Size (8)
    0x95, 0x01,        //   Report Count (1)
    0x81, 0x00,        //   Input (Data, Array, Absolute)
    0xC0               // End Collection
};

static NimBLEServer*         s_server       = nullptr;
static NimBLECharacteristic* s_mediaReport  = nullptr;
static NimBLECharacteristic* s_keyReport    = nullptr;
static NimBLECharacteristic* s_batteryLevel = nullptr;

static volatile bool s_deviceConnected  = false;
static volatile bool s_advertising      = false;
static volatile bool s_bleSuspended     = false;
static volatile bool s_bleEverConnected = false;
static bool          s_bleInitialized   = false;

static SemaphoreHandle_t s_sendMutex = nullptr;
static portMUX_TYPE      s_statsMux  = portMUX_INITIALIZER_UNLOCKED;

// ============================================================================
//  DIAGNOSTICS & EVENT RING BUFFER
// ============================================================================
static bool     s_prefixId        = false; // true -> prepend Report ID byte to notifications
static uint32_t s_connectCount    = 0;
static uint32_t s_disconnectCount = 0;
static uint32_t s_sendCount       = 0;
static uint32_t s_rejectedCount   = 0;
static uint32_t s_mapReadCount    = 0;
static uint16_t s_lastCode        = 0;
static uint8_t  s_lastReport[4]   = { 0 };
static uint8_t  s_lastReportLen   = 0;
static uint32_t s_lastReportMs    = 0;

static BleEventType s_evtType[EVT_RING_SIZE];
static uint32_t     s_evtMs[EVT_RING_SIZE];
static uint8_t      s_evtIdx   = 0;
static uint8_t      s_evtCount = 0;

static void recordBleEvent(BleEventType type) {
    portENTER_CRITICAL(&s_statsMux);
    s_evtType[s_evtIdx] = type;
    s_evtMs[s_evtIdx]   = millis();
    s_evtIdx = (uint8_t)((s_evtIdx + 1U) % EVT_RING_SIZE);
    if (s_evtCount < EVT_RING_SIZE) {
        s_evtCount++;
    }
    portEXIT_CRITICAL(&s_statsMux);
}

static void formatReportHex(const uint8_t* data, uint8_t len, char* out, size_t outSize) {
    if (len == 0 || outSize < 2) {
        if (outSize >= 2) {
            out[0] = '-';
            out[1] = '\0';
        }
        return;
    }
    size_t pos = 0;
    for (uint8_t i = 0; i < len && pos + 3 < outSize; i++) {
        if (i > 0 && pos + 1 < outSize) {
            out[pos++] = ' ';
        }
        int written = snprintf(out + pos, outSize - pos, "%02X", data[i]);
        if (written > 0) {
            pos += (size_t)written;
        }
    }
    out[pos] = '\0';
}

// ============================================================================
//  NIMBLE CALLBACKS
// ============================================================================
class ReportCallbacks : public NimBLECharacteristicCallbacks {
    void onSubscribe(NimBLECharacteristic* pChar, ble_gap_conn_desc* desc, uint16_t sub) override {
        (void)desc;
        bool isKeys = (pChar == s_keyReport);
        BleEventType evt = isKeys
            ? (sub > 0 ? BleEventType::SUB_KEYS : BleEventType::UNSUB_KEYS)
            : (sub > 0 ? BleEventType::SUB_MEDIA : BleEventType::UNSUB_MEDIA);
        recordBleEvent(evt);
        BLE_LOG("[BLE] %s %s\n", sub > 0 ? "SUB" : "UNSUB", isKeys ? "KEYS" : "MEDIA");
    }
};

// Reading Report Map (0x2A4B) confirms the phone performed fresh GATT service discovery
class ReportMapCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pChar) override {
        (void)pChar;
        portENTER_CRITICAL(&s_statsMux);
        s_mapReadCount++;
        portEXIT_CRITICAL(&s_statsMux);
        recordBleEvent(BleEventType::MAP_READ);
        BLE_LOGS("[BLE] Host read Report Map (0x2A4B)");
    }
};

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) override {
        (void)pServer;
        s_deviceConnected  = true;
        s_bleEverConnected = true;
        s_advertising      = false;

        portENTER_CRITICAL(&s_statsMux);
        s_connectCount++;
        portEXIT_CRITICAL(&s_statsMux);

        recordBleEvent(BleEventType::CONNECT);
        NimBLEDevice::stopAdvertising();
        BLE_LOG("[BLE] Device connected (#%u)\n", (unsigned)s_connectCount);

        // Give the phone time to complete service discovery and CCCD subscription
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    void onDisconnect(NimBLEServer* pServer) override {
        (void)pServer;
        s_deviceConnected = false;

        portENTER_CRITICAL(&s_statsMux);
        s_disconnectCount++;
        portEXIT_CRITICAL(&s_statsMux);

        recordBleEvent(BleEventType::DISCONNECT);
        BLE_LOG("[BLE] Device disconnected (#%u)\n", (unsigned)s_disconnectCount);

        if (!s_bleSuspended) {
            NimBLEDevice::startAdvertising();
            s_advertising = true;
            BLE_LOGS("[BLE] Advertising restarted");
        } else {
            BLE_LOGS("[BLE] Suspended - not restarting advertising");
        }
    }
};

// ============================================================================
//  INTERNAL REPORT TRANSMITTER
// ============================================================================
static void sendHidReport(NimBLECharacteristic* reportChar, uint8_t reportId,
                          const uint8_t* payload, uint8_t payloadLen,
                          uint16_t logicalCode, const char* label) {
    if (!s_deviceConnected || reportChar == nullptr) {
        portENTER_CRITICAL(&s_statsMux);
        s_rejectedCount++;
        portEXIT_CRITICAL(&s_statsMux);
        BLE_LOG("[BLE] REJECTED %s 0x%04X (not connected)\n", label, logicalCode);
        return;
    }

    if (reportChar->getSubscribedCount() == 0) {
        portENTER_CRITICAL(&s_statsMux);
        s_rejectedCount++;
        portEXIT_CRITICAL(&s_statsMux);
        BLE_LOG("[BLE] REJECTED %s 0x%04X (no CCCD subscription)\n", label, logicalCode);
        return;
    }

    if (s_sendMutex) {
        xSemaphoreTake(s_sendMutex, portMAX_DELAY);
    }

    uint8_t pressBuf[4] = { 0 };
    uint8_t releaseBuf[4] = { 0 };
    uint8_t len = 0;

    bool usePrefix;
    portENTER_CRITICAL(&s_statsMux);
    usePrefix = s_prefixId;
    portEXIT_CRITICAL(&s_statsMux);

    if (usePrefix) {
        pressBuf[len]   = reportId;
        releaseBuf[len] = reportId;
        len++;
    }
    for (uint8_t i = 0; i < payloadLen && len < sizeof(pressBuf); i++) {
        pressBuf[len]   = payload[i];
        releaseBuf[len] = 0;
        len++;
    }

    uint32_t nowMs = millis();
    portENTER_CRITICAL(&s_statsMux);
    s_lastCode      = logicalCode;
    s_sendCount++;
    s_lastReportMs  = nowMs;
    s_lastReportLen = (len > sizeof(s_lastReport)) ? sizeof(s_lastReport) : len;
    memcpy(s_lastReport, pressBuf, s_lastReportLen);
    portEXIT_CRITICAL(&s_statsMux);

    reportChar->setValue(pressBuf, len);
    reportChar->notify();

    if (BLE_VERBOSE_LOG) {
        char hexBuf[16];
        formatReportHex(pressBuf, len, hexBuf, sizeof(hexBuf));
        BLE_LOG("[BLE] %s 0x%04X -> [%s] sends=%u\n",
                label, logicalCode, hexBuf, (unsigned)s_sendCount);
    }

    vTaskDelay(pdMS_TO_TICKS(KEY_HOLD_MS));

    reportChar->setValue(releaseBuf, len);
    reportChar->notify();

    if (s_sendMutex) {
        xSemaphoreGive(s_sendMutex);
    }
}

// ============================================================================
//  PUBLIC API
// ============================================================================

void ble_hid_init() {
    if (s_bleInitialized) {
        ble_hid_start_advertising();
        return;
    }

    BLE_LOGS("[BLE] NimBLE HID init...");
    NimBLEDevice::init(DEVICE_BLE_NAME);

    s_sendMutex = xSemaphoreCreateMutex();

    {
        Preferences pref;
        if (pref.begin(HID_NVS_NS, true)) {
            bool pfx = (pref.getUChar(HID_NVS_PFX, 0) != 0);
            pref.end();
            portENTER_CRITICAL(&s_statsMux);
            s_prefixId = pfx;
            portEXIT_CRITICAL(&s_statsMux);
        }
    }
    BLE_LOG("[BLE] Prefix Report ID: %s\n", s_prefixId ? "ON" : "OFF");

    NimBLEDevice::setSecurityAuth(true, false, false);         // Bond, no MITM
    NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT); // Just Works

    s_server = NimBLEDevice::createServer();
    s_server->setCallbacks(new ServerCallbacks());

    // ---------------- HID Service (0x1812) ----------------
    NimBLEService* pHid = s_server->createService(NimBLEUUID((uint16_t)0x1812));

    NimBLECharacteristic* pHidInfo = pHid->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A4A), NIMBLE_PROPERTY::READ);
    uint8_t hidInfoVal[4] = { 0x11, 0x01, 0x00, 0x02 };
    pHidInfo->setValue(hidInfoVal, sizeof(hidInfoVal));

    NimBLECharacteristic* pRptMap = pHid->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A4B), NIMBLE_PROPERTY::READ);
    pRptMap->setCallbacks(new ReportMapCallbacks());
    pRptMap->setValue((uint8_t*)s_reportMap, sizeof(s_reportMap));

    // HID Control Point (0x2A4C)
    pHid->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A4C), NIMBLE_PROPERTY::WRITE_NR);

    // Protocol Mode (0x2A4E): Report Protocol (required by Android)
    NimBLECharacteristic* pProtocolMode = pHid->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A4E),
        NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::WRITE_NR);
    uint8_t protocol = 0x01;
    pProtocolMode->setValue(&protocol, 1);

    // Input Report #1: Consumer Media (Report ID 1)
    s_mediaReport = pHid->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A4D), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    s_mediaReport->setCallbacks(new ReportCallbacks());
    NimBLEDescriptor* pRef1 = s_mediaReport->createDescriptor(
        NimBLEUUID((uint16_t)0x2908), NIMBLE_PROPERTY::READ, 2);
    uint8_t ref1Val[2] = { MEDIA_REPORT_ID, 0x01 };
    pRef1->setValue(ref1Val, sizeof(ref1Val));
    uint8_t zeroMedia[2] = { 0, 0 };
    s_mediaReport->setValue(zeroMedia, sizeof(zeroMedia));

    // Input Report #2: Keyboard F1..F24 (Report ID 2)
    s_keyReport = pHid->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A4D), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    s_keyReport->setCallbacks(new ReportCallbacks());
    NimBLEDescriptor* pRef2 = s_keyReport->createDescriptor(
        NimBLEUUID((uint16_t)0x2908), NIMBLE_PROPERTY::READ, 2);
    uint8_t ref2Val[2] = { KEYS_REPORT_ID, 0x01 };
    pRef2->setValue(ref2Val, sizeof(ref2Val));
    uint8_t zeroKey[1] = { 0 };
    s_keyReport->setValue(zeroKey, sizeof(zeroKey));

    pHid->start();

    // ---------------- Battery Service (0x180F) ----------------
    NimBLEService* pBatt = s_server->createService(NimBLEUUID((uint16_t)0x180F));
    s_batteryLevel = pBatt->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A19), NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);
    uint8_t level = 100;
    s_batteryLevel->setValue(&level, 1);
    pBatt->start();

    // ---------------- Device Information Service (0x180A) ----------------
    NimBLEService* pDevInfo = s_server->createService(NimBLEUUID((uint16_t)0x180A));
    NimBLECharacteristic* pManuf = pDevInfo->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A29), NIMBLE_PROPERTY::READ);
    pManuf->setValue(DEVICE_MANUFACTURER);

    NimBLECharacteristic* pPnp = pDevInfo->createCharacteristic(
        NimBLEUUID((uint16_t)0x2A50), NIMBLE_PROPERTY::READ);
    uint8_t pnpVal[7] = { 0x02, 0x02, 0xE5, 0x11, 0xA1, 0x10, 0x02 };
    pPnp->setValue(pnpVal, sizeof(pnpVal));
    pDevInfo->start();

    s_bleInitialized = true;
    BLE_LOGS("[BLE] Services started");
    ble_hid_start_advertising();
    BLE_LOGS("[BLE] HID device initialized");
}

void ble_hid_send_button(uint16_t usageCode) {
    if (usageCode >= HID_FKEY_MIN && usageCode <= HID_FKEY_MAX) {
        uint8_t n = (uint8_t)(usageCode - 0xF000U);
        uint8_t usage = (n <= 12) ? (uint8_t)(0x3AU + n - 1U)   // F1..F12
                                  : (uint8_t)(0x68U + n - 13U); // F13..F24
        sendHidReport(s_keyReport, KEYS_REPORT_ID, &usage, 1, usageCode, "Key");
    } else if (usageCode >= HID_MEDIA_MIN && usageCode <= HID_MEDIA_MAX) {
        uint8_t payload[2] = {
            (uint8_t)(usageCode & 0xFFU),
            (uint8_t)(usageCode >> 8U)
        };
        sendHidReport(s_mediaReport, MEDIA_REPORT_ID, payload, 2, usageCode, "Media");
    } else {
        BLE_LOG("[BLE] Invalid HID code 0x%04X (valid: 0x0001..0x03FF or 0xF001..0xF018)\n",
                usageCode);
    }
}

void ble_hid_suspend(bool suspend) {
    s_bleSuspended = suspend;
    if (suspend) {
        ble_hid_stop_advertising();
        BLE_LOGS("[BLE] Suspended (advertising off)");
    } else {
        BLE_LOGS("[BLE] Resumed");
        if (!s_deviceConnected && !s_advertising) {
            ble_hid_start_advertising();
        }
    }
}

bool ble_hid_is_connected()   { return s_deviceConnected; }
bool ble_hid_is_advertising() { return s_advertising; }
bool ble_hid_ever_connected() { return s_bleEverConnected; }

bool ble_hid_delete_bonds() {
    if (!s_bleInitialized) {
        BLE_LOGS("[BLE] Stack not initialized");
        return false;
    }
    NimBLEDevice::deleteAllBonds();
    BLE_LOGS("[BLE] All bonds deleted - re-pair phone");
    return true;
}

void ble_hid_start_advertising() {
    if (!s_bleInitialized || s_advertising || s_bleSuspended) return;

    NimBLEAdvertising* pAdv = NimBLEDevice::getAdvertising();
    pAdv->addServiceUUID(NimBLEUUID((uint16_t)0x1812));
    pAdv->setAppearance(HID_APPEARANCE_KEYBOARD);

    NimBLEDevice::startAdvertising();
    s_advertising = true;
    BLE_LOGS("[BLE] Advertising started");
}

void ble_hid_stop_advertising() {
    if (!s_bleInitialized || !s_advertising) return;
    NimBLEDevice::stopAdvertising();
    s_advertising = false;
    BLE_LOGS("[BLE] Advertising stopped");
}

BleHidStats ble_hid_get_stats() {
    BleHidStats stats = {};
    uint8_t rawReport[4] = { 0 };
    uint8_t rawLen = 0;

    portENTER_CRITICAL(&s_statsMux);
    stats.connectCount    = s_connectCount;
    stats.disconnectCount = s_disconnectCount;
    stats.sendCount       = s_sendCount;
    stats.rejectedCount   = s_rejectedCount;
    stats.mapReadCount    = s_mapReadCount;
    stats.lastCode        = s_lastCode;
    stats.lastReportMs    = s_lastReportMs;
    stats.prefixId        = s_prefixId;
    rawLen                = s_lastReportLen;
    memcpy(rawReport, s_lastReport, sizeof(rawReport));
    portEXIT_CRITICAL(&s_statsMux);

    stats.mediaSubs = s_mediaReport ? (uint8_t)s_mediaReport->getSubscribedCount() : 0;
    stats.keySubs   = s_keyReport   ? (uint8_t)s_keyReport->getSubscribedCount()   : 0;
    formatReportHex(rawReport, rawLen, stats.lastReportHex, sizeof(stats.lastReportHex));

    return stats;
}

bool ble_hid_get_prefix() {
    bool pfx;
    portENTER_CRITICAL(&s_statsMux);
    pfx = s_prefixId;
    portEXIT_CRITICAL(&s_statsMux);
    return pfx;
}

bool ble_hid_set_prefix(bool on) {
    Preferences pref;
    if (!pref.begin(HID_NVS_NS, false)) {
        BLE_LOGS("[BLE] Failed to open NVS for prefix save");
        return false;
    }
    pref.putUChar(HID_NVS_PFX, on ? 1 : 0);
    pref.end();

    portENTER_CRITICAL(&s_statsMux);
    s_prefixId = on;
    portEXIT_CRITICAL(&s_statsMux);

    BLE_LOG("[BLE] Prefix Report ID: %s (saved to NVS)\n", on ? "ON" : "OFF");
    return true;
}

String ble_hid_events_json() {
    BleEventType types[EVT_RING_SIZE];
    uint32_t     times[EVT_RING_SIZE];
    uint8_t      count = 0;
    uint8_t      startIdx = 0;

    portENTER_CRITICAL(&s_statsMux);
    count = s_evtCount;
    if (count > 0) {
        startIdx = (uint8_t)((s_evtIdx + EVT_RING_SIZE - count) % EVT_RING_SIZE);
        for (uint8_t i = 0; i < count; i++) {
            uint8_t idx = (uint8_t)((startIdx + i) % EVT_RING_SIZE);
            types[i] = s_evtType[idx];
            times[i] = s_evtMs[idx];
        }
    }
    portEXIT_CRITICAL(&s_statsMux);

    String s = "[";
    for (uint8_t i = 0; i < count; i++) {
        if (i > 0) s += ",";
        s += "[" + String((int)types[i]) + "," + String((unsigned long)times[i]) + "]";
    }
    return s + "]";
}
