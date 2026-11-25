// ============================================================================
//  main.cpp — FreeRTOS Task Orchestration & Radio <-> BLE HID Dispatch
//
//  Tasks:
//    - cdcTask   (prio 3): Runs vw_cdc 62.5 kHz TX bit-bang and RX decoding.
//    - webuiTask (prio 2): Sends configurable BLE keepalive packets, starts
//                          SoftAP + Web UI if BLE never connected after boot
//                          timeout, and drives btWebUI_loop().
//    - loop      (prio 1): Drains radio button command queue, handles sticky vs
//                          momentary disc display timers, executes BLE HID
//                          outputs, and commits learned mappings to NVS.
// ============================================================================

#include <Arduino.h>
#include <WiFi.h>
#include <esp_ota_ops.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "config.h"
#include "vw_cdc.h"
#include "ble_hid.h"
#include "bt_webui.h"
#include "btn_map.h"

// Defer ESP-IDF OTA validation until tasks, BLE/Wi-Fi, and WebUI boot cleanly.
// If new firmware crashes before confirmation, bootloader auto-rolls back.
extern "C" bool verifyRollbackLater() {
    return true;
}

static bool g_otaConfirmed = false;

static void confirmOtaAppValidIfPending() {
    if (g_otaConfirmed) return;

    const esp_partition_t* running = esp_ota_get_running_partition();
    if (!running) {
        g_otaConfirmed = true;
        return;
    }

    esp_ota_img_states_t otaState = ESP_OTA_IMG_UNDEFINED;
    if (esp_ota_get_state_partition(running, &otaState) == ESP_OK &&
        otaState == ESP_OTA_IMG_PENDING_VERIFY) {
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            Serial.println("[OTA] Firmware confirmed valid (bootloader rollback cancelled)");
        } else {
            Serial.println("[OTA] WARNING: Failed to mark OTA partition valid");
        }
    }
    g_otaConfirmed = true;
}

static QueueHandle_t g_cmdQueue        = nullptr;
static bool          g_wifiStarted     = false;
static uint32_t      g_bootTimeMs      = 0;

// Radio display & changer state
static uint8_t       g_stickyDisc        = 1;
static uint8_t       g_displayedDisc     = 1;
static uint8_t       g_currentTrack      = 1;
static uint8_t       g_lastSyncedDisc    = 0;
static uint8_t       g_lastSyncedTrack   = 0;
static bool          g_momentaryActive   = false;
static uint32_t      g_momentaryReturnAt = 0;
static bool          g_scanOn            = false;
static bool          g_randomOn          = false;

// Button debounce state
static uint8_t       g_lastCmdCode       = 0xFF;
static uint32_t      g_lastCmdTimeMs     = 0;
static bool          g_hasLastCmd        = false;

// ============================================================================
//  COMMAND CALLBACK (runs in cdcTask context, NOT ISR)
// ============================================================================
static void onRadioCommand(uint8_t cmdByte) {
    if (g_cmdQueue != nullptr) {
        xQueueSend(g_cmdQueue, &cmdByte, 0);
    }
}

// ============================================================================
//  TASK: VW CDC PROTOCOL
// ============================================================================
static void cdcTask(void* parameter) {
    (void)parameter;
    Serial.println("[TASK-CDC] Starting...");
    cdc_init(CDC_CLK_PIN, CDC_DATA_IN_PIN, CDC_DATA_OUT_PIN, onRadioCommand);
    Serial.println("[TASK-CDC] Initialized - streaming CDC frames");

    for (;;) {
        cdc_loop();
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ============================================================================
//  TASK: WEB UI & CONFIGURABLE BLE KEEPALIVE
// ============================================================================
static void webuiTask(void* parameter) {
    (void)parameter;
    Serial.println("[TASK-WEBUI] Started");
    uint32_t lastLogMs       = 0;
    uint32_t lastKeepaliveMs = millis();

    for (;;) {
        uint32_t nowMs     = millis();
        uint32_t elapsedMs = nowMs - g_bootTimeMs;
        bool bleConnected  = ble_hid_is_connected();

        // Configurable periodic BLE HID keepalive (skipped during OTA)
        KeepaliveConfig ka = btnMap_getKeepalive();
        if (ka.enabled && bleConnected && !btWebUI_isOtaActive() &&
            ka.hidCode != 0 && (nowMs - lastKeepaliveMs >= ka.intervalMs)) {
            lastKeepaliveMs = nowMs;
            ble_hid_send_button(ka.hidCode);
        }

        // Periodic status log every 5 seconds
        if (nowMs - lastLogMs >= 5000U) {
            lastLogMs = nowMs;
            Serial.printf("[TASK-WEBUI] %lu ms | BLE: %s | WiFi: %s | Stations: %d | FreeHeap: %u B\n",
                          (unsigned long)elapsedMs,
                          bleConnected ? "YES" : "NO",
                          g_wifiStarted ? "YES" : "NO",
                          g_wifiStarted ? WiFi.softAPgetStationNum() : 0,
                          (unsigned)ESP.getFreeHeap());
        }

        // Start configuration Wi-Fi AP if no BLE device has connected within timeout
        if (!g_wifiStarted && !bleConnected &&
            elapsedMs >= WEBUI_AP_DELAY_MS && !ble_hid_ever_connected()) {
            Serial.println("[TASK-WEBUI] >>> Starting WiFi AP (Configuration Mode)...");
            vTaskDelay(pdMS_TO_TICKS(1000));

            WiFi.mode(WIFI_AP);
            WiFi.setTxPower(WIFI_POWER_19_5dBm);
            vTaskDelay(pdMS_TO_TICKS(1000));
            bool apResult = WiFi.softAP(DEVICE_WIFI_SSID, NULL, 6, 0, 4);

            if (apResult) {
                vTaskDelay(pdMS_TO_TICKS(200));
                Serial.printf("[TASK-WEBUI] >>> AP Started! IP: %s\n",
                              WiFi.softAPIP().toString().c_str());
                btWebUI_init();
                g_wifiStarted = true;
                Serial.printf("[TASK-WEBUI] >>> WebUI RUNNING! Connect to SSID '%s' (http://%s or http://%s.local)\n",
                              DEVICE_WIFI_SSID,
                              WiFi.softAPIP().toString().c_str(),
                              DEVICE_MDNS_HOST);
            } else {
                Serial.println("[TASK-WEBUI] >>> ERROR: SoftAP failed to start");
            }
        }

        if (g_wifiStarted || (ble_hid_ever_connected() && elapsedMs >= 15000U)) {
            confirmOtaAppValidIfPending();
        }

        // Single runtime owner of NVS commits (safe even while Wi-Fi AP is off)
        if (!btWebUI_isOtaActive() && btnMap_isDirty()) {
            cdc_pause(true);
            btnMap_saveIfDirty();
            cdc_pause(false);
        }

        if (g_wifiStarted) {
            btWebUI_loop();
            vTaskDelay(pdMS_TO_TICKS(5));
        } else {
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}

// ============================================================================
//  RADIO DISPLAY & COMMAND HANDLING (loopTask)
// ============================================================================
static uint8_t currentDisplayTrack() {
    return ble_hid_is_connected() ? g_currentTrack : DISPLAY_TRACK_WAIT_BT;
}

static void selectDisc(uint8_t disc) {
    if (disc < 1) disc = 1;
    if (disc > 6) disc = 6;

    g_displayedDisc = disc;
    g_currentTrack  = 1;

    uint8_t targetTrack = currentDisplayTrack();
    cdc_setDiscTrack(g_displayedDisc, targetTrack);
    cdc_setPlayTime(0, 0);
    cdc_restartInit();

    g_lastSyncedDisc  = g_displayedDisc;
    g_lastSyncedTrack = targetTrack;
}

static void changeTrack(int delta) {
    int nextTrack = (int)g_currentTrack + delta;
    if (nextTrack < 1)  nextTrack = 1;
    if (nextTrack > 99) nextTrack = 99;

    g_currentTrack = (uint8_t)nextTrack;
    uint8_t targetTrack = currentDisplayTrack();
    cdc_setDiscTrack(g_displayedDisc, targetTrack);
    cdc_setPlayTime(0, 0);

    g_lastSyncedDisc  = g_displayedDisc;
    g_lastSyncedTrack = targetTrack;
}

static void handleRadioCommand(uint8_t cmdCode) {
    uint32_t nowMs = millis();
    if (g_hasLastCmd && cmdCode == g_lastCmdCode &&
        (nowMs - g_lastCmdTimeMs) < BUTTON_DEBOUNCE_MS) {
        return;
    }
    g_hasLastCmd    = true;
    g_lastCmdCode   = cmdCode;
    g_lastCmdTimeMs = nowMs;

    CmdEntry entry = {};
    bool newlyLearned = false;
    bool matched = btnMap_handleRxCommand(cmdCode, entry, newlyLearned);

    if (newlyLearned) {
        Serial.printf("[LEARN] Captured new packet 0x%02X -> registered as %s\n",
                      cmdCode, entry.name);
        return;
    }
    if (!matched) {
        Serial.printf("[CDC] Mapping table full (%u max) - ignored packet 0x%02X\n",
                      (unsigned)CMD_MAP_MAX_ENTRIES, cmdCode);
        return;
    }

    // 1) Update local radio display state based on configured action
    switch (entry.radioAction) {
    case RadioAction::TRACK_NEXT:
        changeTrack(1);
        break;
    case RadioAction::TRACK_PREV:
        changeTrack(-1);
        break;
    case RadioAction::DISC_STICKY:
        g_momentaryActive = false;
        g_stickyDisc      = entry.actionParam;
        selectDisc(g_stickyDisc);
        break;
    case RadioAction::DISC_MOMENTARY:
        selectDisc(entry.actionParam);
        g_momentaryActive   = true;
        g_momentaryReturnAt = millis() + MOMENTARY_FLASH_MS;
        break;
    case RadioAction::SCAN_TOGGLE:
        g_scanOn = !g_scanOn;
        cdc_setScan(g_scanOn);
        break;
    case RadioAction::RANDOM_TOGGLE:
        g_randomOn = !g_randomOn;
        cdc_setRandom(g_randomOn);
        break;
    case RadioAction::NONE:
    default:
        break;
    }

    // 2) Dispatch configured BLE HID key/media usage to phone (0x0000 = none)
    if (entry.hidCode != 0) {
        ble_hid_send_button(entry.hidCode);
    }
}

// ============================================================================
//  SETUP & LOOP
// ============================================================================
void setup() {
    Serial.begin(115200);
    delay(200); // Short delay: radio probes for CDC immediately at ignition

    Serial.println("========================================");
    Serial.printf("[SETUP] Firmware: %s (%s)\n", FW_VERSION, DEVICE_BLE_NAME);
    Serial.println("========================================");

    g_bootTimeMs = millis();

    btnMap_init();
    btnMap_saveIfDirty(); // Persist seeded defaults before CDC ISR attaches

    g_cmdQueue = xQueueCreate(10, sizeof(uint8_t));

    // Order matters: start CDC task (prio 3) before ble_hid_init() blocks setup
    Serial.println("[SETUP] Creating CDC task...");
    xTaskCreate(cdcTask, "CDC", CDC_TASK_STACK_SIZE, NULL, 3, NULL);

    Serial.println("[SETUP] Initializing BLE HID...");
    ble_hid_init();
    Serial.println("[SETUP] BLE HID ready");

    Serial.println("[SETUP] Creating WebUI task...");
    xTaskCreate(webuiTask, "WebUI", WEBUI_TASK_STACK_SIZE, NULL, 2, NULL);

    Serial.println("========================================");
}

void loop() {
    // 1. Drain radio button command queue
    uint8_t cmdCode = 0;
    while (g_cmdQueue && xQueueReceive(g_cmdQueue, &cmdCode, 0) == pdTRUE) {
        handleRadioCommand(cmdCode);
    }

    // 2. Non-blocking return from momentary disc indicator back to sticky disc
    if (g_momentaryActive && (int32_t)(millis() - g_momentaryReturnAt) >= 0) {
        g_momentaryActive = false;
        selectDisc(g_stickyDisc);
    }

    // 3. Keep displayed track synchronized with BLE connection status
    uint8_t targetTrack = currentDisplayTrack();
    if (g_displayedDisc != g_lastSyncedDisc || targetTrack != g_lastSyncedTrack) {
        cdc_setDiscTrack(g_displayedDisc, targetTrack);
        g_lastSyncedDisc  = g_displayedDisc;
        g_lastSyncedTrack = targetTrack;
    }

    vTaskDelay(pdMS_TO_TICKS(50));
}
