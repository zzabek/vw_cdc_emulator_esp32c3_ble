#pragma once

// ============================================================================
//  config.h — Centralized firmware, radio identity, GPIO pinout & task config
// ============================================================================

#define FW_VERSION                  "v4.0-BLE-HID"

// Device identity advertised over BLE HID and Wi-Fi SoftAP
#define DEVICE_BLE_NAME             "RCD310"
#define DEVICE_WIFI_SSID            "RCD310"
#define DEVICE_MDNS_HOST            "vw-bt"
#define DEVICE_MANUFACTURER         "VW-CDC-Emulator"

// ESP32-C3 GPIO <-> VW Quadlock 12-pin Connector 4 (CDC)
// Pin 12 -> CLOCK, Pin 11 -> DATA IN, Pin 6 -> DATA OUT
#define CDC_CLK_PIN                 0
#define CDC_DATA_IN_PIN             3
#define CDC_DATA_OUT_PIN            10

// Runtime behavior & timing
#define BUTTON_DEBOUNCE_MS          300U
#define WEBUI_AP_DELAY_MS           10000U
#define DISPLAY_TRACK_WAIT_BT       88
#define MOMENTARY_FLASH_MS          250U

// FreeRTOS task stacks (bytes)
#define CDC_TASK_STACK_SIZE         4096U
#define WEBUI_TASK_STACK_SIZE       8192U

// Default BLE keepalive configuration (editable at runtime via Web UI)
#define KEEPALIVE_DEFAULT_ENABLED     true
#define KEEPALIVE_DEFAULT_INTERVAL_MS 5000U
#define KEEPALIVE_DEFAULT_HID_CODE    0xF008
