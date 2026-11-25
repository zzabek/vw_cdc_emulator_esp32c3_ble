#pragma once

// ============================================================================
//  bt_webui.h — HTTP Configuration Portal, Learn Mode UI & OTA Server
// ============================================================================

/**
 * @brief Start mDNS and register all HTTP / API / OTA routes on port 80.
 */
void btWebUI_init();

/**
 * @brief Execute deferred BLE, NVS, OTA watchdog, and reboot tasks safely
 *        outside the AsyncTCP network callback context.
 */
void btWebUI_loop();

/**
 * @brief Return true while an OTA firmware upload is actively writing flash.
 */
bool btWebUI_isOtaActive();
