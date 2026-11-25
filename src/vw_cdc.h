#pragma once

#include <Arduino.h>

// ============================================================================
//  vw_cdc.h — VW CD Changer (CDC) Bus Protocol Driver
//
//  Emulates a factory VW 6-disc CD changer over the 3-wire CDC bus
//  (CLOCK, DATA IN -> radio, DATA OUT <- radio) compatible with VW RCD300 /
//  RCD310 head units (vwcdpic / VWCDC_Driver protocol).
//
//  Callback contract:
//    - CdcCommandCallback receives the validated command byte (<cmd>) from any
//      non-protocol radio packet matching: 0x53 0x2C <cmd> <~cmd>.
//    - The callback is invoked from the CDC task context inside cdc_loop(),
//      NOT from an interrupt service routine (ISR).
//
//  Flash / NVS safety:
//    - Because ESP32-C3 disables flash cache during NVS/OTA writes, callers
//      MUST pause the RX GPIO interrupt with cdc_pause(true) before writing
//      to flash/NVS and resume it afterwards with cdc_pause(false).
// ============================================================================

/**
 * @brief Callback invoked in CDC task context for every validated radio command.
 * @param cmdByte Command payload byte from a valid `53 2C <cmd> <~cmd>` frame.
 */
typedef void (*CdcCommandCallback)(uint8_t cmdByte);

/**
 * @brief Initialize GPIO pins, protocol state machine, and RX pulse ISR.
 * @param clockPin         GPIO connected to radio Quadlock Pin 12 (CLOCK).
 * @param dataToRadioPin   GPIO connected to radio Quadlock Pin 11 (DATA IN).
 * @param dataFromRadioPin GPIO connected to radio Quadlock Pin 6 (DATA OUT), or -1.
 * @param onCommand        Optional callback invoked for received button commands.
 */
void cdc_init(int clockPin, int dataToRadioPin, int dataFromRadioPin,
              CdcCommandCallback onCommand = nullptr);

/**
 * @brief Process RX ring buffer commands and transmit periodic TX status frames.
 * @note  Call frequently (~every 1 ms) from a dedicated high-priority FreeRTOS task.
 */
void cdc_loop();

/**
 * @brief Update the disc (1..7) and track (1..99) reported to the head unit.
 * @note  Track changes apply immediately; changing disc on the radio display
 *        requires calling cdc_restartInit() so the radio sees a disc-load sequence.
 */
void cdc_setDiscTrack(uint8_t disc, uint8_t track);

/**
 * @brief Restart the TX handshake from the disc-load phase (SEQ_LOAD_DISC).
 * @note  Triggers the radio's disc-loading animation before entering PLAYING.
 */
void cdc_restartInit();

/**
 * @brief Toggle the MIX / Random indicator flag sent in PLAYING frames.
 */
void cdc_setRandom(bool on);

/**
 * @brief Toggle the SCAN indicator flag sent in PLAYING frames.
 */
void cdc_setScan(bool on);

/**
 * @brief Reset or set the elapsed mm:ss playback counter displayed on the radio.
 */
void cdc_setPlayTime(uint8_t minutes, uint8_t seconds);

/**
 * @brief Detach (pause=true) or re-attach (pause=false) the DATA OUT RX ISR.
 */
void cdc_pause(bool pause);
