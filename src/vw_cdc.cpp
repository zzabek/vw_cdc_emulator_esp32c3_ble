// ============================================================================
//  vw_cdc.cpp — VW CD Changer Bus Emulator (vwcdpic / VWCDC_Driver protocol)
//
//  PROTOCOL ARCHITECTURE & DATA FLOW
//  ---------------------------------------------------------------------------
//  This driver manages two independent half-duplex signaling channels between
//  the ESP32-C3 and a VW RCD300 / RCD310 head unit:
//
//  1. TX Channel (ESP -> Radio, CLOCK + DATA IN, 62.5 kHz SPI Mode 1 bit-bang):
//     - The emulator generates its own clock (8 us HIGH, 8 us LOW = 16 us/bit)
//       with an 874 us inter-byte pause between bytes 0..6 of each 8-byte frame.
//       Software bit-bang is used instead of the ESP32 Arduino core 3.x SPI API
//       because the hardware driver does not offer deterministic inter-byte gaps.
//     - Frames are transmitted every 50 ms (~20 Hz) via a 4-stage startup
//       handshake that mimics a physical Panasonic/Sony VW CD changer:
//         [1] SEQ_ANNOUNCE_PRESENCE (20 frames on boot, 10 after 0xE4 hello):
//             Idle presence frames (0x74 ... 0x7C) telling the head unit a CDC
//             is connected on the bus.
//         [2] SEQ_LOAD_DISC (12 frames after announce, 24 on cdc_restartInit):
//             Alternates magazine-slot announcement (0x2E..0x29) and status
//             frames so the radio shows "LOAD CDx".
//         [3] SEQ_LEAD_IN (10 frames):
//             Alternating disc lead-in sequence transitioning into playback.
//         [4] SEQ_PLAYING (continuous loop):
//             Live status frames reporting disc, BCD track, BCD mm:ss, and
//             SCAN/MIX mode flags:
//               f0 = 0x34 (PLAY frame header)
//               f1 = 0xBF - disc (bitwise complement encoding used by VW CDC)
//               f2 = 0xFF - BCD(track)
//               f3 = 0xFF - BCD(minutes)
//               f4 = 0xFF - BCD(seconds)
//               f5 = playbackModeByte (0x00=normal, 0x04=MIX, 0xD0=SCAN, 0xD4=both)
//               f6 = 0xCF (PLAY flags)
//               f7 = 0x3C (PLAY frame trailer, always f0 ^ 0x08)
//
//  2. RX Channel (Radio -> ESP, DATA OUT, Pulse-Width Modulated):
//     - The head unit encodes commands as LOW pulse widths on DATA OUT:
//         <  256 us : electrical noise (ignored)
//         >= 3200 us: START marker (~4500 us) -> begins a new 32-bit frame
//         >= 1248 us: logic '1' (~1700 us)
//         otherwise : logic '0' (~570 us)
//     - dataOutPulseIsr() runs in IRAM, assembles bits MSB-first into bytes,
//       and pushes completed bytes into a lock-free SPSC ring buffer.
//     - dispatchReceivedCommands() drains the ring buffer in CDC task context,
//       resynchronizes byte-by-byte, and validates 4-byte command frames:
//         0x53 0x2C <cmd> <~cmd>   (where cmd + ~cmd == 0xFF)
//       This checksum rejects noise and periodic bus-polling zero frames.
//     - Protocol-level control codes (0x14 hold-repeat, 0x38 disc ACK, 0xE4
//       ignition hello) are handled internally; all other valid <cmd> bytes are
//       forwarded to the application callback so any button can be learned.
//
//  Inspired by: https://github.com/ValeraBat/VWCDC_Driver and the vwcdpic project.
// ============================================================================

#include "vw_cdc.h"
#include <Arduino.h>
#include <driver/gpio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// ============================================================================
//  PROTOCOL CONSTANTS
// ============================================================================

// RX command frame header & timing thresholds (verified on VW RCD300/RCD310)
static constexpr uint8_t  CMD_HEADER_BYTE0      = 0x53;
static constexpr uint8_t  CMD_HEADER_BYTE1      = 0x2C;
static constexpr uint32_t RX_PULSE_MIN_VALID_US = 256;
static constexpr uint32_t RX_PULSE_MIN_BIT1_US  = 1248;
static constexpr uint32_t RX_PULSE_MIN_START_US = 3200;
static constexpr uint8_t  RX_FRAME_BITS         = 32;
static constexpr uint8_t  RX_RING_SIZE          = 24;   // 6 command frames buffer

// Protocol-internal RX control commands (not forwarded as user buttons)
static constexpr uint8_t  RX_CMD_HOLD_REPEAT    = 0x14;
static constexpr uint8_t  RX_CMD_DISC_ACK       = 0x38;
static constexpr uint8_t  RX_CMD_RADIO_HELLO    = 0xE4;

// TX bit-bang timing & cadence
static constexpr uint32_t TX_HALF_BIT_US        = 8;    // 16 us/bit = 62.5 kHz
static constexpr uint32_t TX_INTERBYTE_GAP_US   = 874;  // Pause after bytes 0..6
static constexpr uint32_t TX_FRAME_INTERVAL_MS  = 50;   // ~20 frames/s

// TX frame headers, trailers & field encodings
static constexpr uint8_t  TX_HDR_IDLE           = 0x74;
static constexpr uint8_t  TX_TRL_IDLE           = 0x7C;
static constexpr uint8_t  TX_HDR_PLAY           = 0x34;
static constexpr uint8_t  TX_TRL_PLAY           = 0x3C;

static constexpr uint8_t  TX_DISC_BASE          = 0xBF; // f1 = 0xBF - disc
static constexpr uint8_t  TX_STATUS_IDLE_F6     = 0x8F;
static constexpr uint8_t  TX_STATUS_LOAD_F6     = 0xEF;
static constexpr uint8_t  TX_STATUS_LEADIN_F6   = 0xAE;
static constexpr uint8_t  TX_ANNOUNCE_F5        = 0xB7;
static constexpr uint8_t  TX_PLAY_FLAGS_F6      = 0xCF;
static constexpr uint8_t  TX_ANNOUNCE_TRACK_INV = (uint8_t)(0xFF - 0x99);
static constexpr uint8_t  TX_ANNOUNCE_SEC_INV   = (uint8_t)(0xFF - 0x59);

static constexpr uint8_t  TX_SLOT_ANNOUNCE_FIRST = 0x2E;
static constexpr uint8_t  TX_SLOT_ANNOUNCE_LAST  = 0x29;

// Byte 5 (playback mode) bitmasks for VW CDC PLAY frames
static constexpr uint8_t  TX_MODE_NORMAL        = 0x00;
static constexpr uint8_t  TX_MODE_MIX           = 0x04;
static constexpr uint8_t  TX_MODE_SCAN          = 0xD0;
static constexpr uint8_t  TX_MODE_MIX_SCAN      = 0xD4;

// Startup state machine phase frame counts
static constexpr uint16_t PHASE_FRAMES_ANNOUNCE_BOOT  = 20;
static constexpr uint16_t PHASE_FRAMES_ANNOUNCE_HELLO = 10;
static constexpr uint16_t PHASE_FRAMES_LOAD_BOOT      = 12;
static constexpr uint16_t PHASE_FRAMES_LOAD_RESTART   = 24;
static constexpr uint16_t PHASE_FRAMES_LEAD_IN        = 10;

// ============================================================================
//  HARDWARE PINS, CALLBACK & SYNCHRONIZATION
// ============================================================================
static int s_pinClockToRadio  = -1;
static int s_pinDataToRadio   = -1;
static int s_pinDataFromRadio = -1;

static CdcCommandCallback s_commandCallback = nullptr;
static portMUX_TYPE       s_cdcMux          = portMUX_INITIALIZER_UNLOCKED;
static SemaphoreHandle_t  s_pauseMutex      = nullptr;
static uint32_t           s_pauseDepth      = 0;

// ============================================================================
//  EMULATED CHANGER STATE (protected by s_cdcMux across cdcTask & loopTask)
// ============================================================================
enum class TxSequenceState : uint8_t {
    ANNOUNCE_PRESENCE,
    LOAD_DISC,
    LEAD_IN,
    PLAYING
};

static uint8_t         s_disc             = 1;
static uint8_t         s_track            = 1;
static bool            s_randomOn         = false;
static bool            s_scanOn           = false;
static uint8_t         s_playbackModeByte = TX_MODE_NORMAL;
static uint8_t         s_playTimeMinutes  = 0;
static uint8_t         s_playTimeSeconds  = 0;
static uint32_t        s_lastSecondTickMs = 0;

static TxSequenceState s_sequenceState    = TxSequenceState::ANNOUNCE_PRESENCE;
static uint16_t        s_phaseFrameIndex  = 0;
static uint16_t        s_phaseFrameTarget = PHASE_FRAMES_ANNOUNCE_BOOT;
static uint8_t         s_discAnnounceByte = TX_SLOT_ANNOUNCE_FIRST;

// Exclusive to cdcTask (single caller of cdc_loop)
static uint32_t        s_lastStatusFrameMs = 0;

// ============================================================================
//  RX RING BUFFER & ISR DECODER STATE (single-producer ISR, single-consumer task)
// ============================================================================
static volatile uint8_t  s_rxRing[RX_RING_SIZE];
static volatile uint8_t  s_rxWriteIdx        = 0;
static volatile uint8_t  s_rxReadIdx         = 0;
static volatile bool     s_rxFrameInProgress = false;
static volatile uint8_t  s_rxBitsLeftInByte  = 8;
static volatile uint8_t  s_rxBitsLeftInFrame = 0;
static volatile uint8_t  s_rxAssembledByte   = 0;
static volatile uint32_t s_rxFallingEdgeUs   = 0;
static volatile bool     s_rxInsideLowPulse  = false;

// ============================================================================
//  INTERNAL HELPERS
// ============================================================================

/**
 * @brief Convert an integer 0..99 into packed BCD (e.g. 42 -> 0x42).
 */
static inline uint8_t toBCD(uint8_t value) {
    if (value > 99) value = 99;
    return (uint8_t)(((value / 10U) << 4) | (value % 10U));
}

/**
 * @brief Recompute byte 5 (SCAN / MIX flags) of PLAY frames.
 * @note  Caller must hold s_cdcMux.
 */
static void updatePlaybackModeByteLocked() {
    if (s_scanOn && s_randomOn)      s_playbackModeByte = TX_MODE_MIX_SCAN;
    else if (s_scanOn)               s_playbackModeByte = TX_MODE_SCAN;
    else if (s_randomOn)             s_playbackModeByte = TX_MODE_MIX;
    else                             s_playbackModeByte = TX_MODE_NORMAL;
}

/**
 * @brief Reset all RX pulse-decoder state variables (used on init and resume).
 */
static void resetRxDecoderState(bool clearRingIndices) {
    if (clearRingIndices) {
        s_rxWriteIdx = 0;
        s_rxReadIdx  = 0;
    }
    s_rxFrameInProgress = false;
    s_rxBitsLeftInByte  = 8;
    s_rxBitsLeftInFrame = 0;
    s_rxAssembledByte   = 0;
    s_rxInsideLowPulse  = false;
    s_rxFallingEdgeUs   = 0;
}

/**
 * @brief Bit-bang a single bit onto CLOCK and DATA IN (SPI Mode 1).
 *
 * Rising CLK drives the data bit, holds 8 us, then falling CLK lets the radio
 * sample the line followed by another 8 us hold.
 */
static void txSendOneBit(bool bit) {
    digitalWrite(s_pinClockToRadio, HIGH);
    digitalWrite(s_pinDataToRadio, bit ? HIGH : LOW);
    delayMicroseconds(TX_HALF_BIT_US);
    digitalWrite(s_pinClockToRadio, LOW);
    delayMicroseconds(TX_HALF_BIT_US);
}

/**
 * @brief Transmit an 8-byte status frame MSB-first and return lines to idle LOW.
 */
static void sendFrameToRadio(const uint8_t frame[8]) {
    for (int byteIdx = 0; byteIdx < 8; ++byteIdx) {
        for (int bitIdx = 7; bitIdx >= 0; --bitIdx) {
            txSendOneBit(((frame[byteIdx] >> bitIdx) & 0x01) != 0);
        }
        if (byteIdx < 7) {
            delayMicroseconds(TX_INTERBYTE_GAP_US);
        }
    }
    digitalWrite(s_pinClockToRadio, LOW);
    digitalWrite(s_pinDataToRadio,  LOW);
}

/**
 * @brief Advance the mm:ss playback clock once per second while in PLAYING state.
 */
static void tickPlayTime(uint32_t nowMs) {
    portENTER_CRITICAL(&s_cdcMux);
    if (s_sequenceState == TxSequenceState::PLAYING &&
        (nowMs - s_lastSecondTickMs) >= 1000U) {
        s_lastSecondTickMs = nowMs;
        if (++s_playTimeSeconds >= 60) {
            s_playTimeSeconds = 0;
            if (s_playTimeMinutes < 99) {
                s_playTimeMinutes++;
            }
        }
    }
    portEXIT_CRITICAL(&s_cdcMux);
}

/**
 * @brief Populate the next 8-byte TX frame and advance the startup state machine.
 */
static void buildNextTxFrame(uint8_t frame[8]) {
    portENTER_CRITICAL(&s_cdcMux);

    uint8_t disc = s_disc;
    if (disc < 1) disc = 1;
    if (disc > 7) disc = 7;

    uint8_t track = s_track;
    if (track < 1)  track = 1;
    if (track > 99) track = 99;

    switch (s_sequenceState) {
    case TxSequenceState::ANNOUNCE_PRESENCE:
        frame[0] = TX_HDR_IDLE;
        frame[1] = (uint8_t)(TX_DISC_BASE - disc);
        frame[2] = (uint8_t)(0xFF - track);
        frame[3] = 0xFF;
        frame[4] = 0xFF;
        frame[5] = 0xFF;
        frame[6] = TX_STATUS_IDLE_F6;
        frame[7] = TX_TRL_IDLE;

        if (++s_phaseFrameIndex >= s_phaseFrameTarget) {
            s_sequenceState    = TxSequenceState::LOAD_DISC;
            s_phaseFrameIndex  = 0;
            s_phaseFrameTarget = PHASE_FRAMES_LOAD_BOOT;
            s_discAnnounceByte = TX_SLOT_ANNOUNCE_FIRST;
        }
        break;

    case TxSequenceState::LOAD_DISC:
        if ((s_phaseFrameIndex & 1U) == 0U) {
            frame[0] = TX_HDR_PLAY;
            frame[1] = s_discAnnounceByte;
            frame[2] = TX_ANNOUNCE_TRACK_INV;
            frame[3] = TX_ANNOUNCE_TRACK_INV;
            frame[4] = TX_ANNOUNCE_SEC_INV;
            frame[5] = TX_ANNOUNCE_F5;
            frame[6] = 0xFF;
            frame[7] = TX_TRL_PLAY;

            // Cycle through magazine slots 0x2E..0x29 (6 discs) then wrap
            s_discAnnounceByte = (s_discAnnounceByte == TX_SLOT_ANNOUNCE_LAST)
                                 ? TX_SLOT_ANNOUNCE_FIRST
                                 : (uint8_t)(s_discAnnounceByte - 1);
        } else {
            frame[0] = TX_HDR_PLAY;
            frame[1] = (uint8_t)(TX_DISC_BASE - disc);
            frame[2] = (uint8_t)(0xFF - track);
            frame[3] = 0xFF;
            frame[4] = 0xFF;
            frame[5] = 0xFF;
            frame[6] = TX_STATUS_LOAD_F6;
            frame[7] = TX_TRL_PLAY;
        }

        if (++s_phaseFrameIndex >= s_phaseFrameTarget) {
            s_sequenceState    = TxSequenceState::LEAD_IN;
            s_phaseFrameIndex  = 0;
            s_phaseFrameTarget = PHASE_FRAMES_LEAD_IN;
        }
        break;

    case TxSequenceState::LEAD_IN:
        if ((s_phaseFrameIndex & 1U) == 0U) {
            frame[0] = TX_HDR_PLAY;
            frame[1] = (uint8_t)((disc & 0x0F) | 0x20);
            frame[2] = TX_ANNOUNCE_TRACK_INV;
            frame[3] = TX_ANNOUNCE_TRACK_INV;
            frame[4] = TX_ANNOUNCE_SEC_INV;
            frame[5] = TX_ANNOUNCE_F5;
            frame[6] = 0xFF;
            frame[7] = TX_TRL_PLAY;
        } else {
            frame[0] = TX_HDR_PLAY;
            frame[1] = (uint8_t)(TX_DISC_BASE - disc);
            frame[2] = (uint8_t)(0xFF - track);
            frame[3] = 0xFF;
            frame[4] = 0xFF;
            frame[5] = 0xFF;
            frame[6] = TX_STATUS_LEADIN_F6;
            frame[7] = TX_TRL_PLAY;
        }

        if (++s_phaseFrameIndex >= s_phaseFrameTarget) {
            s_sequenceState = TxSequenceState::PLAYING;
        }
        break;

    case TxSequenceState::PLAYING:
        frame[0] = TX_HDR_PLAY;
        frame[1] = (uint8_t)(TX_DISC_BASE - disc);
        frame[2] = (uint8_t)(0xFF - toBCD(track));
        frame[3] = (uint8_t)(0xFF - toBCD(s_playTimeMinutes));
        frame[4] = (uint8_t)(0xFF - toBCD(s_playTimeSeconds));
        frame[5] = s_playbackModeByte;
        frame[6] = TX_PLAY_FLAGS_F6;
        frame[7] = TX_TRL_PLAY;
        break;
    }

    portEXIT_CRITICAL(&s_cdcMux);
}

// ============================================================================
//  RX — INTERRUPT SERVICE ROUTINE
// ============================================================================

/**
 * @brief GPIO CHANGE ISR on DATA OUT measuring LOW pulse durations.
 *
 * IRAM_ATTR is mandatory because flash cache is disabled during NVS/OTA writes.
 * UART logging or blocking calls are forbidden here; decoded bytes are pushed
 * into s_rxRing[] for processing inside cdc_loop().
 */
static void IRAM_ATTR dataOutPulseIsr() {
    if (s_pinDataFromRadio < 0) return;

    uint32_t nowUs = micros();
    bool level = (gpio_get_level((gpio_num_t)s_pinDataFromRadio) != 0);

    if (!level) {
        // Falling edge: start measuring LOW pulse
        s_rxFallingEdgeUs  = nowUs;
        s_rxInsideLowPulse = true;
        return;
    }

    // Rising edge: classify completed LOW pulse
    if (!s_rxInsideLowPulse) return;
    uint32_t lowDurationUs = nowUs - s_rxFallingEdgeUs;
    s_rxInsideLowPulse = false;

    if (lowDurationUs < RX_PULSE_MIN_VALID_US) return;

    if (lowDurationUs >= RX_PULSE_MIN_START_US) {
        s_rxFrameInProgress = true;
        s_rxBitsLeftInFrame = RX_FRAME_BITS;
        s_rxBitsLeftInByte  = 8;
        s_rxAssembledByte   = 0;
        return;
    }

    if (!s_rxFrameInProgress) return;

    s_rxAssembledByte = (uint8_t)((s_rxAssembledByte << 1)
                      | ((lowDurationUs >= RX_PULSE_MIN_BIT1_US) ? 1U : 0U));
    s_rxBitsLeftInByte  = (uint8_t)(s_rxBitsLeftInByte - 1U);
    s_rxBitsLeftInFrame = (uint8_t)(s_rxBitsLeftInFrame - 1U);

    if (s_rxBitsLeftInByte == 0) {
        uint8_t nextWriteIdx = (uint8_t)((s_rxWriteIdx + 1U) % RX_RING_SIZE);
        if (nextWriteIdx != s_rxReadIdx) {
            s_rxRing[s_rxWriteIdx] = s_rxAssembledByte;
            s_rxWriteIdx = nextWriteIdx;
        }
        s_rxBitsLeftInByte = 8;
        s_rxAssembledByte  = 0;
    }

    if (s_rxBitsLeftInFrame == 0) {
        s_rxFrameInProgress = false;
    }
}

// ============================================================================
//  RX — TASK-CONTEXT FRAME DECODER
// ============================================================================

/**
 * @brief Scan s_rxRing[] for valid `53 2C <cmd> <~cmd>` command frames.
 */
static void dispatchReceivedCommands() {
    while (true) {
        uint8_t bytesReady =
            (uint8_t)((s_rxWriteIdx - s_rxReadIdx + RX_RING_SIZE) % RX_RING_SIZE);
        if (bytesReady < 4) break;

        uint8_t b0 = s_rxRing[s_rxReadIdx];
        uint8_t b1 = s_rxRing[(s_rxReadIdx + 1U) % RX_RING_SIZE];
        uint8_t b2 = s_rxRing[(s_rxReadIdx + 2U) % RX_RING_SIZE];
        uint8_t b3 = s_rxRing[(s_rxReadIdx + 3U) % RX_RING_SIZE];

        if (b0 != CMD_HEADER_BYTE0 ||
            b1 != CMD_HEADER_BYTE1 ||
            (uint8_t)(b2 + b3) != 0xFF) {
            // Advance by 1 byte to resynchronize on the next valid frame header
            s_rxReadIdx = (uint8_t)((s_rxReadIdx + 1U) % RX_RING_SIZE);
            continue;
        }

        if (b2 == RX_CMD_HOLD_REPEAT || b2 == RX_CMD_DISC_ACK) {
            // Bus protocol chatter (hold auto-repeat / disc handshake ACK)
        } else if (b2 == RX_CMD_RADIO_HELLO) {
            Serial.println("[CDC] RX 0xE4 (radio hello) - restarting handshake");
            portENTER_CRITICAL(&s_cdcMux);
            s_sequenceState    = TxSequenceState::ANNOUNCE_PRESENCE;
            s_phaseFrameIndex  = 0;
            s_phaseFrameTarget = PHASE_FRAMES_ANNOUNCE_HELLO;
            s_discAnnounceByte = TX_SLOT_ANNOUNCE_FIRST;
            portEXIT_CRITICAL(&s_cdcMux);
        } else if (s_commandCallback) {
            // Invoke application handler in CDC task context (for mapped or learned buttons)
            s_commandCallback(b2);
        }

        s_rxReadIdx = (uint8_t)((s_rxReadIdx + 4U) % RX_RING_SIZE);
    }
}

// ============================================================================
//  PUBLIC API
// ============================================================================

void cdc_init(int clockPin, int dataToRadioPin, int dataFromRadioPin,
              CdcCommandCallback onCommand) {
    Serial.printf("[CDC] Init (bit-bang): CLK=%d DATA-IN=%d DATA-OUT=%d\n",
                  clockPin, dataToRadioPin, dataFromRadioPin);

    if (!s_pauseMutex) {
        s_pauseMutex = xSemaphoreCreateMutex();
    }
    s_pauseDepth = 0;

    s_pinClockToRadio  = clockPin;
    s_pinDataToRadio   = dataToRadioPin;
    s_pinDataFromRadio = dataFromRadioPin;
    s_commandCallback  = onCommand;

    pinMode(s_pinClockToRadio, OUTPUT);
    pinMode(s_pinDataToRadio,  OUTPUT);
    digitalWrite(s_pinClockToRadio, LOW);
    digitalWrite(s_pinDataToRadio,  LOW);

    resetRxDecoderState(true);

    if (s_pinDataFromRadio >= 0) {
        pinMode(s_pinDataFromRadio, INPUT);
        attachInterrupt(digitalPinToInterrupt(s_pinDataFromRadio),
                        dataOutPulseIsr, CHANGE);
    }

    portENTER_CRITICAL(&s_cdcMux);
    s_disc             = 1;
    s_track            = 1;
    s_randomOn         = false;
    s_scanOn           = false;
    s_playbackModeByte = TX_MODE_NORMAL;
    s_playTimeMinutes  = 0;
    s_playTimeSeconds  = 0;
    s_lastSecondTickMs = millis();
    s_sequenceState    = TxSequenceState::ANNOUNCE_PRESENCE;
    s_phaseFrameIndex  = 0;
    s_phaseFrameTarget = PHASE_FRAMES_ANNOUNCE_BOOT;
    s_discAnnounceByte = TX_SLOT_ANNOUNCE_FIRST;
    portEXIT_CRITICAL(&s_cdcMux);

    s_lastStatusFrameMs = millis();
    Serial.println("[CDC] Init complete");
}

void cdc_loop() {
    dispatchReceivedCommands();

    uint32_t nowMs = millis();
    if (nowMs - s_lastStatusFrameMs < TX_FRAME_INTERVAL_MS) {
        return;
    }
    s_lastStatusFrameMs = nowMs;

    tickPlayTime(nowMs);

    uint8_t frame[8];
    buildNextTxFrame(frame);
    sendFrameToRadio(frame);
}

void cdc_setDiscTrack(uint8_t disc, uint8_t track) {
    portENTER_CRITICAL(&s_cdcMux);
    s_disc  = disc;
    s_track = track;
    portEXIT_CRITICAL(&s_cdcMux);
}

void cdc_restartInit() {
    uint8_t discSnapshot;
    portENTER_CRITICAL(&s_cdcMux);
    s_sequenceState    = TxSequenceState::LOAD_DISC;
    s_phaseFrameIndex  = 0;
    s_phaseFrameTarget = PHASE_FRAMES_LOAD_RESTART;
    s_discAnnounceByte = TX_SLOT_ANNOUNCE_FIRST;
    s_lastSecondTickMs = millis();
    discSnapshot       = s_disc;
    portEXIT_CRITICAL(&s_cdcMux);

    Serial.printf("[CDC] Restart init -> CD%u\n", discSnapshot);
}

void cdc_setRandom(bool on) {
    portENTER_CRITICAL(&s_cdcMux);
    s_randomOn = on;
    updatePlaybackModeByteLocked();
    portEXIT_CRITICAL(&s_cdcMux);
}

void cdc_setScan(bool on) {
    portENTER_CRITICAL(&s_cdcMux);
    s_scanOn = on;
    updatePlaybackModeByteLocked();
    portEXIT_CRITICAL(&s_cdcMux);
}

void cdc_setPlayTime(uint8_t minutes, uint8_t seconds) {
    if (minutes > 99) minutes = 99;
    if (seconds > 59) seconds = 59;

    portENTER_CRITICAL(&s_cdcMux);
    s_playTimeMinutes  = minutes;
    s_playTimeSeconds  = seconds;
    s_lastSecondTickMs = millis();
    portEXIT_CRITICAL(&s_cdcMux);
}

void cdc_pause(bool pause) {
    if (s_pinDataFromRadio < 0) return;

    if (s_pauseMutex) xSemaphoreTake(s_pauseMutex, portMAX_DELAY);
    if (pause) {
        if (s_pauseDepth == 0) {
            detachInterrupt(digitalPinToInterrupt(s_pinDataFromRadio));
        }
        s_pauseDepth++;
    } else if (s_pauseDepth > 0) {
        s_pauseDepth--;
        if (s_pauseDepth == 0) {
            resetRxDecoderState(false);
            attachInterrupt(digitalPinToInterrupt(s_pinDataFromRadio),
                            dataOutPulseIsr, CHANGE);
        }
    }
    if (s_pauseMutex) xSemaphoreGive(s_pauseMutex);
}
