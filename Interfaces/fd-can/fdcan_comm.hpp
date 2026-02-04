/*
 * ==============================================================================
 * FDCAN Communication Library
 * Copyright (c) 2026 Gaurav Dattatray Tale / Xkalp. All rights reserved.
 * Portions Copyright (c) 2024 George Yuanji Wang / I-M-Robotics-Lab.
 *
 * This software is provided under the terms of the GNU General Public License
 * v3.0 (GPL-3.0). Any redistribution of this source code or binary forms must
 * retain the above copyright notice and this attribution without modification.
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions, and the following disclaimer.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.
 * ==============================================================================
 */

#ifndef FD_CAN_FDCAN_COMM_HPP_
#define FD_CAN_FDCAN_COMM_HPP_

#include <cstdint>
#include <cstring>
#include "fdcan.h"

/* FDCAN Configuration Constants */
#define FDCAN_MAX_PAYLOAD_SIZE    64U

/* FDCAN Callback Types */
typedef void (*FDCANRxCallback)(uint32_t id, const uint8_t* data, uint32_t length);

/* FDCAN Error Status Enum */
enum FDCANError {
    FDCAN_ERR_NONE    = 0,   /* No error */
    FDCAN_ERR_WARNING = 1,   /* Error warning state (TEC/REC > 96) */
    FDCAN_ERR_PASSIVE = 2,   /* Error passive state (TEC/REC > 127) */
    FDCAN_BUS_OFF    = 3     /* Bus-off state - requires restart (TEC > 255) */
};

/**
 * @struct FDCANConfig
 * @brief Structure for consolidating all FDCAN setup parameters.
 */
struct FDCANConfig {
    // RX Filter Configuration (Standard ID only for simplicity)
    uint32_t filter_id1;
    uint32_t filter_id2;
    uint32_t filter_type; // FDCAN_FILTER_DUAL / MASK / RANGE

    // TX Header Configuration
    bool brs_on;
};

/**
 * @class FDCANCommunication
 * @brief FDCAN communication library for STM32G4 with bus-off recovery and callbacks.
 */
class FDCANCommunication {
public:
    /**
     * @brief Constructor - Requires the FDCAN handle pointer.
     */
    FDCANCommunication(FDCAN_HandleTypeDef* handle);

    /**
     * @brief Initialize FDCAN, set up filters, and start peripheral.
     * @param config Configuration structure for filters and TX header defaults.
     * @param rx_cb RX callback (called when message received).
     * @return true if successful, false on error.
     */
    bool init(const FDCANConfig& config, FDCANRxCallback rx_cb);

    /**
     * @brief Deinitialize FDCAN peripheral.
     * @return true if successful.
     */
    bool deinit(void);

    /**
     * @brief Transmit FDCAN message (non-blocking). Thread-safe.
     * * @param id CAN message ID (11-bit standard: 0x000-0x7FF).
     * @param data Data buffer pointer.
     * @param length Data length (0-64 bytes, auto-capped).
     * @return true if transmission queued successfully.
     */
    bool transmit(uint32_t id, const uint8_t* data, uint32_t length);

    /**
     * @brief Get current error status.
     * @return FDCANError enum value (prioritized: BUS_OFF > PASSIVE > WARNING > NONE).
     */
    FDCANError getErrorStatus(void);

    /**
     * @brief Check if in bus-off state.
     * @return true if bus-off, false otherwise.
     */
    bool isBusOff(void) const;

    /**
     * @brief Recover from bus-off condition.
     * @return true if recovery successful.
     */
    bool recoverFromBusOff(void);

private:
    // Friend functions for HAL callbacks to access private ISR methods
    friend void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs);
    // --- Internal ISR Processing (Called by HAL Callbacks) ---
    // Note: These must remain accessible to HAL interrupt handlers
    void processRxFifo(void);
    static uint32_t bytesToDlc(uint8_t size);
    static uint32_t dlcToBytes(uint8_t dlc);
    static FDCANError decodeErrorStatus(uint32_t psr);

    // Helper to configure filter (used internally by init/recover)
    bool configRxFilter(const FDCANConfig& config);

    // Helper to configure TX header (used internally by init/recover)
    void configTxHeader(const FDCANConfig& config);

    FDCAN_HandleTypeDef* const hfdcan; // Now const, initialized in constructor
    FDCANRxCallback rx_callback;
    FDCAN_TxHeaderTypeDef tx_header_cache; // Cached TX header
    bool is_initialized;
    bool is_bus_off;
    FDCANConfig active_config; // Store configuration for restart/recovery
};

/* Global instance for interrupt handler access - requires manual linkage */
// The user must define this instance: e.g., FDCANCommunication g_fdcan_comm(&hfdcan2);
extern FDCANCommunication g_fdcan_comm;

#endif /* FD_CAN_FDCAN_COMM_HPP_ */
