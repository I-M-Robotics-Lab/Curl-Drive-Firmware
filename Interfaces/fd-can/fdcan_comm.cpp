
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

#include "fdcan_comm.hpp"
FDCANCommunication g_fdcan_comm(&hfdcan2); // one true definition

/* ============================================================================
 * Constructor
 * ============================================================================ */

FDCANCommunication::FDCANCommunication(FDCAN_HandleTypeDef* handle)
  : hfdcan(handle),
    rx_callback(nullptr),
    is_initialized(false),
    is_bus_off(false)
{
    // Ensure the handle is valid before proceeding
    if (hfdcan == nullptr) {
        // Log error or set a flag if logging/error reporting is available
    }
    std::memset(&tx_header_cache, 0, sizeof(tx_header_cache));
    std::memset(&active_config, 0, sizeof(active_config));
}

/* ============================================================================
 * Initialization
 * ============================================================================ */

bool FDCANCommunication::init(const FDCANConfig& config, FDCANRxCallback rx_cb)
{
    if (hfdcan == nullptr) return false;

    /* Validate inputs */
    if (rx_cb == nullptr) {
        return false;
    }

    /* Validate config */
    if (config.filter_type != FDCAN_FILTER_DUAL && config.filter_type != FDCAN_FILTER_MASK && config.filter_type != FDCAN_FILTER_RANGE) {
        return false;
    }

    /* Store callbacks and configuration */
    rx_callback = rx_cb;
    active_config = config;

    /* 1. Configure Filter */
    if (!configRxFilter(config)) {
        return false;
    }

    /* 2. Configure TX Header Cache */
    configTxHeader(config);

    /* 3. Enable Essential Interrupts */
    // Only RX interrupt needed - errors handled by polling
    if (HAL_FDCAN_ActivateNotification(hfdcan,
                                       FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                       0U) != HAL_OK) {
        return false;
    }

    /* 4. Start FDCAN */
    if (HAL_FDCAN_Start(hfdcan) != HAL_OK) {
        return false;
    }

    is_initialized = true;
    is_bus_off = false;

    return true;
}

/* ============================================================================
 * Deinitialization
 * ============================================================================ */

bool FDCANCommunication::deinit(void)
{
    if (!is_initialized || hfdcan == nullptr) {
        return true; // Already deinitialized or invalid handle
    }

    /* Deactivate all active notifications */
    /* Deactivate specific notifications (fixed syntax) */
    if (HAL_FDCAN_DeactivateNotification(hfdcan,
                                    FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != HAL_OK) {
        // Log error, but continue attempting to stop
    }

    /* Stop FDCAN */
    if (HAL_FDCAN_Stop(hfdcan) != HAL_OK) {
        return false;
    }

    is_initialized = false;
    return true;
}

/*=============================================================================
 * Configuration Helpers (Private)
 ============================================================================== */

bool FDCANCommunication::configRxFilter(const FDCANConfig& config)
{
    FDCAN_FilterTypeDef filter_config = {0};

    /* Configure RX Filter */
    filter_config.IdType = FDCAN_STANDARD_ID;
    filter_config.FilterIndex = 0;
    filter_config.FilterType = config.filter_type;
    filter_config.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter_config.FilterID1 = config.filter_id1;
    filter_config.FilterID2 = config.filter_id2;

    if (HAL_FDCAN_ConfigFilter(hfdcan, &filter_config) != HAL_OK) {
        return false;
    }

    /* Configure Global Filter: reject non-matching frames */
    if (HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                      FDCAN_REJECT, // Reject non-matching standard frames
                                      FDCAN_REJECT, // Reject non-matching extended frames
                                      FDCAN_FILTER_REMOTE, // Filter remote frames from standard
                                      FDCAN_FILTER_REMOTE) != HAL_OK) { // Filter remote frames from extended
        return false;
    }

    return true;
}

void FDCANCommunication::configTxHeader(const FDCANConfig& config)
{
    /* Cache static TX header fields. */
    tx_header_cache = (FDCAN_TxHeaderTypeDef){0};
    tx_header_cache.IdType = FDCAN_STANDARD_ID;
    tx_header_cache.TxFrameType = FDCAN_DATA_FRAME;
    tx_header_cache.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header_cache.BitRateSwitch = config.brs_on ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
    tx_header_cache.FDFormat = FDCAN_FD_CAN;
    tx_header_cache.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
    tx_header_cache.MessageMarker = 0U;
    // Identifier and DataLength are set per-message in transmit()
}

/* ============================================================================
 * Transmission (Thread-Safe)
 * ============================================================================ */

bool FDCANCommunication::transmit(uint32_t id, const uint8_t* data, uint32_t length)
{
    if (!is_initialized || data == nullptr || hfdcan == nullptr || length > FDCAN_MAX_PAYLOAD_SIZE) {
        return false;
    }

    /* Create local copy of cached header to avoid race conditions
     * if transmit() is called concurrently from multiple contexts (e.g., main loop and ISR).
     */
    FDCAN_TxHeaderTypeDef tx_header_local = tx_header_cache;

    /* Update per-message fields */
    // Note: Standard ID is 11 bits (0x7FF)
    tx_header_local.Identifier = id & 0x7FFU;
    tx_header_local.DataLength = bytesToDlc(length);

    /* Add message to TX FIFO */
    if (HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header_local, (uint8_t*)data) != HAL_OK) {
        return false;
    }

    return true;
}

/* ============================================================================
 * Error Monitoring and Recovery
 * ============================================================================ */

FDCANError FDCANCommunication::decodeErrorStatus(uint32_t psr)
{
    /* Check errors in priority order (most severe first) */
    if ((psr & FDCAN_PSR_BO) != 0U) {
        return FDCAN_BUS_OFF;
    }
    if ((psr & FDCAN_PSR_EP) != 0U) {
        return FDCAN_ERR_PASSIVE;
    }
    if ((psr & FDCAN_PSR_EW) != 0U) {
        return FDCAN_ERR_WARNING;
    }

    return FDCAN_ERR_NONE;
}

FDCANError FDCANCommunication::getErrorStatus(void)
{
    if (hfdcan == nullptr || hfdcan->Instance == nullptr) return FDCAN_ERR_NONE;
    uint32_t psr = hfdcan->Instance->PSR;
    return decodeErrorStatus(psr);
}

bool FDCANCommunication::isBusOff(void) const
{
    if (hfdcan == nullptr || hfdcan->Instance == nullptr) return false;
    // Check internal state OR PSR register
    return is_bus_off || ((hfdcan->Instance->PSR & FDCAN_PSR_BO) != 0U);
}

bool FDCANCommunication::recoverFromBusOff(void)
{
    if (!isBusOff()) {
        return true;  /* Not in bus-off */
    }

    /* 1. Stop FDCAN */
    if (!deinit()) {
        return false;
    }

    /* 2. Wait for bus recovery (required ~1.3ms for a 500kbit/s bus, using 200ms is safe) */
    HAL_Delay(200);

    /* 3. Restart FDCAN using stored config and callbacks */
    if (!init(active_config, rx_callback)) {
        return false;
    }

    is_bus_off = false;
    return true;
}

/* ============================================================================
 * Helper Functions (Static)
 * ============================================================================ */

uint32_t FDCANCommunication::bytesToDlc(uint8_t size)
{
    /* Convert byte count to DLC value (Optimized for standard lengths) */
    if (size <= 8U) return size;
    if (size <= 12U) return FDCAN_DLC_BYTES_12;
    if (size <= 16U) return FDCAN_DLC_BYTES_16;
    if (size <= 20U) return FDCAN_DLC_BYTES_20;
    if (size <= 24U) return FDCAN_DLC_BYTES_24;
    if (size <= 32U) return FDCAN_DLC_BYTES_32;
    if (size <= 48U) return FDCAN_DLC_BYTES_48;
    return FDCAN_DLC_BYTES_64;
}

uint32_t FDCANCommunication::dlcToBytes(uint8_t dlc)
{
    /* Lookup table for DLC to bytes conversion (CAN-FD spec) */
    static const uint32_t DLC_BYTES_TABLE[16U] = {
        0U,  1U,  2U,  3U,   4U,   5U,   6U,   7U,
        8U,  12U, 16U, 20U,  24U,  32U,  48U,  64U
    };

    if (dlc >= 16U) {
        return 0U;
    }

    return DLC_BYTES_TABLE[dlc];
}

/* ============================================================================
 * Interrupt Processing
 * ============================================================================ */

void FDCANCommunication::processRxFifo(void)
{
    if (rx_callback == nullptr || hfdcan == nullptr || hfdcan->Instance == nullptr) {
        return;
    }

    uint32_t id = 0U;
    uint8_t data[FDCAN_MAX_PAYLOAD_SIZE] = {0};
    uint32_t length = 0U;

    /* Process all available messages in RX FIFO0 */
    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) != 0U) {

        FDCAN_RxHeaderTypeDef rx_header = {0};

        /* Get message from RX FIFO0 - pass user buffer directly to avoid extra copy */
        if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rx_header, data) != HAL_OK) {
            // Error occurred - continue processing other messages
            continue;
        }

        /* Extract and decode message */
        id = rx_header.Identifier;
        // Convert DLC code to byte count
        length = dlcToBytes((uint8_t)rx_header.DataLength);

        if (length > FDCAN_MAX_PAYLOAD_SIZE) {
            length = FDCAN_MAX_PAYLOAD_SIZE;
        }

        /* Invoke user callback with received data */
        rx_callback(id, data, length);
    }
}

/* ============================================================================
 * HAL Callback Implementations (C linkage)
 * ============================================================================ */

/**
 * @brief RX FIFO0 callback - called by HAL_FDCAN_IRQHandler
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs)
{
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != 0U) {
        g_fdcan_comm.processRxFifo();
    }
}


