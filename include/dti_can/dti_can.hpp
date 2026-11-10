#pragma once

/**
 * @file dti_can.hpp
 * @brief Master header for DTI Inverter CAN Codec & Telemetry Logging Library
 * 
 * Based on DTI Inverter CAN Communication Matrix V2 specification.
 * Provides:
 * - CAN frame encoding & decoding (Codec)
 * - Real-time state aggregation & event callbacks (InverterReceiver)
 * - Telemetry & raw CAN CSV logging & terminal dashboard (Logger)
 */

#include "dti_types.hpp"
#include "dti_codec.hpp"
#include "dti_receiver.hpp"
#include "dti_logger.hpp"
