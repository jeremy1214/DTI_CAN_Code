#pragma once

#include "dti_types.hpp"
#include <variant>

namespace DTI {

// Variant type holding any decoded telemetry packet
using TelemetryPacket = std::variant<
    GeneralData6,
    GeneralData1,
    GeneralData2,
    GeneralData3,
    GeneralData4,
    GeneralData5,
    ConfigAvailAC,
    ConfigAvailDC
>;

/**
 * @brief Stateless CAN Codec for DTI Inverter & VCU communication
 * 
 * Provides pure static functions for encoding and decoding CAN frames according
 * to the DTI Inverter CAN Matrix V2 specification.
 */
class Codec {
public:
    // ========================================================================
    // CAN ID Parser & Formatter
    // ========================================================================

    /**
     * @brief Extract Packet ID and Node ID from incoming CAN ID.
     * 
     * Supports:
     * - Extended 29-bit CAN ID (DTI/VESC standard): (Packet_ID << 8) | Node_ID
     * - Standard 11-bit CAN ID: (Packet_ID << 5) | Node_ID
     * - Raw Packet ID: if can_id <= 0xFF, treats can_id as Packet_ID
     */
    static bool extractPacketAndNodeId(uint32_t canId, uint8_t& outPacketId, uint8_t& outNodeId);

    /**
     * @brief Build 29-bit Extended CAN ID from Packet ID and Inverter Node ID.
     */
    static constexpr uint32_t makeExtendedCanId(uint8_t packetId, uint8_t nodeId) {
        return (static_cast<uint32_t>(packetId) << PACKET_ID_SHIFT) | static_cast<uint32_t>(nodeId);
    }

    // ========================================================================
    // Inverter Telemetry Frame Decoding
    // ========================================================================

    /**
     * @brief Decode any incoming CAN frame into corresponding telemetry packet struct.
     * @param frame Raw CAN frame
     * @param outPacketId Identified InverterPacketId
     * @param outNodeId Extracted Node ID
     * @param outPacket Variant holding decoded structure
     * @return true if successfully decoded, false if unknown packet or invalid DLC
     */
    static bool decodeTelemetryFrame(
        const CANFrame& frame,
        InverterPacketId& outPacketId,
        uint8_t& outNodeId,
        TelemetryPacket& outPacket
    );

    // Individual Telemetry Packet Decoders
    static bool decodeGeneralData6(const uint8_t* data, uint8_t dlc, GeneralData6& out);
    static bool decodeGeneralData1(const uint8_t* data, uint8_t dlc, GeneralData1& out);
    static bool decodeGeneralData2(const uint8_t* data, uint8_t dlc, GeneralData2& out);
    static bool decodeGeneralData3(const uint8_t* data, uint8_t dlc, GeneralData3& out);
    static bool decodeGeneralData4(const uint8_t* data, uint8_t dlc, GeneralData4& out);
    static bool decodeGeneralData5(const uint8_t* data, uint8_t dlc, GeneralData5& out);
    static bool decodeConfigAvailAC(const uint8_t* data, uint8_t dlc, ConfigAvailAC& out);
    static bool decodeConfigAvailDC(const uint8_t* data, uint8_t dlc, ConfigAvailDC& out);

    // ========================================================================
    // VCU Command Frame Encoding (VCU -> Inverter)
    // ========================================================================

    // 0x01: Set AC Current (int16, Scale: 10, Apk)
    static CANFrame encodeSetAcCurrent(float currentApk, uint8_t nodeId = DEFAULT_NODE_ID);

    // 0x02: Set Brake Current (int16, Scale: 10, Apk)
    static CANFrame encodeSetBrakeCurrent(float brakeApk, uint8_t nodeId = DEFAULT_NODE_ID);

    // 0x03: Set ERPM (int32, Scale: 1, RPM)
    static CANFrame encodeSetErpm(int32_t erpm, uint8_t nodeId = DEFAULT_NODE_ID);

    // 0x05: Set Relative Current (int16, Scale: 10, %)
    static CANFrame encodeSetRelativeCurrent(float percent, uint8_t nodeId = DEFAULT_NODE_ID);

    // 0x07: Set Digital Output (uint8, Bitmask DO1~DO4)
    static CANFrame encodeSetDigitalOutput(uint8_t doBitmask, uint8_t nodeId = DEFAULT_NODE_ID);

    // 0x08: Set Max AC Current Limit (int16, Scale: 10, Apk)
    static CANFrame encodeSetMaxAcCurrent(float maxAcApk, uint8_t nodeId = DEFAULT_NODE_ID);

    // 0x0A: Set Max DC Current Limit (int16, Scale: 10, Adc)
    static CANFrame encodeSetMaxDcCurrent(float maxDcAdc, uint8_t nodeId = DEFAULT_NODE_ID);

    // 0x0B: Set Max DC Brake Current Limit (int16, Scale: 10, Adc)
    static CANFrame encodeSetMaxDcBrakeCurrent(float maxDcBrakeAdc, uint8_t nodeId = DEFAULT_NODE_ID);

    // 0x0C: Drive Enable (uint8, 1: Enable, 0: Disable)
    static CANFrame encodeDriveEnable(bool enable, uint8_t nodeId = DEFAULT_NODE_ID);

    // ========================================================================
    // VCU Command Frame Decoding (For Bus Monitoring / Inverter Emulation)
    // ========================================================================
    static bool decodeVcuCommand(const CANFrame& frame, VcuCommandData& outCmd);

    // ========================================================================
    // Low-Level Endianness Utilities
    // ========================================================================
    static inline int16_t readInt16BE(const uint8_t* d) {
        return static_cast<int16_t>((static_cast<uint16_t>(d[0]) << 8) | static_cast<uint16_t>(d[1]));
    }
    static inline uint16_t readUint16BE(const uint8_t* d) {
        return (static_cast<uint16_t>(d[0]) << 8) | static_cast<uint16_t>(d[1]);
    }
    static inline int32_t readInt32BE(const uint8_t* d) {
        return static_cast<int32_t>(
            (static_cast<uint32_t>(d[0]) << 24) |
            (static_cast<uint32_t>(d[1]) << 16) |
            (static_cast<uint32_t>(d[2]) << 8)  |
            (static_cast<uint32_t>(d[3]))
        );
    }
    static inline uint32_t readUint32BE(const uint8_t* d) {
        return (static_cast<uint32_t>(d[0]) << 24) |
               (static_cast<uint32_t>(d[1]) << 16) |
               (static_cast<uint32_t>(d[2]) << 8)  |
               (static_cast<uint32_t>(d[3]));
    }
    static inline uint32_t readUint24BE(const uint8_t* d) {
        return (static_cast<uint32_t>(d[0]) << 16) |
               (static_cast<uint32_t>(d[1]) << 8)  |
               (static_cast<uint32_t>(d[2]));
    }

    static inline void writeInt16BE(uint8_t* d, int16_t val) {
        d[0] = static_cast<uint8_t>((static_cast<uint16_t>(val) >> 8) & 0xFF);
        d[1] = static_cast<uint8_t>(static_cast<uint16_t>(val) & 0xFF);
    }
    static inline void writeInt32BE(uint8_t* d, int32_t val) {
        d[0] = static_cast<uint8_t>((static_cast<uint32_t>(val) >> 24) & 0xFF);
        d[1] = static_cast<uint8_t>((static_cast<uint32_t>(val) >> 16) & 0xFF);
        d[2] = static_cast<uint8_t>((static_cast<uint32_t>(val) >> 8) & 0xFF);
        d[3] = static_cast<uint8_t>(static_cast<uint32_t>(val) & 0xFF);
    }
};

} // namespace DTI
