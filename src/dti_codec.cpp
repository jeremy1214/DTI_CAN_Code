#include "dti_can/dti_codec.hpp"
#include <cmath>
#include <cstring>

namespace DTI {

bool Codec::extractPacketAndNodeId(uint32_t canId, uint8_t& outPacketId, uint8_t& outNodeId) {
    if (canId <= 0xFF) {
        // Raw packet ID passed directly
        outPacketId = static_cast<uint8_t>(canId);
        outNodeId   = DEFAULT_NODE_ID;
        return true;
    }

    // Extended 29-bit CAN ID (DTI / VESC format):
    // bits 0..7: Node ID
    // bits 8..15: Packet ID
    outPacketId = static_cast<uint8_t>((canId >> PACKET_ID_SHIFT) & 0xFF);
    outNodeId   = static_cast<uint8_t>(canId & NODE_ID_MASK);
    return true;
}

bool Codec::decodeGeneralData6(const uint8_t* data, uint8_t dlc, GeneralData6& out) {
    if (!data || dlc < 6) return false;

    // Byte 0: Control Mode
    out.control_mode = static_cast<ControlMode>(data[0]);

    // Bytes 1-2: Target Iq (int16, Scale: 10, Apk)
    int16_t target_iq_raw = readInt16BE(&data[1]);
    out.target_iq_apk = static_cast<float>(target_iq_raw) / 10.0f;

    // Bytes 3-4: Motor Position (uint16, Scale: 10, degree)
    uint16_t motor_pos_raw = readUint16BE(&data[3]);
    out.motor_position_deg = static_cast<float>(motor_pos_raw) / 10.0f;

    // Byte 5: isMotorStill (1: Still, 0: Rotating)
    out.is_motor_still = (data[5] != 0) ? MotorStillState::STILL : MotorStillState::ROTATING;

    // Bytes 6-7: Reserved
    if (dlc >= 8) {
        out.reserved = readUint16BE(&data[6]);
    } else {
        out.reserved = 0xFFFF;
    }

    return true;
}

bool Codec::decodeGeneralData1(const uint8_t* data, uint8_t dlc, GeneralData1& out) {
    if (!data || dlc < 8) return false;

    // Bytes 0-3: Real ERPM (int32, Scale: 1, RPM)
    out.real_erpm = readInt32BE(&data[0]);

    // Bytes 4-5: Duty Cycle (int16, Scale: 10, %)
    int16_t duty_raw = readInt16BE(&data[4]);
    out.duty_cycle_pct = static_cast<float>(duty_raw) / 10.0f;

    // Bytes 6-7: Input Voltage (int16, Scale: 1, V)
    int16_t volt_raw = readInt16BE(&data[6]);
    out.input_voltage_v = static_cast<float>(volt_raw);

    return true;
}

bool Codec::decodeGeneralData2(const uint8_t* data, uint8_t dlc, GeneralData2& out) {
    if (!data || dlc < 4) return false;

    // Bytes 0-1: AC Current (int16, Scale: 10, Apk)
    int16_t ac_raw = readInt16BE(&data[0]);
    out.ac_current_apk = static_cast<float>(ac_raw) / 10.0f;

    // Bytes 2-3: DC Current (int16, Scale: 10, Adc)
    int16_t dc_raw = readInt16BE(&data[2]);
    out.dc_current_adc = static_cast<float>(dc_raw) / 10.0f;

    // Bytes 4-7: Reserved (uint32)
    if (dlc >= 8) {
        out.reserved = readUint32BE(&data[4]);
    } else {
        out.reserved = 0xFFFFFFFF;
    }

    return true;
}

bool Codec::decodeGeneralData3(const uint8_t* data, uint8_t dlc, GeneralData3& out) {
    if (!data || dlc < 5) return false;

    // Bytes 0-1: Controller Temp (int16, Scale: 10, °C)
    int16_t ctrl_temp_raw = readInt16BE(&data[0]);
    out.controller_temp_c = static_cast<float>(ctrl_temp_raw) / 10.0f;

    // Bytes 2-3: Motor Temp (int16, Scale: 10, °C)
    int16_t motor_temp_raw = readInt16BE(&data[2]);
    out.motor_temp_c = static_cast<float>(motor_temp_raw) / 10.0f;

    // Byte 4: Fault Code
    out.fault_code = static_cast<FaultCode>(data[4]);

    // Bytes 5-7: Reserved (uint24)
    if (dlc >= 8) {
        out.reserved = readUint24BE(&data[5]);
    } else {
        out.reserved = 0xFFFFFF;
    }

    return true;
}

bool Codec::decodeGeneralData4(const uint8_t* data, uint8_t dlc, GeneralData4& out) {
    if (!data || dlc < 8) return false;

    // Bytes 0-3: Id Current (int32, Scale: 100, Apk)
    int32_t id_raw = readInt32BE(&data[0]);
    out.id_current_apk = static_cast<float>(id_raw) / 100.0f;

    // Bytes 4-7: Iq Current (int32, Scale: 100, Apk)
    int32_t iq_raw = readInt32BE(&data[4]);
    out.iq_current_apk = static_cast<float>(iq_raw) / 100.0f;

    return true;
}

bool Codec::decodeGeneralData5(const uint8_t* data, uint8_t dlc, GeneralData5& out) {
    if (!data || dlc < 8) return false;

    // Byte 0: Throttle Signal (int8, Scale: 1, %)
    out.throttle_pct = static_cast<int8_t>(data[0]);

    // Byte 1: Brake Signal (int8, Scale: 1, %)
    out.brake_pct = static_cast<int8_t>(data[1]);

    // Byte 2: Digital IOs (uint8, Bitmask)
    out.digital_ios = data[2];

    // Byte 3: Drive Enable Status (1: Driven)
    out.drive_enable_status = (data[3] == 1) ? DriveEnableState::ENABLED : DriveEnableState::DISABLED;

    // Bytes 4-5: Limit Status Bits (uint16)
    out.limit_status_bits = readUint16BE(&data[4]);

    // Byte 6: Reserved
    // Byte 7: CAN Map Version
    out.can_map_version = data[7];

    return true;
}

bool Codec::decodeConfigAvailAC(const uint8_t* data, uint8_t dlc, ConfigAvailAC& out) {
    if (!data || dlc < 8) return false;

    out.max_ac_configured_apk = static_cast<float>(readInt16BE(&data[0])) / 10.0f;
    out.max_ac_available_apk  = static_cast<float>(readInt16BE(&data[2])) / 10.0f;
    out.min_ac_configured_apk = static_cast<float>(readInt16BE(&data[4])) / 10.0f;
    out.min_ac_available_apk  = static_cast<float>(readInt16BE(&data[6])) / 10.0f;

    return true;
}

bool Codec::decodeConfigAvailDC(const uint8_t* data, uint8_t dlc, ConfigAvailDC& out) {
    if (!data || dlc < 8) return false;

    out.max_dc_configured_adc = static_cast<float>(readInt16BE(&data[0])) / 10.0f;
    out.max_dc_available_adc  = static_cast<float>(readInt16BE(&data[2])) / 10.0f;
    out.min_dc_configured_adc = static_cast<float>(readInt16BE(&data[4])) / 10.0f;
    out.min_dc_available_adc  = static_cast<float>(readInt16BE(&data[6])) / 10.0f;

    return true;
}

bool Codec::decodeTelemetryFrame(
    const CANFrame& frame,
    InverterPacketId& outPacketId,
    uint8_t& outNodeId,
    TelemetryPacket& outPacket
) {
    uint8_t pktIdRaw = 0;
    if (!extractPacketAndNodeId(frame.id, pktIdRaw, outNodeId)) {
        return false;
    }

    outPacketId = static_cast<InverterPacketId>(pktIdRaw);

    switch (outPacketId) {
        case InverterPacketId::GENERAL_DATA_6: {
            GeneralData6 gd6;
            gd6.timestamp_us = frame.timestamp_us;
            if (decodeGeneralData6(frame.data, frame.dlc, gd6)) {
                outPacket = gd6;
                return true;
            }
            break;
        }
        case InverterPacketId::GENERAL_DATA_1: {
            GeneralData1 gd1;
            gd1.timestamp_us = frame.timestamp_us;
            if (decodeGeneralData1(frame.data, frame.dlc, gd1)) {
                outPacket = gd1;
                return true;
            }
            break;
        }
        case InverterPacketId::GENERAL_DATA_2: {
            GeneralData2 gd2;
            gd2.timestamp_us = frame.timestamp_us;
            if (decodeGeneralData2(frame.data, frame.dlc, gd2)) {
                outPacket = gd2;
                return true;
            }
            break;
        }
        case InverterPacketId::GENERAL_DATA_3: {
            GeneralData3 gd3;
            gd3.timestamp_us = frame.timestamp_us;
            if (decodeGeneralData3(frame.data, frame.dlc, gd3)) {
                outPacket = gd3;
                return true;
            }
            break;
        }
        case InverterPacketId::GENERAL_DATA_4: {
            GeneralData4 gd4;
            gd4.timestamp_us = frame.timestamp_us;
            if (decodeGeneralData4(frame.data, frame.dlc, gd4)) {
                outPacket = gd4;
                return true;
            }
            break;
        }
        case InverterPacketId::GENERAL_DATA_5: {
            GeneralData5 gd5;
            gd5.timestamp_us = frame.timestamp_us;
            if (decodeGeneralData5(frame.data, frame.dlc, gd5)) {
                outPacket = gd5;
                return true;
            }
            break;
        }
        case InverterPacketId::CONFIG_AVAIL_AC: {
            ConfigAvailAC ac;
            ac.timestamp_us = frame.timestamp_us;
            if (decodeConfigAvailAC(frame.data, frame.dlc, ac)) {
                outPacket = ac;
                return true;
            }
            break;
        }
        case InverterPacketId::CONFIG_AVAIL_DC: {
            ConfigAvailDC dc;
            dc.timestamp_us = frame.timestamp_us;
            if (decodeConfigAvailDC(frame.data, frame.dlc, dc)) {
                outPacket = dc;
                return true;
            }
            break;
        }
        default:
            break;
    }

    outPacketId = InverterPacketId::UNKNOWN;
    return false;
}

// ============================================================================
// VCU Command Encoders
// ============================================================================

CANFrame Codec::encodeSetAcCurrent(float currentApk, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::SET_AC_CURRENT), nodeId);
    f.dlc = 2;
    int16_t raw = static_cast<int16_t>(std::round(currentApk * 10.0f));
    writeInt16BE(&f.data[0], raw);
    return f;
}

CANFrame Codec::encodeSetBrakeCurrent(float brakeApk, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::SET_BRAKE_CURRENT), nodeId);
    f.dlc = 2;
    int16_t raw = static_cast<int16_t>(std::round(brakeApk * 10.0f));
    writeInt16BE(&f.data[0], raw);
    return f;
}

CANFrame Codec::encodeSetErpm(int32_t erpm, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::SET_ERPM), nodeId);
    f.dlc = 4;
    writeInt32BE(&f.data[0], erpm);
    return f;
}

CANFrame Codec::encodeSetRelativeCurrent(float percent, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::SET_RELATIVE_CURRENT), nodeId);
    f.dlc = 2;
    int16_t raw = static_cast<int16_t>(std::round(percent * 10.0f));
    writeInt16BE(&f.data[0], raw);
    return f;
}

CANFrame Codec::encodeSetDigitalOutput(uint8_t doBitmask, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::SET_DIGITAL_OUTPUT), nodeId);
    f.dlc = 1;
    f.data[0] = doBitmask;
    return f;
}

CANFrame Codec::encodeSetMaxAcCurrent(float maxAcApk, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::SET_MAX_AC_CURRENT), nodeId);
    f.dlc = 2;
    int16_t raw = static_cast<int16_t>(std::round(maxAcApk * 10.0f));
    writeInt16BE(&f.data[0], raw);
    return f;
}

CANFrame Codec::encodeSetMaxDcCurrent(float maxDcAdc, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::SET_MAX_DC_CURRENT), nodeId);
    f.dlc = 2;
    int16_t raw = static_cast<int16_t>(std::round(maxDcAdc * 10.0f));
    writeInt16BE(&f.data[0], raw);
    return f;
}

CANFrame Codec::encodeSetMaxDcBrakeCurrent(float maxDcBrakeAdc, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::SET_MAX_DC_BRAKE_CURRENT), nodeId);
    f.dlc = 2;
    int16_t raw = static_cast<int16_t>(std::round(maxDcBrakeAdc * 10.0f));
    writeInt16BE(&f.data[0], raw);
    return f;
}

CANFrame Codec::encodeDriveEnable(bool enable, uint8_t nodeId) {
    CANFrame f{};
    f.id = makeExtendedCanId(static_cast<uint8_t>(VcuPacketId::DRIVE_ENABLE), nodeId);
    f.dlc = 1;
    f.data[0] = enable ? 1 : 0;
    return f;
}

bool Codec::decodeVcuCommand(const CANFrame& frame, VcuCommandData& outCmd) {
    uint8_t pktIdRaw = 0;
    uint8_t nodeId   = 0;
    if (!extractPacketAndNodeId(frame.id, pktIdRaw, nodeId)) {
        return false;
    }

    outCmd.packet_id = static_cast<VcuPacketId>(pktIdRaw);
    outCmd.node_id   = nodeId;

    switch (outCmd.packet_id) {
        case VcuPacketId::SET_AC_CURRENT:
            if (frame.dlc < 2) return false;
            outCmd.value.target_ac_current_apk = static_cast<float>(readInt16BE(&frame.data[0])) / 10.0f;
            return true;
        case VcuPacketId::SET_BRAKE_CURRENT:
            if (frame.dlc < 2) return false;
            outCmd.value.target_brake_current_apk = static_cast<float>(readInt16BE(&frame.data[0])) / 10.0f;
            return true;
        case VcuPacketId::SET_ERPM:
            if (frame.dlc < 4) return false;
            outCmd.value.target_erpm = readInt32BE(&frame.data[0]);
            return true;
        case VcuPacketId::SET_RELATIVE_CURRENT:
            if (frame.dlc < 2) return false;
            outCmd.value.target_relative_current = static_cast<float>(readInt16BE(&frame.data[0])) / 10.0f;
            return true;
        case VcuPacketId::SET_DIGITAL_OUTPUT:
            if (frame.dlc < 1) return false;
            outCmd.value.digital_output_mask = frame.data[0];
            return true;
        case VcuPacketId::SET_MAX_AC_CURRENT:
            if (frame.dlc < 2) return false;
            outCmd.value.max_ac_current_limit_apk = static_cast<float>(readInt16BE(&frame.data[0])) / 10.0f;
            return true;
        case VcuPacketId::SET_MAX_DC_CURRENT:
            if (frame.dlc < 2) return false;
            outCmd.value.max_dc_current_limit_adc = static_cast<float>(readInt16BE(&frame.data[0])) / 10.0f;
            return true;
        case VcuPacketId::SET_MAX_DC_BRAKE_CURRENT:
            if (frame.dlc < 2) return false;
            outCmd.value.max_dc_brake_limit_adc = static_cast<float>(readInt16BE(&frame.data[0])) / 10.0f;
            return true;
        case VcuPacketId::DRIVE_ENABLE:
            if (frame.dlc < 1) return false;
            outCmd.value.drive_enable_cmd = frame.data[0];
            return true;
        default:
            return false;
    }
}

} // namespace DTI
