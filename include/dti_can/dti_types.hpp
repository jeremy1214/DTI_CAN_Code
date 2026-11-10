#pragma once

#include <cstdint>
#include <string>
#include <chrono>

namespace DTI {

// ============================================================================
// CAN Protocol Constants
// ============================================================================

// Default Node ID for DTI Inverter (typically 0x0F or configured in DTI Tool)
constexpr uint8_t DEFAULT_NODE_ID = 0x0F;

// CAN ID Bitfield masks for DTI / VESC extended 29-bit CAN ID:
// CAN_ID = (Packet_ID << 8) | Node_ID
constexpr uint32_t PACKET_ID_SHIFT = 8;
constexpr uint32_t NODE_ID_MASK    = 0x00FF;
constexpr uint32_t PACKET_ID_MASK  = 0xFF00;

// ============================================================================
// VCU Command Packet IDs (Sheet 1: VCU_Tx_Commands)
// ============================================================================
enum class VcuPacketId : uint8_t {
    SET_AC_CURRENT              = 0x01, // 0x01: Target AC Current (2 Bytes, int16, Scale: 10, Apk)
    SET_BRAKE_CURRENT           = 0x02, // 0x02: Target Brake AC Current (2 Bytes, int16, Scale: 10, Apk)
    SET_ERPM                    = 0x03, // 0x03: Target ERPM (4 Bytes, int32, Scale: 1, RPM)
    SET_RELATIVE_CURRENT        = 0x05, // 0x05: Target Relative Current (2 Bytes, int16, Scale: 10, %)
    SET_DIGITAL_OUTPUT          = 0x07, // 0x07: Digital Output Command (1 Byte, uint8, Bitmask DO1~DO4)
    SET_MAX_AC_CURRENT          = 0x08, // 0x08: Max AC Current Limit (2 Bytes, int16, Scale: 10, Apk)
    SET_MAX_DC_CURRENT          = 0x0A, // 0x0A: Max DC Current Limit (2 Bytes, int16, Scale: 10, Adc)
    SET_MAX_DC_BRAKE_CURRENT    = 0x0B, // 0x0B: Max DC Brake Limit (2 Bytes, int16, Scale: 10, Adc)
    DRIVE_ENABLE                = 0x0C  // 0x0C: Drive Enable (1 Byte, uint8, 1: Enable, 0: Disable)
};

// ============================================================================
// Inverter Telemetry Packet IDs (Sheet 2: Inverter_Tx_Telemetry)
// ============================================================================
enum class InverterPacketId : uint8_t {
    GENERAL_DATA_6              = 0x1F, // 0x1F: Mode, Target Iq, Position, isMotorStill (8 Bytes, 20 ms)
    GENERAL_DATA_1              = 0x20, // 0x20: Real ERPM, Duty Cycle, Input Voltage (8 Bytes, 10 ms)
    GENERAL_DATA_2              = 0x21, // 0x21: AC Current, DC Current (8 Bytes, 10 ms)
    GENERAL_DATA_3              = 0x22, // 0x22: Controller Temp, Motor Temp, Fault Code (8 Bytes, 100 ms)
    GENERAL_DATA_4              = 0x23, // 0x23: Id Current, Iq Current (8 Bytes, 20 ms)
    GENERAL_DATA_5              = 0x24, // 0x24: Throttle, Brake, IO, Drive Enable, Limits, Map Ver (8 Bytes, 100 ms)
    CONFIG_AVAIL_AC             = 0x25, // 0x25: Configured & Available AC Currents (8 Bytes, 100 ms)
    CONFIG_AVAIL_DC             = 0x26, // 0x26: Configured & Available DC Currents (8 Bytes, 100 ms)
    UNKNOWN                     = 0xFF
};

// ============================================================================
// Enumerations
// ============================================================================

// Control Mode (General Data 6, Byte 0)
enum class ControlMode : uint8_t {
    SPEED           = 1,
    CURRENT         = 2,
    CURRENT_BRAKE   = 3,
    POS             = 4,
    NONE            = 7,
    UNKNOWN         = 255
};

// Fault Code (General Data 3, Byte 4)
enum class FaultCode : uint8_t {
    NONE                = 0x00,
    OVER_VOLTAGE        = 0x01,
    UNDER_VOLTAGE       = 0x02,
    DRV                 = 0x03,
    ABS_OVER_CURRENT    = 0x04,
    OVER_TEMP_FET       = 0x05,
    OVER_TEMP_MOTOR     = 0x06,
    SENSOR_WIRE_FAULT   = 0x07, // 0x07: Sensor wire fault
    SENSOR_FAULT        = 0x08, // 0x08: Sensor fault
    CAN_CMD_ERR         = 0x09, // 0x09: CAN cmd err
    ANALOG_INPUT_ERR    = 0x0A  // 0x0A: Analog input err
};

// Motor Motion State (General Data 6, Byte 5)
enum class MotorStillState : uint8_t {
    ROTATING    = 0,
    STILL       = 1
};

// Drive Enable Status (General Data 5, Byte 3 / Command 0x0C)
enum class DriveEnableState : uint8_t {
    DISABLED    = 0,
    ENABLED     = 1
};

// ============================================================================
// Generic CAN Frame Structure
// ============================================================================
struct CANFrame {
    uint32_t id{0};             // Standard (11-bit) or Extended (29-bit) CAN ID
    uint8_t  dlc{0};            // Data Length Code (0~8)
    uint8_t  data[8]{0};        // Data Payload
    uint64_t timestamp_us{0};   // Optional timestamp in microseconds
};

// ============================================================================
// Telemetry Packet Data Structures (Decoded Physical Values)
// ============================================================================

// Packet 0x1F: General Data 6 (20 ms)
struct GeneralData6 {
    ControlMode     control_mode{ControlMode::UNKNOWN};
    float           target_iq_apk{0.0f};        // Target Iq (Scale: 10, Apk)
    float           motor_position_deg{0.0f};   // Motor Position (Scale: 10, 0~359 deg)
    MotorStillState is_motor_still{MotorStillState::STILL};
    uint16_t        reserved{0xFFFF};
    uint64_t        timestamp_us{0};
};

// Packet 0x20: General Data 1 (10 ms)
struct GeneralData1 {
    int32_t         real_erpm{0};               // Actual motor electrical RPM (Scale: 1, RPM)
    float           duty_cycle_pct{0.0f};       // PWM Duty Cycle (Scale: 10, %, -100.0% ~ 100.0%)
    float           input_voltage_v{0.0f};      // DC Bus Voltage (Scale: 1, V)
    uint64_t        timestamp_us{0};
};

// Packet 0x21: General Data 2 (10 ms)
struct GeneralData2 {
    float           ac_current_apk{0.0f};       // Actual AC current RMS/Peak (Scale: 10, Apk)
    float           dc_current_adc{0.0f};       // Actual DC bus current (Scale: 10, Adc)
    uint32_t        reserved{0xFFFFFFFF};
    uint64_t        timestamp_us{0};
};

// Packet 0x22: General Data 3 (100 ms)
struct GeneralData3 {
    float           controller_temp_c{0.0f};    // Controller / IGBT Temp (Scale: 10, °C)
    float           motor_temp_c{0.0f};         // Motor internal Temp (Scale: 10, °C)
    FaultCode       fault_code{FaultCode::NONE};
    uint32_t        reserved{0xFFFFFF};
    uint64_t        timestamp_us{0};
};

// Packet 0x23: General Data 4 (20 ms)
struct GeneralData4 {
    float           id_current_apk{0.0f};       // FOC Id direct-axis current (Scale: 100, Apk)
    float           iq_current_apk{0.0f};       // FOC Iq quadrature-axis current (Scale: 100, Apk)
    uint64_t        timestamp_us{0};
};

// Packet 0x24: General Data 5 (100 ms)
struct GeneralData5 {
    int8_t          throttle_pct{0};            // Throttle signal (Scale: 1, %)
    int8_t          brake_pct{0};               // Brake signal (Scale: 1, %)
    uint8_t         digital_ios{0};             // Digital IO Bitmask (DO1~DO4, DI1~DI4)
    DriveEnableState drive_enable_status{DriveEnableState::DISABLED}; // 1: Driven
    uint16_t        limit_status_bits{0};       // Current/Temp/Voltage Limit & Derating flags
    uint8_t         can_map_version{0};         // CAN Map Version (e.g. 25 = V2.5)
    uint64_t        timestamp_us{0};
};

// Packet 0x25: Configured & Available AC (100 ms)
struct ConfigAvailAC {
    float           max_ac_configured_apk{0.0f};// DTI Tool configured max AC (Scale: 10, Apk)
    float           max_ac_available_apk{0.0f}; // Current available max AC (Scale: 10, Apk)
    float           min_ac_configured_apk{0.0f};// DTI Tool configured min AC (Scale: 10, Apk)
    float           min_ac_available_apk{0.0f}; // Current available min AC (Scale: 10, Apk)
    uint64_t        timestamp_us{0};
};

// Packet 0x26: Configured & Available DC (100 ms)
struct ConfigAvailDC {
    float           max_dc_configured_adc{0.0f};// DTI Tool configured max DC (Scale: 10, Adc)
    float           max_dc_available_adc{0.0f}; // Current available max DC (Scale: 10, Adc)
    float           min_dc_configured_adc{0.0f};// DTI Tool configured min DC (Scale: 10, Adc)
    float           min_dc_available_adc{0.0f}; // Current available min DC (Scale: 10, Adc)
    uint64_t        timestamp_us{0};
};

// ============================================================================
// Comprehensive Inverter Full State Snapshot
// ============================================================================
struct InverterState {
    uint8_t         node_id{0};                 // Inverter CAN Node ID
    bool            is_connected{false};        // Whether communication is active
    uint64_t        last_seen_timestamp_us{0};  // Last received frame timestamp

    // General Data 1 (0x20)
    int32_t         erpm{0};
    float           duty_cycle_pct{0.0f};
    float           input_voltage_v{0.0f};

    // General Data 2 (0x21)
    float           ac_current_apk{0.0f};
    float           dc_current_adc{0.0f};

    // General Data 3 (0x22)
    float           controller_temp_c{0.0f};
    float           motor_temp_c{0.0f};
    FaultCode       fault_code{FaultCode::NONE};

    // General Data 4 (0x23)
    float           id_current_apk{0.0f};
    float           iq_current_apk{0.0f};

    // General Data 5 (0x24)
    int8_t          throttle_pct{0};
    int8_t          brake_pct{0};
    uint8_t         digital_ios{0};
    DriveEnableState drive_enable_status{DriveEnableState::DISABLED};
    uint16_t        limit_status_bits{0};
    uint8_t         can_map_version{0};

    // General Data 6 (0x1F)
    ControlMode     control_mode{ControlMode::UNKNOWN};
    float           target_iq_apk{0.0f};
    float           motor_position_deg{0.0f};
    MotorStillState is_motor_still{MotorStillState::STILL};

    // Configured & Available AC (0x25)
    float           max_ac_configured_apk{0.0f};
    float           max_ac_available_apk{0.0f};
    float           min_ac_configured_apk{0.0f};
    float           min_ac_available_apk{0.0f};

    // Configured & Available DC (0x26)
    float           max_dc_configured_adc{0.0f};
    float           max_dc_available_adc{0.0f};
    float           min_dc_configured_adc{0.0f};
    float           min_dc_available_adc{0.0f};

    // Timestamp tracking for each individual packet (us)
    uint64_t        pkt_time_gd1_us{0};
    uint64_t        pkt_time_gd2_us{0};
    uint64_t        pkt_time_gd3_us{0};
    uint64_t        pkt_time_gd4_us{0};
    uint64_t        pkt_time_gd5_us{0};
    uint64_t        pkt_time_gd6_us{0};
    uint64_t        pkt_time_ac_us{0};
    uint64_t        pkt_time_dc_us{0};
};

// ============================================================================
// VCU Command Data Structures
// ============================================================================
struct VcuCommandData {
    VcuPacketId packet_id;
    uint8_t     node_id{DEFAULT_NODE_ID};
    union {
        float    target_ac_current_apk;    // 0x01 (Scale: 10)
        float    target_brake_current_apk; // 0x02 (Scale: 10)
        int32_t  target_erpm;              // 0x03 (Scale: 1)
        float    target_relative_current;  // 0x05 (Scale: 10, %)
        uint8_t  digital_output_mask;      // 0x07 (Bitmask DO1~DO4)
        float    max_ac_current_limit_apk; // 0x08 (Scale: 10)
        float    max_dc_current_limit_adc; // 0x0A (Scale: 10)
        float    max_dc_brake_limit_adc;   // 0x0B (Scale: 10)
        uint8_t  drive_enable_cmd;         // 0x0C (1: Enable, 0: Disable)
    } value;
};

// ============================================================================
// Helper String Conversion Functions
// ============================================================================
inline const char* toString(ControlMode mode) {
    switch (mode) {
        case ControlMode::SPEED:         return "SPEED";
        case ControlMode::CURRENT:       return "CURRENT";
        case ControlMode::CURRENT_BRAKE: return "CURRENT_BRAKE";
        case ControlMode::POS:           return "POS";
        case ControlMode::NONE:          return "NONE";
        default:                         return "UNKNOWN";
    }
}

inline const char* toString(FaultCode fault) {
    switch (fault) {
        case FaultCode::NONE:              return "No Fault";
        case FaultCode::OVER_VOLTAGE:      return "Over Voltage";
        case FaultCode::UNDER_VOLTAGE:     return "Under Voltage";
        case FaultCode::DRV:               return "DRV Fault";
        case FaultCode::ABS_OVER_CURRENT:  return "Abs Over Current";
        case FaultCode::OVER_TEMP_FET:     return "Over Temp FET";
        case FaultCode::OVER_TEMP_MOTOR:   return "Over Temp Motor";
        case FaultCode::SENSOR_WIRE_FAULT: return "Sensor Wire Fault (0x07)";
        case FaultCode::SENSOR_FAULT:      return "Sensor Fault (0x08)";
        case FaultCode::CAN_CMD_ERR:       return "CAN Command Error (0x09)";
        case FaultCode::ANALOG_INPUT_ERR:  return "Analog Input Error (0x0A)";
        default:                           return "Unknown Fault";
    }
}

inline const char* toString(InverterPacketId pkt) {
    switch (pkt) {
        case InverterPacketId::GENERAL_DATA_6:  return "General Data 6 (0x1F)";
        case InverterPacketId::GENERAL_DATA_1:  return "General Data 1 (0x20)";
        case InverterPacketId::GENERAL_DATA_2:  return "General Data 2 (0x21)";
        case InverterPacketId::GENERAL_DATA_3:  return "General Data 3 (0x22)";
        case InverterPacketId::GENERAL_DATA_4:  return "General Data 4 (0x23)";
        case InverterPacketId::GENERAL_DATA_5:  return "General Data 5 (0x24)";
        case InverterPacketId::CONFIG_AVAIL_AC: return "Configured & Avail AC (0x25)";
        case InverterPacketId::CONFIG_AVAIL_DC: return "Configured & Avail DC (0x26)";
        default:                                return "Unknown Packet";
    }
}

inline const char* toString(VcuPacketId pkt) {
    switch (pkt) {
        case VcuPacketId::SET_AC_CURRENT:           return "Set AC Current (0x01)";
        case VcuPacketId::SET_BRAKE_CURRENT:        return "Set Brake Current (0x02)";
        case VcuPacketId::SET_ERPM:                 return "Set ERPM (0x03)";
        case VcuPacketId::SET_RELATIVE_CURRENT:     return "Set Relative Current (0x05)";
        case VcuPacketId::SET_DIGITAL_OUTPUT:       return "Set Digital Output (0x07)";
        case VcuPacketId::SET_MAX_AC_CURRENT:       return "Set Max AC Current (0x08)";
        case VcuPacketId::SET_MAX_DC_CURRENT:       return "Set Max DC Current (0x0A)";
        case VcuPacketId::SET_MAX_DC_BRAKE_CURRENT: return "Set Max DC Brake Current (0x0B)";
        case VcuPacketId::DRIVE_ENABLE:             return "Drive Enable (0x0C)";
        default:                                    return "Unknown VCU Command";
    }
}

} // namespace DTI
