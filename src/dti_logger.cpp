#include "dti_can/dti_logger.hpp"
#include "dti_can/dti_codec.hpp"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <chrono>

namespace DTI {

Logger::Logger() = default;

Logger::~Logger() {
    stop();
}

static std::string stripExtension(const std::string& path) {
    size_t lastDot = path.find_last_of('.');
    if (lastDot == std::string::npos) return path;
    return path.substr(0, lastDot);
}

bool Logger::start(const std::string& baseFilePath, LogMode mode, bool autoFlush) {
    std::lock_guard<std::mutex> lock(mutex_);
    stop();

    mode_ = mode;
    autoFlush_ = autoFlush;

    auto now = std::chrono::steady_clock::now().time_since_epoch();
    startEpochUs_ = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());

    std::string base = stripExtension(baseFilePath);

    if (mode == LogMode::TELEMETRY_CSV || mode == LogMode::BOTH) {
        std::string telemPath = (mode == LogMode::BOTH) ? (base + "_telemetry.csv") : baseFilePath;
        telemetryFile_.open(telemPath, std::ios::out | std::ios::trunc);
        if (!telemetryFile_.is_open()) {
            return false;
        }
        writeTelemetryHeader();
    }

    if (mode == LogMode::RAW_CAN_CSV || mode == LogMode::BOTH) {
        std::string rawPath = (mode == LogMode::BOTH) ? (base + "_raw.csv") : baseFilePath;
        rawCanFile_.open(rawPath, std::ios::out | std::ios::trunc);
        if (!rawCanFile_.is_open()) {
            telemetryFile_.close();
            return false;
        }
        writeRawCanHeader();
    }

    isRunning_ = true;
    return true;
}

void Logger::stop() {
    if (telemetryFile_.is_open()) {
        telemetryFile_.flush();
        telemetryFile_.close();
    }
    if (rawCanFile_.is_open()) {
        rawCanFile_.flush();
        rawCanFile_.close();
    }
    isRunning_ = false;
}

void Logger::flush() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (telemetryFile_.is_open()) telemetryFile_.flush();
    if (rawCanFile_.is_open()) rawCanFile_.flush();
}

void Logger::writeTelemetryHeader() {
    telemetryFile_
        << "Timestamp_ms,"
        << "Elapsed_s,"
        << "Node_ID,"
        << "ERPM,"
        << "Input_Voltage_V,"
        << "Duty_Cycle_Pct,"
        << "AC_Current_Apk,"
        << "DC_Current_Adc,"
        << "Controller_Temp_C,"
        << "Motor_Temp_C,"
        << "Id_Current_Apk,"
        << "Iq_Current_Apk,"
        << "Target_Iq_Apk,"
        << "Motor_Pos_deg,"
        << "Motor_State,"
        << "Throttle_Pct,"
        << "Brake_Pct,"
        << "Digital_IOs,"
        << "Drive_Enable,"
        << "Control_Mode,"
        << "Fault_Code,"
        << "Fault_Desc,"
        << "Limit_Status_Bits,"
        << "CAN_Map_Version,"
        << "Max_AC_Avail_Apk,"
        << "Max_DC_Avail_Adc\n";
    if (autoFlush_) telemetryFile_.flush();
}

void Logger::writeRawCanHeader() {
    rawCanFile_
        << "Timestamp_ms,"
        << "Elapsed_s,"
        << "CAN_ID_Hex,"
        << "Packet_ID_Hex,"
        << "Packet_Name,"
        << "DLC,"
        << "B0,B1,B2,B3,B4,B5,B6,B7\n";
    if (autoFlush_) rawCanFile_.flush();
}

void Logger::logState(const InverterState& state) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isRunning_ || !telemetryFile_.is_open()) return;

    uint64_t ts = state.last_seen_timestamp_us;
    double elapsed_s = (ts >= startEpochUs_) ? (static_cast<double>(ts - startEpochUs_) / 1000000.0) : 0.0;
    uint64_t ts_ms = ts / 1000;

    std::ostringstream ss;
    ss << std::fixed << std::setprecision(2);
    ss << ts_ms << ","
       << std::setprecision(4) << elapsed_s << ","
       << "0x" << std::hex << std::uppercase << static_cast<int>(state.node_id) << std::dec << ","
       << state.erpm << ","
       << std::setprecision(2)
       << state.input_voltage_v << ","
       << state.duty_cycle_pct << ","
       << state.ac_current_apk << ","
       << state.dc_current_adc << ","
       << state.controller_temp_c << ","
       << state.motor_temp_c << ","
       << state.id_current_apk << ","
       << state.iq_current_apk << ","
       << state.target_iq_apk << ","
       << state.motor_position_deg << ","
       << (state.is_motor_still == MotorStillState::STILL ? "STILL" : "ROTATING") << ","
       << static_cast<int>(state.throttle_pct) << ","
       << static_cast<int>(state.brake_pct) << ","
       << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(state.digital_ios) << std::dec << ","
       << (state.drive_enable_status == DriveEnableState::ENABLED ? "ENABLED" : "DISABLED") << ","
       << toString(state.control_mode) << ","
       << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(state.fault_code) << std::dec << ","
       << "\"" << toString(state.fault_code) << "\","
       << "0x" << std::hex << std::setw(4) << std::setfill('0') << state.limit_status_bits << std::dec << ","
       << static_cast<int>(state.can_map_version) << ","
       << state.max_ac_available_apk << ","
       << state.max_dc_available_adc << "\n";

    telemetryFile_ << ss.str();
    if (autoFlush_) telemetryFile_.flush();
}

void Logger::logRawFrame(const CANFrame& frame) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!isRunning_ || !rawCanFile_.is_open()) return;

    uint64_t ts = frame.timestamp_us;
    if (ts == 0) {
        auto now = std::chrono::steady_clock::now().time_since_epoch();
        ts = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
    }
    double elapsed_s = (ts >= startEpochUs_) ? (static_cast<double>(ts - startEpochUs_) / 1000000.0) : 0.0;
    uint64_t ts_ms = ts / 1000;

    uint8_t pktId = 0;
    uint8_t nodeId = 0;
    Codec::extractPacketAndNodeId(frame.id, pktId, nodeId);
    const char* pktName = toString(static_cast<InverterPacketId>(pktId));

    std::ostringstream ss;
    ss << ts_ms << ","
       << std::fixed << std::setprecision(4) << elapsed_s << ","
       << "0x" << std::hex << std::uppercase << frame.id << ","
       << "0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(pktId) << ","
       << "\"" << pktName << "\","
       << std::dec << static_cast<int>(frame.dlc);

    for (int i = 0; i < 8; ++i) {
        if (i < frame.dlc) {
            ss << ",0x" << std::hex << std::setw(2) << std::setfill('0') << std::uppercase << static_cast<int>(frame.data[i]);
        } else {
            ss << ",-";
        }
    }
    ss << "\n";

    rawCanFile_ << ss.str();
    if (autoFlush_) rawCanFile_.flush();
}

std::string Logger::formatDashboard(const InverterState& state) {
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(1);

    ss << "======================================================================\n";
    ss << "                     DTI INVERTER TELEMETRY MONITOR                   \n";
    ss << "======================================================================\n";
    ss << " Node ID : 0x" << std::hex << std::uppercase << static_cast<int>(state.node_id) << std::dec
       << " | Comm Status: " << (state.is_connected ? "ONLINE" : "OFFLINE")
       << " | Mode: " << toString(state.control_mode)
       << " | Version: " << (state.can_map_version / 10) << "." << (state.can_map_version % 10) << "\n";
    ss << "----------------------------------------------------------------------\n";

    // Row 1
    ss << " Electrical RPM   : " << std::setw(8) << state.erpm << " RPM"
       << "  | DC Bus Voltage  : " << std::setw(8) << state.input_voltage_v << " V\n";

    // Row 2
    ss << " PWM Duty Cycle   : " << std::setw(8) << state.duty_cycle_pct << " %  "
       << "  | AC Phase Current: " << std::setw(8) << state.ac_current_apk << " Apk\n";

    // Row 3
    ss << " DC Bus Current   : " << std::setw(8) << state.dc_current_adc << " Adc"
       << "  | Motor Position  : " << std::setw(8) << state.motor_position_deg << " deg\n";

    // Row 4
    ss << " Inverter Temp    : " << std::setw(8) << state.controller_temp_c << " °C "
       << "  | Motor Temp      : " << std::setw(8) << state.motor_temp_c << " °C\n";

    // Row 5
    ss << std::setprecision(2);
    ss << " FOC Id Current   : " << std::setw(8) << state.id_current_apk << " Apk"
       << "  | FOC Iq Current  : " << std::setw(8) << state.iq_current_apk << " Apk\n";

    // Row 6
    ss << " Target Iq Current: " << std::setw(8) << state.target_iq_apk << " Apk"
       << "  | Motor Motion    : " << (state.is_motor_still == MotorStillState::STILL ? "STILL" : "ROTATING") << "\n";

    // Row 7
    ss << " Throttle / Brake : " << std::setw(3) << static_cast<int>(state.throttle_pct) << "% / "
       << std::setw(3) << static_cast<int>(state.brake_pct) << "%"
       << "  | Drive Enabled   : " << (state.drive_enable_status == DriveEnableState::ENABLED ? "YES (DRIVEN)" : "NO (DISABLED)") << "\n";

    // Row 8
    ss << " Fault Code       : 0x" << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(state.fault_code)
       << " (" << toString(state.fault_code) << ")" << std::dec << std::setfill(' ') << "\n";

    ss << " Limits / Derating: 0x" << std::hex << std::setw(4) << std::setfill('0') << state.limit_status_bits
       << " | Digital IOs: 0x" << std::setw(2) << static_cast<int>(state.digital_ios) << std::dec << std::setfill(' ') << "\n";

    ss << "----------------------------------------------------------------------\n";
    ss << std::setprecision(1);
    ss << " Max AC Current: Config = " << state.max_ac_configured_apk << " Apk, Avail = " << state.max_ac_available_apk << " Apk\n";
    ss << " Max DC Current: Config = " << state.max_dc_configured_adc << " Adc, Avail = " << state.max_dc_available_adc << " Adc\n";
    ss << "======================================================================\n";

    return ss.str();
}

void Logger::printDashboard(const InverterState& state, bool clearScreen) {
    if (clearScreen) {
        std::cout << "\033[2J\033[H";
    }
    std::cout << formatDashboard(state) << std::flush;
}

} // namespace DTI
