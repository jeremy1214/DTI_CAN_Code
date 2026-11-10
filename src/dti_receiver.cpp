#include "dti_can/dti_receiver.hpp"
#include <chrono>

namespace DTI {

InverterReceiver::InverterReceiver(uint8_t filterNodeId)
    : filterNodeId_(filterNodeId) {
}

uint64_t InverterReceiver::getCurrentTimeUs() {
    auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}

bool InverterReceiver::processRaw(uint32_t canId, const uint8_t* data, uint8_t dlc, uint64_t timestamp_us) {
    CANFrame frame{};
    frame.id = canId;
    frame.dlc = dlc;
    if (data && dlc > 0) {
        uint8_t copyLen = dlc > 8 ? 8 : dlc;
        for (uint8_t i = 0; i < copyLen; ++i) {
            frame.data[i] = data[i];
        }
    }
    frame.timestamp_us = (timestamp_us != 0) ? timestamp_us : getCurrentTimeUs();
    return processFrame(frame);
}

bool InverterReceiver::processFrame(const CANFrame& frame) {
    CANFrame inFrame = frame;
    if (inFrame.timestamp_us == 0) {
        inFrame.timestamp_us = getCurrentTimeUs();
    }

    InverterPacketId pktId = InverterPacketId::UNKNOWN;
    uint8_t nodeId = 0;
    TelemetryPacket decodedPacket;

    bool decoded = Codec::decodeTelemetryFrame(inFrame, pktId, nodeId, decodedPacket);

    if (decoded) {
        // Node ID filtering
        if (filterNodeId_ != 0 && nodeId != filterNodeId_) {
            if (onFrameProcessed_) {
                onFrameProcessed_(inFrame, false);
            }
            return false;
        }

        FaultCode newFault = FaultCode::NONE;
        bool faultChanged = false;
        InverterState stateSnapshot;

        {
            std::lock_guard<std::mutex> lock(mutex_);
            state_.node_id = nodeId;
            state_.is_connected = true;
            state_.last_seen_timestamp_us = inFrame.timestamp_us;

            switch (pktId) {
                case InverterPacketId::GENERAL_DATA_6: {
                    const auto& gd6 = std::get<GeneralData6>(decodedPacket);
                    state_.control_mode       = gd6.control_mode;
                    state_.target_iq_apk      = gd6.target_iq_apk;
                    state_.motor_position_deg = gd6.motor_position_deg;
                    state_.is_motor_still     = gd6.is_motor_still;
                    state_.pkt_time_gd6_us    = inFrame.timestamp_us;
                    break;
                }
                case InverterPacketId::GENERAL_DATA_1: {
                    const auto& gd1 = std::get<GeneralData1>(decodedPacket);
                    state_.erpm               = gd1.real_erpm;
                    state_.duty_cycle_pct     = gd1.duty_cycle_pct;
                    state_.input_voltage_v    = gd1.input_voltage_v;
                    state_.pkt_time_gd1_us    = inFrame.timestamp_us;
                    break;
                }
                case InverterPacketId::GENERAL_DATA_2: {
                    const auto& gd2 = std::get<GeneralData2>(decodedPacket);
                    state_.ac_current_apk     = gd2.ac_current_apk;
                    state_.dc_current_adc     = gd2.dc_current_adc;
                    state_.pkt_time_gd2_us    = inFrame.timestamp_us;
                    break;
                }
                case InverterPacketId::GENERAL_DATA_3: {
                    const auto& gd3 = std::get<GeneralData3>(decodedPacket);
                    state_.controller_temp_c  = gd3.controller_temp_c;
                    state_.motor_temp_c       = gd3.motor_temp_c;
                    state_.fault_code         = gd3.fault_code;
                    state_.pkt_time_gd3_us    = inFrame.timestamp_us;

                    if (state_.fault_code != lastFaultCode_) {
                        faultChanged = true;
                        newFault = state_.fault_code;
                        lastFaultCode_ = newFault;
                    }
                    break;
                }
                case InverterPacketId::GENERAL_DATA_4: {
                    const auto& gd4 = std::get<GeneralData4>(decodedPacket);
                    state_.id_current_apk     = gd4.id_current_apk;
                    state_.iq_current_apk     = gd4.iq_current_apk;
                    state_.pkt_time_gd4_us    = inFrame.timestamp_us;
                    break;
                }
                case InverterPacketId::GENERAL_DATA_5: {
                    const auto& gd5 = std::get<GeneralData5>(decodedPacket);
                    state_.throttle_pct        = gd5.throttle_pct;
                    state_.brake_pct           = gd5.brake_pct;
                    state_.digital_ios         = gd5.digital_ios;
                    state_.drive_enable_status = gd5.drive_enable_status;
                    state_.limit_status_bits   = gd5.limit_status_bits;
                    state_.can_map_version     = gd5.can_map_version;
                    state_.pkt_time_gd5_us     = inFrame.timestamp_us;
                    break;
                }
                case InverterPacketId::CONFIG_AVAIL_AC: {
                    const auto& ac = std::get<ConfigAvailAC>(decodedPacket);
                    state_.max_ac_configured_apk = ac.max_ac_configured_apk;
                    state_.max_ac_available_apk  = ac.max_ac_available_apk;
                    state_.min_ac_configured_apk = ac.min_ac_configured_apk;
                    state_.min_ac_available_apk  = ac.min_ac_available_apk;
                    state_.pkt_time_ac_us        = inFrame.timestamp_us;
                    break;
                }
                case InverterPacketId::CONFIG_AVAIL_DC: {
                    const auto& dc = std::get<ConfigAvailDC>(decodedPacket);
                    state_.max_dc_configured_adc = dc.max_dc_configured_adc;
                    state_.max_dc_available_adc  = dc.max_dc_available_adc;
                    state_.min_dc_configured_adc = dc.min_dc_configured_adc;
                    state_.min_dc_available_adc  = dc.min_dc_available_adc;
                    state_.pkt_time_dc_us        = inFrame.timestamp_us;
                    break;
                }
                default:
                    break;
            }

            stateSnapshot = state_;
        }

        // Callbacks outside lock
        if (faultChanged && onFault_) {
            onFault_(newFault, toString(newFault));
        }

        if (onTelemetryUpdate_) {
            onTelemetryUpdate_(pktId, stateSnapshot);
        }
    }

    if (onFrameProcessed_) {
        onFrameProcessed_(inFrame, decoded);
    }

    return decoded;
}

InverterState InverterReceiver::getState() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_;
}

void InverterReceiver::resetState() {
    std::lock_guard<std::mutex> lock(mutex_);
    state_ = InverterState{};
    lastFaultCode_ = FaultCode::NONE;
}

bool InverterReceiver::isConnected(uint64_t timeout_ms, uint64_t current_time_us) const {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!state_.is_connected || state_.last_seen_timestamp_us == 0) {
        return false;
    }

    uint64_t now_us = (current_time_us != 0) ? current_time_us : getCurrentTimeUs();
    uint64_t diff_us = (now_us > state_.last_seen_timestamp_us) ? (now_us - state_.last_seen_timestamp_us) : 0;
    return diff_us <= (timeout_ms * 1000);
}

bool InverterReceiver::hasFault() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.fault_code != FaultCode::NONE;
}

FaultCode InverterReceiver::getFaultCode() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return state_.fault_code;
}

} // namespace DTI
