#pragma once

#include "dti_codec.hpp"
#include <mutex>
#include <functional>
#include <chrono>

namespace DTI {

/**
 * @brief Thread-safe Inverter Telemetry Receiver & State Aggregator
 * 
 * Aggregates all periodic telemetry broadcast packets from the DTI inverter into
 * a consolidated InverterState. Supports observer callbacks and connection health monitoring.
 */
class InverterReceiver {
public:
    using TelemetryUpdateCallback = std::function<void(InverterPacketId pktId, const InverterState& state)>;
    using FaultCallback           = std::function<void(FaultCode fault, const char* desc)>;
    using FrameProcessedCallback  = std::function<void(const CANFrame& frame, bool decoded)>;

    explicit InverterReceiver(uint8_t filterNodeId = 0); // 0 = accept any node ID
    ~InverterReceiver() = default;

    // Non-copyable, movable
    InverterReceiver(const InverterReceiver&) = delete;
    InverterReceiver& operator=(const InverterReceiver&) = delete;
    InverterReceiver(InverterReceiver&&) = default;
    InverterReceiver& operator=(InverterReceiver&&) = default;

    // ========================================================================
    // Frame Ingestion
    // ========================================================================

    /**
     * @brief Process an incoming CAN frame.
     * @param frame Raw CAN frame
     * @return true if frame was recognized and decoded, false otherwise
     */
    bool processFrame(const CANFrame& frame);

    /**
     * @brief Convenient helper to process raw CAN message fields.
     */
    bool processRaw(uint32_t canId, const uint8_t* data, uint8_t dlc, uint64_t timestamp_us = 0);

    // ========================================================================
    // State Access (Thread-Safe)
    // ========================================================================

    /**
     * @brief Retrieve a snapshot of the current inverter state.
     */
    InverterState getState() const;

    /**
     * @brief Reset all internal state fields to default.
     */
    void resetState();

    /**
     * @brief Check whether inverter telemetry is active within given timeout.
     * @param timeout_ms Timeout threshold in milliseconds (default 500 ms)
     * @param current_time_us Current system time in us (0 to use steady_clock)
     */
    bool isConnected(uint64_t timeout_ms = 500, uint64_t current_time_us = 0) const;

    /**
     * @brief Returns true if inverter is reporting any fault.
     */
    bool hasFault() const;

    /**
     * @brief Get active fault code.
     */
    FaultCode getFaultCode() const;

    /**
     * @brief Get filter node ID (0 means accepting all).
     */
    uint8_t getFilterNodeId() const { return filterNodeId_; }
    void setFilterNodeId(uint8_t nodeId) { filterNodeId_ = nodeId; }

    // ========================================================================
    // Callbacks Registration
    // ========================================================================
    void onTelemetryUpdate(TelemetryUpdateCallback cb) { onTelemetryUpdate_ = std::move(cb); }
    void onFault(FaultCallback cb)                     { onFault_ = std::move(cb); }
    void onFrameProcessed(FrameProcessedCallback cb)   { onFrameProcessed_ = std::move(cb); }

private:
    uint8_t                 filterNodeId_{0};
    mutable std::mutex      mutex_;
    InverterState           state_{};
    FaultCode               lastFaultCode_{FaultCode::NONE};

    TelemetryUpdateCallback onTelemetryUpdate_{nullptr};
    FaultCallback           onFault_{nullptr};
    FrameProcessedCallback  onFrameProcessed_{nullptr};

    static uint64_t getCurrentTimeUs();
};

} // namespace DTI
