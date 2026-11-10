#pragma once

#include "dti_types.hpp"
#include <string>
#include <fstream>
#include <mutex>
#include <memory>

namespace DTI {

/**
 * @brief Telemetry and CAN Frame Logger
 * 
 * Supports:
 * 1. Telemetry State CSV logging (engineering units: RPM, V, A, deg, °C, etc.)
 * 2. Raw CAN Frame CSV logging (ID, DLC, raw bytes, timestamp, packet description)
 * 3. Real-time Console Monitor formatting
 */
class Logger {
public:
    enum class LogMode {
        TELEMETRY_CSV,  // Logs aggregated physical state variables
        RAW_CAN_CSV,    // Logs raw CAN frames with hex bytes and packet names
        BOTH            // Logs both formats into separate files
    };

    Logger();
    ~Logger();

    // Non-copyable, movable
    Logger(const Logger&) = delete;
    Logger& operator=(const Logger&) = delete;
    Logger(Logger&&) = default;
    Logger& operator=(Logger&&) = default;

    // ========================================================================
    // File Management
    // ========================================================================

    /**
     * @brief Open log file(s).
     * @param baseFilePath Base file name or directory (e.g. "dti_log_20261110.csv")
     * @param mode LogMode::TELEMETRY_CSV, RAW_CAN_CSV, or BOTH
     * @param autoFlush If true, flushes to disk after each write (avoids data loss on crash)
     * @return true if successfully opened
     */
    bool start(const std::string& baseFilePath, LogMode mode = LogMode::TELEMETRY_CSV, bool autoFlush = true);

    /**
     * @brief Close all open log files.
     */
    void stop();

    /**
     * @brief Check if logger is actively recording.
     */
    bool isRunning() const { return isRunning_; }

    // ========================================================================
    // Logging Functions
    // ========================================================================

    /**
     * @brief Log an aggregated InverterState snapshot to Telemetry CSV.
     */
    void logState(const InverterState& state);

    /**
     * @brief Log a raw CAN frame to Raw CAN CSV.
     */
    void logRawFrame(const CANFrame& frame);

    /**
     * @brief Flush internal file buffers to disk immediately.
     */
    void flush();

    // ========================================================================
    // Real-time Console Dashboard Display
    // ========================================================================

    /**
     * @brief Format InverterState into a multi-line formatted dashboard string.
     */
    static std::string formatDashboard(const InverterState& state);

    /**
     * @brief Print the formatted dashboard to stdout (console).
     * @param clearScreen If true, clears terminal before printing
     */
    static void printDashboard(const InverterState& state, bool clearScreen = false);

private:
    std::mutex    mutex_;
    bool          isRunning_{false};
    bool          autoFlush_{true};
    LogMode       mode_{LogMode::TELEMETRY_CSV};
    std::ofstream telemetryFile_;
    std::ofstream rawCanFile_;
    uint64_t      startEpochUs_{0};

    void writeTelemetryHeader();
    void writeRawCanHeader();
};

} // namespace DTI
