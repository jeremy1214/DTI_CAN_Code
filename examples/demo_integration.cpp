#include "dti_can/dti_can.hpp"
#include <iostream>
#include <thread>
#include <chrono>
#include <vector>

/**
 * @brief Demonstration showing how an external program (VCU firmware, GUI, test bench, ROS node)
 *        integrates the DTI CAN Codec and Logger.
 */
int main() {
    std::cout << "================================================================" << std::endl;
    std::cout << "        DTI Inverter CAN Codec & Logger Integration Demo        " << std::endl;
    std::cout << "================================================================\n" << std::endl;

    const uint8_t INVERTER_NODE_ID = 0x0F;

    // 1. Initialize Inverter State Receiver
    DTI::InverterReceiver receiver(INVERTER_NODE_ID);

    // Register optional event callbacks
    receiver.onFault([](DTI::FaultCode fault, const char* desc) {
        std::cout << "[ALERT] Fault Detected on Inverter: " << desc << " (0x" 
                  << std::hex << static_cast<int>(fault) << std::dec << ")" << std::endl;
    });

    receiver.onTelemetryUpdate([](DTI::InverterPacketId pktId, const DTI::InverterState& /*state*/) {
        // External application can respond immediately to specific packet updates
        if (pktId == DTI::InverterPacketId::GENERAL_DATA_1) {
            // e.g. Speed/Voltage loop check
        }
    });

    // 2. Initialize Data Logger
    DTI::Logger logger;
    std::string logBasePath = "dti_inverter_run_log.csv";
    if (logger.start(logBasePath, DTI::Logger::LogMode::BOTH, true)) {
        std::cout << "[LOGGER] Started logging to: " << std::endl;
        std::cout << "         -> Telemetry CSV : dti_inverter_run_log_telemetry.csv" << std::endl;
        std::cout << "         -> Raw CAN CSV   : dti_inverter_run_log_raw.csv\n" << std::endl;
    } else {
        std::cerr << "[ERROR] Failed to open log file." << std::endl;
        return 1;
    }

    std::cout << "[SIM] Simulating 10 cycles of incoming CAN telemetry from DTI Inverter...\n" << std::endl;

    // 3. Simulate incoming CAN traffic from hardware interface (e.g. PCAN, SocketCAN, STM32)
    for (int step = 1; step <= 10; ++step) {
        // Simulate varying realistic data values
        int32_t sim_erpm       = 2000 + step * 400;             // ERPM rising
        float   sim_volt       = 380.0f - step * 0.5f;          // DC Bus Voltage (V)
        float   sim_duty       = 20.0f + step * 5.0f;           // Duty cycle (%)
        float   sim_ac_curr    = 30.0f + step * 8.0f;           // Phase AC current (Apk)
        float   sim_dc_curr    = 15.0f + step * 4.0f;           // DC Bus current (Adc)
        float   sim_ctrl_temp  = 38.0f + step * 0.4f;           // Inverter IGBT temp (°C)
        float   sim_motor_temp = 45.0f + step * 0.8f;           // Motor temp (°C)
        float   sim_target_iq  = sim_ac_curr + 2.0f;            // Target Iq (Apk)
        float   sim_motor_pos  = static_cast<float>((step * 36) % 360); // Angle (0~359 deg)

        // --------------------------------------------------------------------
        // Frame 1: General Data 1 (0x20) - ERPM, Duty Cycle, Input Voltage
        // --------------------------------------------------------------------
        {
            DTI::CANFrame f{};
            f.id = DTI::Codec::makeExtendedCanId(0x20, INVERTER_NODE_ID);
            f.dlc = 8;
            DTI::Codec::writeInt32BE(&f.data[0], sim_erpm);
            DTI::Codec::writeInt16BE(&f.data[4], static_cast<int16_t>(sim_duty * 10.0f));
            DTI::Codec::writeInt16BE(&f.data[6], static_cast<int16_t>(sim_volt));

            receiver.processFrame(f);
            logger.logRawFrame(f);
        }

        // --------------------------------------------------------------------
        // Frame 2: General Data 2 (0x21) - AC Current, DC Current
        // --------------------------------------------------------------------
        {
            DTI::CANFrame f{};
            f.id = DTI::Codec::makeExtendedCanId(0x21, INVERTER_NODE_ID);
            f.dlc = 8;
            DTI::Codec::writeInt16BE(&f.data[0], static_cast<int16_t>(sim_ac_curr * 10.0f));
            DTI::Codec::writeInt16BE(&f.data[2], static_cast<int16_t>(sim_dc_curr * 10.0f));

            receiver.processFrame(f);
            logger.logRawFrame(f);
        }

        // --------------------------------------------------------------------
        // Frame 3: General Data 6 (0x1F) - Mode, Target Iq, Position, Motion
        // --------------------------------------------------------------------
        {
            DTI::CANFrame f{};
            f.id = DTI::Codec::makeExtendedCanId(0x1F, INVERTER_NODE_ID);
            f.dlc = 8;
            f.data[0] = static_cast<uint8_t>(DTI::ControlMode::SPEED);
            DTI::Codec::writeInt16BE(&f.data[1], static_cast<int16_t>(sim_target_iq * 10.0f));
            DTI::Codec::writeInt16BE(&f.data[3], static_cast<uint16_t>(sim_motor_pos * 10.0f));
            f.data[5] = 0; // Rotating

            receiver.processFrame(f);
            logger.logRawFrame(f);
        }

        // --------------------------------------------------------------------
        // Frame 4: General Data 3 (0x22) - Temperatures, Fault Code
        // --------------------------------------------------------------------
        {
            DTI::CANFrame f{};
            f.id = DTI::Codec::makeExtendedCanId(0x22, INVERTER_NODE_ID);
            f.dlc = 8;
            DTI::Codec::writeInt16BE(&f.data[0], static_cast<int16_t>(sim_ctrl_temp * 10.0f));
            DTI::Codec::writeInt16BE(&f.data[2], static_cast<int16_t>(sim_motor_temp * 10.0f));
            f.data[4] = 0x00; // No fault

            receiver.processFrame(f);
            logger.logRawFrame(f);
        }

        // --------------------------------------------------------------------
        // Frame 5: General Data 4 (0x23) - FOC Id, Iq
        // --------------------------------------------------------------------
        {
            DTI::CANFrame f{};
            f.id = DTI::Codec::makeExtendedCanId(0x23, INVERTER_NODE_ID);
            f.dlc = 8;
            DTI::Codec::writeInt32BE(&f.data[0], static_cast<int32_t>(1.25f * 100.0f));
            DTI::Codec::writeInt32BE(&f.data[4], static_cast<int32_t>(sim_ac_curr * 0.98f * 100.0f));

            receiver.processFrame(f);
            logger.logRawFrame(f);
        }

        // --------------------------------------------------------------------
        // Frame 6: General Data 5 (0x24) - Throttle, Brake, IO, Driven, Map Ver
        // --------------------------------------------------------------------
        {
            DTI::CANFrame f{};
            f.id = DTI::Codec::makeExtendedCanId(0x24, INVERTER_NODE_ID);
            f.dlc = 8;
            f.data[0] = static_cast<int8_t>(20 + step * 5); // Throttle %
            f.data[1] = 0;                                  // Brake %
            f.data[2] = 0x01;                               // DO1 active
            f.data[3] = 1;                                  // Drive Enable: Driven
            f.data[7] = 25;                                 // Map Version 2.5

            receiver.processFrame(f);
            logger.logRawFrame(f);
        }

        // Log the consolidated InverterState snapshot to Telemetry CSV
        auto stateSnapshot = receiver.getState();
        logger.logState(stateSnapshot);

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    // 4. Print Dashboard of final state
    std::cout << "\n[MONITOR] Final Inverter Real-Time State:" << std::endl;
    DTI::Logger::printDashboard(receiver.getState());

    // 5. Demonstrate VCU Command Encoding (External program sending command to inverter)
    std::cout << "\n[VCU TX] Demonstrating VCU Command Encoding:" << std::endl;

    // Command A: Enable Drive
    auto cmdDrive = DTI::Codec::encodeDriveEnable(true, INVERTER_NODE_ID);
    std::cout << " -> Drive Enable Cmd (0x0C): CAN ID = 0x" << std::hex << cmdDrive.id 
              << ", DLC = " << std::dec << static_cast<int>(cmdDrive.dlc)
              << ", Data[0] = 0x" << std::hex << static_cast<int>(cmdDrive.data[0]) << std::dec << std::endl;

    // Command B: Set Target AC Current to 45.0 Apk
    auto cmdAcCur = DTI::Codec::encodeSetAcCurrent(45.0f, INVERTER_NODE_ID);
    std::cout << " -> Set AC Current (0x01) 45.0 Apk: CAN ID = 0x" << std::hex << cmdAcCur.id 
              << ", DLC = " << std::dec << static_cast<int>(cmdAcCur.dlc)
              << ", Raw Hex = 0x" << std::hex << static_cast<int>(cmdAcCur.data[0])
              << " 0x" << static_cast<int>(cmdAcCur.data[1]) << std::dec << std::endl;

    // Command C: Set Target ERPM to 8000 RPM
    auto cmdErpm = DTI::Codec::encodeSetErpm(8000, INVERTER_NODE_ID);
    std::cout << " -> Set ERPM (0x03) 8000 RPM: CAN ID = 0x" << std::hex << cmdErpm.id 
              << ", DLC = " << std::dec << static_cast<int>(cmdErpm.dlc) << std::endl;

    // Flush and stop logger
    logger.stop();
    std::cout << "\n[SUCCESS] Demo completed! CSV logs successfully flushed and closed." << std::endl;

    return 0;
}
