#include "dti_can/dti_can.hpp"
#include <iostream>
#include <cassert>
#include <cmath>

static int g_tests_passed = 0;
static int g_tests_failed = 0;

#define TEST_ASSERT(cond, msg) \
    do { \
        if (!(cond)) { \
            std::cerr << "[FAIL] Line " << __LINE__ << ": " << msg << std::endl; \
            g_tests_failed++; \
        } else { \
            g_tests_passed++; \
        } \
    } while(0)

static bool floatNear(float a, float b, float eps = 0.05f) {
    return std::fabs(a - b) < eps;
}

void testTelemetryDecoding() {
    std::cout << ">>> Running testTelemetryDecoding..." << std::endl;

    // 1. General Data 1 (0x20): ERPM = 4500, Duty = 52.3%, Volt = 360V
    {
        DTI::CANFrame f{};
        f.id = DTI::Codec::makeExtendedCanId(0x20, 0x0F);
        f.dlc = 8;
        // ERPM = 4500 (0x00001194)
        f.data[0] = 0x00; f.data[1] = 0x00; f.data[2] = 0x11; f.data[3] = 0x94;
        // Duty = 52.3% -> raw = 523 (0x020B)
        f.data[4] = 0x02; f.data[5] = 0x0B;
        // Volt = 360V (0x0168)
        f.data[6] = 0x01; f.data[7] = 0x68;

        DTI::GeneralData1 gd1;
        TEST_ASSERT(DTI::Codec::decodeGeneralData1(f.data, f.dlc, gd1), "decodeGeneralData1 failed");
        TEST_ASSERT(gd1.real_erpm == 4500, "ERPM mismatch");
        TEST_ASSERT(floatNear(gd1.duty_cycle_pct, 52.3f), "Duty cycle mismatch");
        TEST_ASSERT(floatNear(gd1.input_voltage_v, 360.0f), "Voltage mismatch");
    }

    // 2. General Data 2 (0x21): AC Current = 85.4 Apk, DC Current = 42.1 Adc
    {
        DTI::CANFrame f{};
        f.id = DTI::Codec::makeExtendedCanId(0x21, 0x0F);
        f.dlc = 8;
        // AC = 85.4 -> 854 (0x0356)
        f.data[0] = 0x03; f.data[1] = 0x56;
        // DC = 42.1 -> 421 (0x01A5)
        f.data[2] = 0x01; f.data[3] = 0xA5;

        DTI::GeneralData2 gd2;
        TEST_ASSERT(DTI::Codec::decodeGeneralData2(f.data, f.dlc, gd2), "decodeGeneralData2 failed");
        TEST_ASSERT(floatNear(gd2.ac_current_apk, 85.4f), "AC current mismatch");
        TEST_ASSERT(floatNear(gd2.dc_current_adc, 42.1f), "DC current mismatch");
    }

    // 3. General Data 3 (0x22): Controller Temp = 45.5°C, Motor Temp = 60.2°C, Fault = Sensor Fault (0x08)
    {
        DTI::CANFrame f{};
        f.id = DTI::Codec::makeExtendedCanId(0x22, 0x0F);
        f.dlc = 8;
        // Ctrl Temp = 45.5 -> 455 (0x01C7)
        f.data[0] = 0x01; f.data[1] = 0xC7;
        // Motor Temp = 60.2 -> 602 (0x025A)
        f.data[2] = 0x02; f.data[3] = 0x5A;
        // Fault = 0x08
        f.data[4] = 0x08;

        DTI::GeneralData3 gd3;
        TEST_ASSERT(DTI::Codec::decodeGeneralData3(f.data, f.dlc, gd3), "decodeGeneralData3 failed");
        TEST_ASSERT(floatNear(gd3.controller_temp_c, 45.5f), "Controller temp mismatch");
        TEST_ASSERT(floatNear(gd3.motor_temp_c, 60.2f), "Motor temp mismatch");
        TEST_ASSERT(gd3.fault_code == DTI::FaultCode::SENSOR_FAULT, "Fault code mismatch");
    }

    // 4. General Data 4 (0x23): Id = 1.25 Apk, Iq = 34.50 Apk
    {
        DTI::CANFrame f{};
        f.id = DTI::Codec::makeExtendedCanId(0x23, 0x0F);
        f.dlc = 8;
        // Id = 1.25 -> raw = 125 (0x0000007D)
        f.data[0] = 0x00; f.data[1] = 0x00; f.data[2] = 0x00; f.data[3] = 0x7D;
        // Iq = 34.50 -> raw = 3450 (0x00000D7A)
        f.data[4] = 0x00; f.data[5] = 0x00; f.data[6] = 0x0D; f.data[7] = 0x7A;

        DTI::GeneralData4 gd4;
        TEST_ASSERT(DTI::Codec::decodeGeneralData4(f.data, f.dlc, gd4), "decodeGeneralData4 failed");
        TEST_ASSERT(floatNear(gd4.id_current_apk, 1.25f), "Id current mismatch");
        TEST_ASSERT(floatNear(gd4.iq_current_apk, 34.50f), "Iq current mismatch");
    }

    // 5. General Data 5 (0x24): Throttle = 75%, Brake = 0%, IO = 0x05, Driven = 1, Version = 25 (V2.5)
    {
        DTI::CANFrame f{};
        f.id = DTI::Codec::makeExtendedCanId(0x24, 0x0F);
        f.dlc = 8;
        f.data[0] = 75; // Throttle
        f.data[1] = 0;  // Brake
        f.data[2] = 0x05; // IO
        f.data[3] = 1;  // Driven
        f.data[4] = 0x00; f.data[5] = 0x02; // Limit bits
        f.data[6] = 0xFF; // Reserved
        f.data[7] = 25; // Map Version 2.5

        DTI::GeneralData5 gd5;
        TEST_ASSERT(DTI::Codec::decodeGeneralData5(f.data, f.dlc, gd5), "decodeGeneralData5 failed");
        TEST_ASSERT(gd5.throttle_pct == 75, "Throttle mismatch");
        TEST_ASSERT(gd5.brake_pct == 0, "Brake mismatch");
        TEST_ASSERT(gd5.digital_ios == 0x05, "IO mismatch");
        TEST_ASSERT(gd5.drive_enable_status == DTI::DriveEnableState::ENABLED, "Drive status mismatch");
        TEST_ASSERT(gd5.limit_status_bits == 0x0002, "Limit status mismatch");
        TEST_ASSERT(gd5.can_map_version == 25, "CAN Map Version mismatch");
    }

    // 6. General Data 6 (0x1F): Mode = CURRENT (2), Target Iq = 45.0 Apk, Pos = 180.5 deg, Still = 0
    {
        DTI::CANFrame f{};
        f.id = DTI::Codec::makeExtendedCanId(0x1F, 0x0F);
        f.dlc = 8;
        f.data[0] = 2; // CURRENT mode
        // Target Iq = 45.0 -> 450 (0x01C2)
        f.data[1] = 0x01; f.data[2] = 0xC2;
        // Pos = 180.5 -> 1805 (0x070D)
        f.data[3] = 0x07; f.data[4] = 0x0D;
        f.data[5] = 0; // Rotating

        DTI::GeneralData6 gd6;
        TEST_ASSERT(DTI::Codec::decodeGeneralData6(f.data, f.dlc, gd6), "decodeGeneralData6 failed");
        TEST_ASSERT(gd6.control_mode == DTI::ControlMode::CURRENT, "Control mode mismatch");
        TEST_ASSERT(floatNear(gd6.target_iq_apk, 45.0f), "Target Iq mismatch");
        TEST_ASSERT(floatNear(gd6.motor_position_deg, 180.5f), "Motor position mismatch");
        TEST_ASSERT(gd6.is_motor_still == DTI::MotorStillState::ROTATING, "Motor motion mismatch");
    }

    // 7. Configured & Avail AC (0x25)
    {
        DTI::CANFrame f{};
        f.id = DTI::Codec::makeExtendedCanId(0x25, 0x0F);
        f.dlc = 8;
        // Max Config = 200.0 Apk -> 2000
        DTI::Codec::writeInt16BE(&f.data[0], 2000);
        // Max Avail = 180.0 Apk -> 1800
        DTI::Codec::writeInt16BE(&f.data[2], 1800);
        // Min Config = -50.0 Apk -> -500
        DTI::Codec::writeInt16BE(&f.data[4], -500);
        // Min Avail = -40.0 Apk -> -400
        DTI::Codec::writeInt16BE(&f.data[6], -400);

        DTI::ConfigAvailAC ac;
        TEST_ASSERT(DTI::Codec::decodeConfigAvailAC(f.data, f.dlc, ac), "decodeConfigAvailAC failed");
        TEST_ASSERT(floatNear(ac.max_ac_configured_apk, 200.0f), "Max AC Config mismatch");
        TEST_ASSERT(floatNear(ac.max_ac_available_apk, 180.0f), "Max AC Avail mismatch");
        TEST_ASSERT(floatNear(ac.min_ac_configured_apk, -50.0f), "Min AC Config mismatch");
        TEST_ASSERT(floatNear(ac.min_ac_available_apk, -40.0f), "Min AC Avail mismatch");
    }

    // 8. Configured & Avail DC (0x26)
    {
        DTI::CANFrame f{};
        f.id = DTI::Codec::makeExtendedCanId(0x26, 0x0F);
        f.dlc = 8;
        // Max DC Config = 150.0 Adc -> 1500
        DTI::Codec::writeInt16BE(&f.data[0], 1500);
        // Max DC Avail = 120.0 Adc -> 1200
        DTI::Codec::writeInt16BE(&f.data[2], 1200);
        // Min DC Config = -30.0 Adc -> -300
        DTI::Codec::writeInt16BE(&f.data[4], -300);
        // Min DC Avail = -25.0 Adc -> -250
        DTI::Codec::writeInt16BE(&f.data[6], -250);

        DTI::ConfigAvailDC dc;
        TEST_ASSERT(DTI::Codec::decodeConfigAvailDC(f.data, f.dlc, dc), "decodeConfigAvailDC failed");
        TEST_ASSERT(floatNear(dc.max_dc_configured_adc, 150.0f), "Max DC Config mismatch");
        TEST_ASSERT(floatNear(dc.max_dc_available_adc, 120.0f), "Max DC Avail mismatch");
        TEST_ASSERT(floatNear(dc.min_dc_configured_adc, -30.0f), "Min DC Config mismatch");
        TEST_ASSERT(floatNear(dc.min_dc_available_adc, -25.0f), "Min DC Avail mismatch");
    }
}

void testCommandEncodingAndDecoding() {
    std::cout << ">>> Running testCommandEncodingAndDecoding (Loopback)..." << std::endl;
    const uint8_t node = 0x0F;

    // 1. Set AC Current (0x01)
    {
        auto frame = DTI::Codec::encodeSetAcCurrent(35.5f, node);
        TEST_ASSERT(frame.dlc == 2, "DLC must be 2");
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x01 failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::SET_AC_CURRENT, "Packet ID mismatch");
        TEST_ASSERT(floatNear(cmd.value.target_ac_current_apk, 35.5f), "Target AC mismatch");
    }

    // 2. Set Brake Current (0x02)
    {
        auto frame = DTI::Codec::encodeSetBrakeCurrent(20.0f, node);
        TEST_ASSERT(frame.dlc == 2, "DLC must be 2");
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x02 failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::SET_BRAKE_CURRENT, "Packet ID mismatch");
        TEST_ASSERT(floatNear(cmd.value.target_brake_current_apk, 20.0f), "Target Brake mismatch");
    }

    // 3. Set ERPM (0x03)
    {
        auto frame = DTI::Codec::encodeSetErpm(12000, node);
        TEST_ASSERT(frame.dlc == 4, "DLC must be 4");
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x03 failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::SET_ERPM, "Packet ID mismatch");
        TEST_ASSERT(cmd.value.target_erpm == 12000, "Target ERPM mismatch");
    }

    // 4. Set Relative Current (0x05)
    {
        auto frame = DTI::Codec::encodeSetRelativeCurrent(-80.5f, node);
        TEST_ASSERT(frame.dlc == 2, "DLC must be 2");
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x05 failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::SET_RELATIVE_CURRENT, "Packet ID mismatch");
        TEST_ASSERT(floatNear(cmd.value.target_relative_current, -80.5f), "Relative current mismatch");
    }

    // 5. Set Digital Output (0x07)
    {
        auto frame = DTI::Codec::encodeSetDigitalOutput(0x0A, node);
        TEST_ASSERT(frame.dlc == 1, "DLC must be 1");
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x07 failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::SET_DIGITAL_OUTPUT, "Packet ID mismatch");
        TEST_ASSERT(cmd.value.digital_output_mask == 0x0A, "Digital output mask mismatch");
    }

    // 6. Set Max AC Current (0x08)
    {
        auto frame = DTI::Codec::encodeSetMaxAcCurrent(250.0f, node);
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x08 failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::SET_MAX_AC_CURRENT, "Packet ID mismatch");
        TEST_ASSERT(floatNear(cmd.value.max_ac_current_limit_apk, 250.0f), "Max AC limit mismatch");
    }

    // 7. Set Max DC Current (0x0A)
    {
        auto frame = DTI::Codec::encodeSetMaxDcCurrent(180.0f, node);
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x0A failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::SET_MAX_DC_CURRENT, "Packet ID mismatch");
        TEST_ASSERT(floatNear(cmd.value.max_dc_current_limit_adc, 180.0f), "Max DC limit mismatch");
    }

    // 8. Set Max DC Brake Current (0x0B)
    {
        auto frame = DTI::Codec::encodeSetMaxDcBrakeCurrent(50.0f, node);
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x0B failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::SET_MAX_DC_BRAKE_CURRENT, "Packet ID mismatch");
        TEST_ASSERT(floatNear(cmd.value.max_dc_brake_limit_adc, 50.0f), "Max DC brake limit mismatch");
    }

    // 9. Drive Enable (0x0C)
    {
        auto frame = DTI::Codec::encodeDriveEnable(true, node);
        DTI::VcuCommandData cmd{};
        TEST_ASSERT(DTI::Codec::decodeVcuCommand(frame, cmd), "decode 0x0C failed");
        TEST_ASSERT(cmd.packet_id == DTI::VcuPacketId::DRIVE_ENABLE, "Packet ID mismatch");
        TEST_ASSERT(cmd.value.drive_enable_cmd == 1, "Drive enable mismatch");
    }
}

void testReceiverAndStateAggregation() {
    std::cout << ">>> Running testReceiverAndStateAggregation..." << std::endl;
    DTI::InverterReceiver receiver(0x0F);

    bool faultCallbackTriggered = false;
    DTI::FaultCode capturedFault = DTI::FaultCode::NONE;

    receiver.onFault([&](DTI::FaultCode fault, const char* /*desc*/) {
        faultCallbackTriggered = true;
        capturedFault = fault;
    });

    // Send GD1 (0x20)
    DTI::CANFrame f1{};
    f1.id = DTI::Codec::makeExtendedCanId(0x20, 0x0F);
    f1.dlc = 8;
    DTI::Codec::writeInt32BE(&f1.data[0], 5500); // 5500 RPM
    DTI::Codec::writeInt16BE(&f1.data[4], 450);  // 45.0%
    DTI::Codec::writeInt16BE(&f1.data[6], 400);  // 400V
    TEST_ASSERT(receiver.processFrame(f1), "Process f1 failed");

    // Send GD3 (0x22) with Over Temp Motor fault (0x06)
    DTI::CANFrame f3{};
    f3.id = DTI::Codec::makeExtendedCanId(0x22, 0x0F);
    f3.dlc = 8;
    DTI::Codec::writeInt16BE(&f3.data[0], 500);  // 50.0°C
    DTI::Codec::writeInt16BE(&f3.data[2], 950);  // 95.0°C
    f3.data[4] = static_cast<uint8_t>(DTI::FaultCode::OVER_TEMP_MOTOR);
    TEST_ASSERT(receiver.processFrame(f3), "Process f3 failed");

    auto state = receiver.getState();
    TEST_ASSERT(state.erpm == 5500, "Receiver ERPM mismatch");
    TEST_ASSERT(floatNear(state.duty_cycle_pct, 45.0f), "Receiver Duty mismatch");
    TEST_ASSERT(floatNear(state.input_voltage_v, 400.0f), "Receiver Voltage mismatch");
    TEST_ASSERT(floatNear(state.motor_temp_c, 95.0f), "Receiver Motor Temp mismatch");
    TEST_ASSERT(state.fault_code == DTI::FaultCode::OVER_TEMP_MOTOR, "Receiver Fault Code mismatch");
    TEST_ASSERT(faultCallbackTriggered, "Fault callback did not trigger");
    TEST_ASSERT(capturedFault == DTI::FaultCode::OVER_TEMP_MOTOR, "Captured fault mismatch");
    TEST_ASSERT(receiver.isConnected(1000), "Receiver should be connected");
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "    DTI CAN Matrix Codec Unit Tests    " << std::endl;
    std::cout << "========================================" << std::endl;

    testTelemetryDecoding();
    testCommandEncodingAndDecoding();
    testReceiverAndStateAggregation();

    std::cout << "\n----------------------------------------" << std::endl;
    std::cout << "Tests Passed: " << g_tests_passed << std::endl;
    std::cout << "Tests Failed: " << g_tests_failed << std::endl;
    std::cout << "========================================" << std::endl;

    return (g_tests_failed == 0) ? 0 : 1;
}
