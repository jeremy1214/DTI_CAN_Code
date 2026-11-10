# DTI Inverter CAN Codec & Telemetry Logger (C++)

本專案依據 **`DTI_Inverter_VCU_CAN_Matrix_V2.xlsx`** 通訊矩陣規格撰寫，專為電動車控制單元（VCU）、主控電腦、上位機測試台（HIL / Dyno）、儀表顯示軟體或 ROS 節點所設計的高效能 C++ CAN 編解碼與數據記錄函式庫。

---

## 📌 功能特色

1. **完整支援 Matrix V2 所有通訊封包**：
   - **逆變器遙測接收 (Inverter Telemetry Broadcast)**：包含 8 組狀態封包（`0x1F` ~ `0x26`），完整涵蓋轉速、占空比、母線電壓、相電流、母線電流、功率元件/馬達溫度、故障碼、FOC Id/Iq、油門/煞車百分比、限制旗標等。
   - **VCU 控制指令發送 (VCU Commands)**：包含 9 組控制封包（`0x01` ~ `0x0C`），支援電流控制、轉速控制、相對電流、回充煞車、開關機使能、DO 輸出與動態電流限額設定。
2. **外部程式極致易用 (External Integration Ready)**：
   - **純標準 C++17**：零外部相依性（Zero 3rd-party dependencies），只需標準庫。
   - **硬體無關 (Hardware-Agnostic)**：不綁死任何 CAN 介面卡，可搭配 **PCAN**、**Kvaser**、**Vector**、**Linux SocketCAN**、**STM32 HAL**、**ESP32** 或 **MCP2515**。
   - **無狀態解碼 (Stateless Codec)** 與 **聚合狀態接收器 (Thread-safe State Receiver)** 雙層架構。
3. **多功能數據記錄器 (Telemetry & CAN Logger)**：
   - **物理量 CSV 記錄 (`Telemetry CSV`)**：自動記錄時間戳、電壓、電流、轉速、溫度、故障碼等工程實體值，可直接使用 Excel、MATLAB 或 Python 繪圖與分析。
   - **Raw CAN 封包記錄 (`Raw CAN CSV`)**：記錄每個封包的原始 CAN ID、DLC、十六進位 Byte 內容與封包名稱，便於通訊除錯與重播。
   - **即時終端儀表板 (`Console Monitor`)**：支援在終端機漂亮格式化輸出逆變器全車狀態。

---

## 📂 專案架構

```text
D:\Coding\DTI_CAN_Code\
├── include\
│   └── dti_can\
│       ├── dti_types.hpp        // 資料型態、列舉(ControlMode, FaultCode)、InverterState 定義
│       ├── dti_codec.hpp        // 核心編解碼器 (純靜態無狀態函式，Big-Endian/Scale 處理)
│       ├── dti_receiver.hpp     // 狀態聚合接收器 (執行緒安全、超時偵測、Event Callbacks)
│       ├── dti_logger.hpp       // 數據記錄器 (Telemetry CSV, Raw CAN CSV, 終端儀表板)
│       └── dti_can.hpp          // Master 主標頭檔
├── src\
│   ├── dti_codec.cpp            // 編解碼實作
│   ├── dti_receiver.cpp         // 接收器實作
│   └── dti_logger.cpp           // 記錄器實作
├── examples\
│   ├── test_codec.cpp           // 全封包單元測試與驗證程式
│   └── demo_integration.cpp     // 外部程式完整整合範例 (模擬接收、回呼、寫檔)
├── CMakeLists.txt               // CMake 建置設定
├── Makefile                     // g++ 快速編譯腳本
└── README.md                    // 說明手冊
```

---

## 📊 CAN Matrix 通訊協定規格整理

### 1. CAN ID 格式
DTI Inverter 採用 CAN 2.0B 29-bit 延伸訊框（Extended Frame）：
- **CAN ID 結構**：`(Packet_ID << 8) | Node_ID`
- **預設 Node ID**：`0x0F`（可於 DTI CAN Tool 中設定）。
- **位元組順序 (Endianness)**：**Big-Endian**（高位元先送，Motorola 格式）。

---

### 2. 逆變器廣播遙測 (Inverter Tx Telemetry)

| Packet ID | 訊息名稱 | 週期 | DLC | 訊號名稱 | Byte 位址 | 型態 | 解析度 (Scale) | 單位 | 說明 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`0x1F`** | General Data 6 | 20 ms | 8 | Control_Mode | Byte 0 | uint8 | 1 | Enum | 1:SPEED, 2:CURRENT, 3:CURRENT_BRAKE, 4:POS, 7:NONE |
| | | | | Target_Iq | Bytes 1-2 | int16 | 10 | Apk | 目標 Iq 電流（實際值 = Raw / 10） |
| | | | | Motor_Position | Bytes 3-4 | uint16 | 10 | deg | 馬達角度（0 ~ 359 度） |
| | | | | isMotorStill | Byte 5 | uint8 | 1 | Enum | 1: 靜止 (Still), 0: 旋轉 (Rotating) |
| **`0x20`** | General Data 1 | 10 ms | 8 | Real_ERPM | Bytes 0-3 | int32 | 1 | RPM | 馬達電轉速 eRPM (Motor RPM × 極對數) |
| | | | | Duty_Cycle | Bytes 4-5 | int16 | 10 | % | PWM 占空比 (-100.0% ~ 100.0%) |
| | | | | Input_Voltage | Bytes 6-7 | int16 | 1 | V | 直流母線輸入電壓 |
| **`0x21`** | General Data 2 | 10 ms | 8 | AC_Current | Bytes 0-1 | int16 | 10 | Apk | 交流相電流 |
| | | | | DC_Current | Bytes 2-3 | int16 | 10 | Adc | 直流母線電流 |
| **`0x22`** | General Data 3 | 100 ms| 8 | Controller_Temp | Bytes 0-1 | int16 | 10 | °C | 逆變器功率元件 (IGBT/MOSFET) 溫度 |
| | | | | Motor_Temp | Bytes 2-3 | int16 | 10 | °C | 馬達內部線圈溫度 |
| | | | | Fault_Code | Byte 4 | uint8 | 1 | Enum | 故障碼 (0x07:線路, 0x08:感知器, 0x09:CAN, 0x0A:類比) |
| **`0x23`** | General Data 4 | 20 ms | 8 | Id_Current | Bytes 0-3 | int32 | 100 | Apk | FOC 直軸電流 Id（實際值 = Raw / 100） |
| | | | | Iq_Current | Bytes 4-7 | int32 | 100 | Apk | FOC 交軸電流 Iq（實際值 = Raw / 100） |
| **`0x24`** | General Data 5 | 100 ms| 8 | Throttle_Signal | Byte 0 | int8 | 1 | % | 油門踏板訊號 |
| | | | | Brake_Signal | Byte 1 | int8 | 1 | % | 煞車踏板訊號 |
| | | | | Digital_IOs | Byte 2 | uint8 | 1 | Bitmask| 數位 I/O 狀態 (DO1~DO4, DI1~DI4) |
| | | | | Drive_Enable_Status | Byte 3 | uint8 | 1 | Enum | 1: Driven (使能中) |
| | | | | Limit_Status_Bits | Bytes 4-5 | uint16| 1 | Bitmask| 系統限制與降額旗標 |
| | | | | CAN_Map_Version | Byte 7 | uint8 | 1 | Enum | 地圖版本 (例如 25 代表 V2.5) |
| **`0x25`** | Config & Avail AC| 100 ms| 8 | Max_AC_Current_Configured | Bytes 0-1 | int16 | 10 | Apk | DTI Tool 設定的最大 AC 電流 |
| | | | | Max_AC_Current_Available | Bytes 2-3 | int16 | 10 | Apk | 當前可用最大 AC 電流 (降額後) |
| | | | | Min_AC_Current_Configured | Bytes 4-5 | int16 | 10 | Apk | DTI Tool 設定的最小 AC 電流 |
| | | | | Min_AC_Current_Available | Bytes 6-7 | int16 | 10 | Apk | 當前可用最小 AC 電流 |
| **`0x26`** | Config & Avail DC| 100 ms| 8 | Max_DC_Current_Configured | Bytes 0-1 | int16 | 10 | Adc | DTI Tool 設定的最大 DC 電流 |
| | | | | Max_DC_Current_Available | Bytes 2-3 | int16 | 10 | Adc | 當前可用最大 DC 電流 (降額後) |
| | | | | Min_DC_Current_Configured | Bytes 4-5 | int16 | 10 | Adc | DTI Tool 設定的最小 DC 電流 |
| | | | | Min_DC_Current_Available | Bytes 6-7 | int16 | 10 | Adc | 當前可用最小 DC 電流 |

---

### 3. VCU 發送控制指令 (VCU Tx Commands)

| Packet ID | 訊息名稱 | 週期/條件 | DLC | 訊號名稱 | Byte 位址 | 型態 | Scale | 單位 | 說明 |
| :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- | :--- |
| **`0x01`** | Set AC Current | 10 ms | 2 | Target_AC_Current | Bytes 0-1 | int16 | 10 | Apk | 交流控制電流指令 |
| **`0x02`** | Set Brake Current | 10 ms | 2 | Target_Brake_AC_Current | Bytes 0-1 | int16 | 10 | Apk | 回充煞車電流指令 |
| **`0x03`** | Set ERPM | 10 ms | 4 | Target_ERPM | Bytes 0-3 | int32 | 1 | RPM | 轉速控制指令 |
| **`0x05`** | Set Relative Current | 10 ms | 2 | Target_Relative_Current | Bytes 0-1 | int16 | 10 | % | 相對電流指令 (-100.0% ~ 100.0%) |
| **`0x07`** | Set Digital Output | 事件驅動 | 1 | Digital_Output_Cmd | Byte 0 | uint8 | 1 | Bitmask| DO1~DO4 腳位控制 |
| **`0x08`** | Set Max AC Current | 事件驅動 | 2 | Max_AC_Current_Limit | Bytes 0-1 | int16 | 10 | Apk | 動態設定最大 AC 電流保護限制 |
| **`0x0A`** | Set Max DC Current | 事件驅動 | 2 | Max_DC_Current_Limit | Bytes 0-1 | int16 | 10 | Adc | 動態設定最大 DC 電流保護限制 |
| **`0x0B`** | Set Max DC Brake Current| 事件驅動 | 2 | Max_DC_Brake_Limit | Bytes 0-1 | int16 | 10 | Adc | 動態設定最大 DC 回充電流限制 |
| **`0x0C`** | Drive Enable | 事件驅動 | 1 | Drive_Enable_Cmd | Byte 0 | uint8 | 1 | Enum | 1: Enable (啟動), 0: Disable (關閉) |

---

## 🚀 外部程式整合指南 (Integration Guide)

只需 `#include "dti_can/dti_can.hpp"` 即可使用所有模組。

### 1. 快速使用範例 (接收、解碼與記錄)

```cpp
#include "dti_can/dti_can.hpp"
#include <iostream>

int main() {
    // 建立接收器 (指定逆變器 Node ID 為 0x0F，若填 0 則接收所有節點)
    DTI::InverterReceiver receiver(0x0F);

    // 建立數據記錄器 (同時寫入 Telemetry CSV 與 Raw CAN CSV)
    DTI::Logger logger;
    logger.start("my_car_log.csv", DTI::Logger::LogMode::BOTH);

    // 註冊故障事件回呼 (當逆變器報錯時立即觸發)
    receiver.onFault([](DTI::FaultCode fault, const char* desc) {
        std::cerr << "警告: 逆變器發生故障 -> " << desc << std::endl;
    });

    // 當底層 CAN 驅動 (如 PCAN, SocketCAN, STM32) 收到訊框時：
    // void onHardwareCanReceived(uint32_t id, uint8_t* data, uint8_t dlc)
    uint32_t can_id = 0x200F; // Packet 0x20, Node 0x0F
    uint8_t  data[8] = { 0x00, 0x00, 0x11, 0x94, 0x01, 0x90, 0x01, 0x68 }; // 4500 RPM, 40%, 360V
    uint8_t  dlc = 8;

    // 1. 餵入接收器進行解碼與狀態更新
    receiver.processRaw(can_id, data, dlc);

    // 2. 記錄原始封包
    DTI::CANFrame frame{can_id, dlc, {0}};
    std::memcpy(frame.data, data, dlc);
    logger.logRawFrame(frame);

    // 3. 讀取最新狀態快照 (執行緒安全)
    DTI::InverterState state = receiver.getState();
    std::cout << "即時轉速: " << state.erpm << " RPM, 電壓: " << state.input_voltage_v << " V\n";

    // 4. 記錄當前全車聚合狀態
    logger.logState(state);

    // 5. 終端機即時儀表板輸出
    DTI::Logger::printDashboard(state);

    logger.stop();
    return 0;
}
```

---

### 2. VCU 指令打包發送 (Encoding)

外部程式若需向逆變器下達控制命令，直接使用 `Codec::encode*` 函數即可獲得打包好的 CAN Frame：

```cpp
// 啟動逆變器使能
DTI::CANFrame cmdEnable = DTI::Codec::encodeDriveEnable(true, 0x0F);
// sendCanToHardware(cmdEnable.id, cmdEnable.data, cmdEnable.dlc);

// 設定轉速 6000 eRPM
DTI::CANFrame cmdSpeed = DTI::Codec::encodeSetErpm(6000, 0x0F);

// 設定輸出電流 50.0 Apk
DTI::CANFrame cmdCurrent = DTI::Codec::encodeSetAcCurrent(50.0f, 0x0F);

// 設定煞車回充電流 20.0 Apk
DTI::CANFrame cmdBrake = DTI::Codec::encodeSetBrakeCurrent(20.0f, 0x0F);
```

---

## 🛠️ 編譯與測試

### 使用 g++ / Makefile 直接編譯 (最簡單)
```bash
# 編譯並執行單元測試
make test

# 編譯並執行整合範例
make demo
```

### 使用 CMake
```bash
mkdir build && cd build
cmake ..
cmake --build .

# 執行測試
./dti_test
./dti_demo
```

---

## 📈 CSV 記錄檔案範例

### 1. `*_telemetry.csv` (全車物理量時序記錄)
```csv
Timestamp_ms,Elapsed_s,Node_ID,ERPM,Input_Voltage_V,Duty_Cycle_Pct,AC_Current_Apk,DC_Current_Adc,Controller_Temp_C,Motor_Temp_C,Id_Current_Apk,Iq_Current_Apk,Target_Iq_Apk,Motor_Pos_deg,Motor_State,Throttle_Pct,Brake_Pct,Digital_IOs,Drive_Enable,Control_Mode,Fault_Code,Fault_Desc,Limit_Status_Bits,CAN_Map_Version,Max_AC_Avail_Apk,Max_DC_Avail_Adc
257603202,0.0022,0xF,2400,379.00,25.00,38.00,19.00,38.40,45.80,1.25,37.24,40.00,36.00,ROTATING,25,0,0x01,ENABLED,SPEED,0x00,"No Fault",0x0000,25,0.00,0.00
257603245,0.0372,0xF,2800,379.00,30.00,46.00,23.00,38.80,46.60,1.25,45.08,48.00,72.00,ROTATING,30,0,0x01,ENABLED,SPEED,0x00,"No Fault",0x0000,25,0.00,0.00
```

### 2. `*_raw.csv` (原始通訊封包記錄)
```csv
Timestamp_ms,Elapsed_s,CAN_ID_Hex,Packet_ID_Hex,Packet_Name,DLC,B0,B1,B2,B3,B4,B5,B6,B7
257603202,0.0003,0x200F,0x20,"General Data 1 (0x20)",8,0x00,0x00,0x09,0x60,0x00,0xFA,0x01,0x7B
257603202,0.0004,0x210F,0x21,"General Data 2 (0x21)",8,0x01,0x7C,0x00,0xBE,0x00,0x00,0x00,0x00
257603202,0.0004,0x1F0F,0x1F,"General Data 6 (0x1F)",8,0x01,0x01,0x90,0x01,0x68,0x00,0x00,0x00
```
#   D T I _ C A N _ C o d e  
 