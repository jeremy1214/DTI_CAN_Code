# DTI CAN 專案分析

## 1. 專案摘要

本專案是以 C++17 撰寫的 DTI inverter CAN 通訊函式庫。核心工作是解析 DTI CAN frame、轉換成具單位的遙測值，並提供選用的狀態聚合、故障通知、CSV 記錄與命令編碼功能。

| 項目 | 說明 |
|---|---|
| 專案類型 | C++ 函式庫，附測試程式與整合示範 |
| 語言／標準 | C++17 |
| 外部依賴 | 核心程式使用 C++ 標準函式庫；不綁定特定 CAN 硬體或驅動 |
| 通訊規格 | DTI Inverter CAN Matrix V2；預期使用 29-bit extended CAN ID |
| 主要目的 | CAN frame ↔ DTI 封包／工程實際值的轉換 |
| 外部程式應負責 | 連接 CAN 硬體、收取 frame，並決定解碼結果的輸出方式 |

若整合端只需要「輸入 raw CAN frame，輸出實際值」，最直接的 API 是 `DTI::Codec::decodeTelemetryFrame`；不必使用 Logger，也不一定要建立 Receiver。

## 2. 專案結構

目前主要程式檔案按用途分為：

| 目錄 | 檔案數 | 職責 |
|---|---:|---|
| `include/dti_can/` | 5 個 header | 公開資料型別與函式庫 API |
| `src/` | 3 個 `.cpp` 實作檔 | Codec、Receiver、Logger 的實作 |
| `examples/` | 2 個程式 | Codec 測試與整合示範 |

### 公開標頭

- [dti_can.hpp](include/dti_can/dti_can.hpp)：整合用 master header，包含其他公開 header。
- [dti_types.hpp](include/dti_can/dti_types.hpp)：CAN frame、封包 ID、列舉、解碼後資料結構與聚合狀態。
- [dti_codec.hpp](include/dti_can/dti_codec.hpp)：無狀態解碼／編碼 API。
- [dti_receiver.hpp](include/dti_can/dti_receiver.hpp)：逐 frame 接收、狀態聚合與 callback API。
- [dti_logger.hpp](include/dti_can/dti_logger.hpp)：CSV 記錄及終端機儀表板 API。

### 實作與範例

- [dti_codec.cpp](src/dti_codec.cpp)：CAN ID 解析、遙測封包解碼、VCU 命令編碼。
- [dti_receiver.cpp](src/dti_receiver.cpp)：呼叫 Codec，更新 `InverterState`，並觸發 callback。
- [dti_logger.cpp](src/dti_logger.cpp)：輸出遙測 CSV、raw CAN CSV 與 dashboard。
- [test_codec.cpp](examples/test_codec.cpp)：Codec 測試程式。
- [demo_integration.cpp](examples/demo_integration.cpp)：模擬多種遙測 frame 的整合範例。

## 3. 架構與資料流

函式庫本身不直接接觸 CAN 硬體。典型資料流如下：

```text
CAN 硬體／驅動（外部程式）
        │  CAN ID + DLC + data[0..7]
        ▼
DTI::CANFrame
        │
        ├── Codec::decodeTelemetryFrame
        │       └── 封包 ID + Node ID + TelemetryPacket（單筆封包解碼結果）
        │
        └── InverterReceiver::processFrame / processRaw
                ├── Codec 解碼
                ├── 合併到 InverterState
                └── 更新／故障 callback
                         │
                         └── Logger（選用）
```

### 主要層次

1. **資料型別層**：`CANFrame` 描述原始 CAN frame；`GeneralData*`、`ConfigAvail*` 描述單一封包的解碼結果；`InverterState` 描述多個封包累積出的狀態快照。
2. **解碼／編碼層（Codec）**：無狀態的靜態函式，負責解析 CAN ID、Big-Endian 數值與 scale。這是最適合只想解 raw data 的整合層。
3. **狀態接收層（Receiver）**：將每一筆解碼結果合併到共用狀態，提供執行緒安全的 snapshot 存取、Node ID 篩選與事件 callback。
4. **輸出層（Logger）**：可將狀態或 raw frame 寫入 CSV，也可列印 dashboard；不是解碼的必要依賴。
5. **硬體介接層（外部程式）**：使用 PCAN、SocketCAN、Vector 等驅動讀取 frame，並將資料傳入本函式庫。專案沒有實作 CAN 裝置 I/O。

## 4. CAN 封包與輸出資料

遙測解碼器支援以下封包 ID：

| Packet ID | 解碼型別 | 主要輸出欄位（實際值） |
|---|---|---|
| `0x1F` | `GeneralData6` | 控制模式、目標 Iq（A）、馬達位置（度）、馬達靜止狀態 |
| `0x20` | `GeneralData1` | eRPM、占空比（%）、輸入電壓（V） |
| `0x21` | `GeneralData2` | AC 電流（Apk）、DC 電流（Adc） |
| `0x22` | `GeneralData3` | 控制器溫度（°C）、馬達溫度（°C）、故障碼 |
| `0x23` | `GeneralData4` | Id／Iq 電流（Apk） |
| `0x24` | `GeneralData5` | 油門／煞車（%）、IO bitmask、使能狀態、限制旗標、CAN map 版本 |
| `0x25` | `ConfigAvailAC` | AC 設定／可用最大與最小電流（Apk） |
| `0x26` | `ConfigAvailDC` | DC 設定／可用最大與最小電流（Adc） |

解碼值的單位及欄位定義位於 [dti_types.hpp](include/dti_can/dti_types.hpp)；實際換算邏輯位於 [dti_codec.cpp](src/dti_codec.cpp)。例如原始 AC 電流值會依規格除以 10，FOC Id／Iq 則除以 100。

`TelemetryPacket` 是 `std::variant`，根據封包 ID 其中只會放一種解碼資料型別。`decodeTelemetryFrame` 成功時回傳 `true`；未知封包或資料長度不足時回傳 `false`。

## 5. 最精簡的外部程式使用方式

外部程式收到 frame 後填入 `CANFrame`，呼叫 Codec，再將對應型別的實際值傳給自己的 callback、佇列或訊息發布器：

```cpp
#include "dti_can/dti_can.hpp"
#include <algorithm>
#include <variant>

void onCanReceived(uint32_t canId, const uint8_t* data, uint8_t dlc)
{
    if (data == nullptr || dlc > 8) {
        return; // 實際專案可改為回報輸入錯誤
    }

    DTI::CANFrame frame{};
    frame.id = canId;
    frame.dlc = dlc;
    std::copy_n(data, dlc, frame.data);

    DTI::InverterPacketId packetId;
    uint8_t nodeId = 0;
    DTI::TelemetryPacket packet;

    if (!DTI::Codec::decodeTelemetryFrame(frame, packetId, nodeId, packet)) {
        return; // 未支援的 packet ID 或 DLC 不足
    }

    if (const auto* values = std::get_if<DTI::GeneralData1>(&packet)) {
        publishToApplication(nodeId,
                             values->real_erpm,
                             values->duty_cycle_pct,
                             values->input_voltage_v);
    }
}
```

實務上應依 `packetId` 處理所有需要的封包型別，或以 `std::visit` 分派 `TelemetryPacket`。上述範例中的 `publishToApplication` 代表整合端自己的輸出函式，不是本函式庫提供的 API。

### 直接解碼與 Receiver 的選擇

- **只需逐封包轉成實際值**：使用 `Codec::decodeTelemetryFrame`，每次輸入一筆 frame，取回該封包對應的 variant。
- **需要跨封包的最新狀態**：使用 `InverterReceiver::processRaw` 或 `processFrame`，然後讀取 `getState()`，或註冊 telemetry callback。
- **需要寫 CSV／顯示 dashboard**：再額外使用 `Logger`。

使用 Receiver 時，`processRaw(canId, data, dlc)` 的回傳值表示是否辨識並成功解碼；`getState()` 會給出多個封包欄位合併後的快照。未收到的封包欄位仍維持預設值，使用端可搭配各封包 timestamp 判斷資料新鮮度。

## 6. 建置與測試入口

- [CMakeLists.txt](CMakeLists.txt)：要求 CMake 3.14 以上，設定 C++17；定義 `dti_can_core` 靜態函式庫、`dti_can_shared` shared library、測試程式與 demo。
- [Makefile](Makefile)：提供 g++ 編譯、`make test` 與 `make demo` 目標，編譯旗標包含 `-std=c++17`。
- [README.md](README.md)：包含通訊規格表、整合流程與編譯指引。

主要可執行入口是 `examples/test_codec.cpp` 和 `examples/demo_integration.cpp`；函式庫本身沒有常駐的 `main()` 或 CAN 接收執行緒。

## 7. 整合時需留意的邊界

1. **CAN ID 格式**：通訊規格使用 `(Packet_ID << 8) | Node_ID` 的 DTI extended CAN ID。雖然 `extractPacketAndNodeId` 的註解提及 11-bit standard CAN ID，現行實作對大於 `0xFF` 的 ID 一律按 extended 格式拆解，沒有獨立的 standard-ID 解析分支。整合時應傳入符合 DTI 格式的 ID。
2. **DLC 與資料指標**：frame 最多有 8 個 data bytes；呼叫端應確保 `dlc <= 8` 且資料指標有效。每種封包的最低 DLC 驗證由 Codec 負責。
3. **標準與延伸 frame 標記**：`CANFrame` 只有 `id`、`dlc`、data 與 timestamp，沒有 standard/extended 標記。硬體 API 的 frame 類型資訊須由整合端自行確認。
4. **單筆解碼不會累積狀態**：`Codec` 每次只解一筆 frame；如需跨封包狀態，必須自行管理或使用 `InverterReceiver`。
5. **shared library 的平台整合**：CMake 有建立 `dti_can_shared` 目標，但沒有安裝／匯出 package 設定或 C ABI wrapper。跨語言使用或 Windows DLL 部署需要額外的介接工作；目前最直接的使用對象是 C++ 程式。

## 8. 總結

此專案的核心是硬體無關的 DTI CAN Codec，周邊的 Receiver 與 Logger 是可選元件。對只需要「raw CAN data → 實際值 → 傳給其他程式模組」的需求，建議由外部程式處理 CAN 收取及輸出，直接以 `CANFrame` 呼叫 `Codec::decodeTelemetryFrame`，並將回傳的 packet 型別轉交給自己的發布介面。
