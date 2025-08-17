
#include <WiFi.h>
#include <WiFiManager.h>
#include <EEPROM.h>
#include <HTTPClient.h>
#include <Update.h>
#include <ArduinoJson.h>

#include <ModbusMaster.h>
#include <SPI.h>
#include <LoRa.h>

#include <esp_task_wdt.h>   // ESP32 Arduino Core 3.x (ESP-IDF v5.x)

// ---------------- CONFIG ----------------
#define EEPROM_SIZE           8                                // เก็บ "G1\0" + เผื่อ
#define CURRENT_VERSION       "1.0.3"                          // <-- ตั้งเวอร์ชันเฟิร์มแวร์นี้
const char* versionInfoUrl =  "http://20.2.91.100/version.json"; // <-- เปลี่ยนเป็น IP/โดเมน VM

// RS485 (UART2)
#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define RS485_DIRECTION_PIN   25
#define RS485_RXD_SELECT      LOW
#define RS485_TXD_SELECT      HIGH

// LoRa (SX127x)
#define LORA_SS               5
#define LORA_RST              14
#define LORA_DIO0             33
#define LORA_FREQ_HZ          433E6

// Watchdog (เฝ้า loopTask + TaskLoRa)
#define WDT_TIMEOUT_S         20   // 20s เผื่อ OTA/HTTP/Modbus ช้า

// เกณฑ์กรองค่า (นอกช่วง -> NaN)
#define MOI_MIN  0.0f
#define MOI_MAX  100.0f
#define TEMP_MIN -40.0f
#define TEMP_MAX  85.0f
#define EC_MIN   0.0f
#define EC_MAX   50000.0f
#define PH_MIN   0.0f
#define PH_MAX   14.0f
#define N_MIN    0.0f
#define N_MAX    2000.0f
#define P_MIN    0.0f
#define P_MAX    2000.0f
#define K_MIN    0.0f
#define K_MAX    2000.0f

// ---------------- GLOBALS ----------------
String NodeID = "G1";   // เก็บ NodeID เป็นสตริง เช่น "G1","D3"
ModbusMaster mb;

// ---------- Utilities ----------
static inline bool inRange(float v, float lo, float hi) { return (v >= lo && v <= hi); }

// แสดงทศนิยม/NaN (แก้ ambiguous ด้วย cast)
String fmtFloatOrNaN(float v, uint8_t prec = 2) {
  if (isnan(v)) return "NaN";
  return String(v, (unsigned int)prec);
}
// ส่งเป็นจำนวนเต็ม ถ้า NaN ให้พิมพ์ "NaN"
String fmtIntOrNaN(float v) {
  if (isnan(v)) return "NaN";
  long iv = lroundf(v);
  return String(iv);
}

// อนุญาตเฉพาะชุด NodeID นี้
bool isValidNodeId(String id) {
  id.trim(); id.toUpperCase();
  const char* allowed[] = {"G1","G2","G3","D1","D2","D3","D4"};
  for (auto s : allowed) if (id == s) return true;
  return false;
}

// โหลด NodeID จาก EEPROM (3 byte: 'G','1','\0')
void loadNodeID() {
  EEPROM.begin(EEPROM_SIZE);
  char buf[4] = {0,0,0,0};
  buf[0] = EEPROM.read(0);
  buf[1] = EEPROM.read(1);
  buf[2] = EEPROM.read(2);
  EEPROM.end();

  String id = String(buf);
  id.trim(); id.toUpperCase();
  NodeID = isValidNodeId(id) ? id : "G1";
}

// บันทึก NodeID ลง EEPROM
void saveNodeID(const String& id) {
  String v = id; v.trim(); v.toUpperCase();
  EEPROM.begin(EEPROM_SIZE);
  EEPROM.write(0, v.length() > 0 ? v[0] : 'G');
  EEPROM.write(1, v.length() > 1 ? v[1] : '1');
  EEPROM.write(2, 0);   // null terminator
  EEPROM.commit();
  EEPROM.end();
}

// RS485 ทิศทางส่ง/รับ
void preTransmission()  { digitalWrite(RS485_DIRECTION_PIN, RS485_TXD_SELECT); }
void postTransmission() { digitalWrite(RS485_DIRECTION_PIN, RS485_RXD_SELECT); }

// ---------------- OTA ----------------
void doOTA(const String& firmwareURL) {
  HTTPClient http;
  http.setTimeout(7000); // ป้องกันบล็อกนาน
  http.begin(firmwareURL);
  int httpCode = http.GET();

  esp_task_wdt_reset();   // ป้อน WDT ระหว่างงานยาว

  if (httpCode == 200) {
    int contentLength = http.getSize();
    WiFiClient* stream = http.getStreamPtr();

    if (Update.begin(contentLength)) {
      // การเขียนสตรีมเป็นงานบล็อก อาจนาน → ป้อน WDT ก่อน/หลัง
      esp_task_wdt_reset();
      size_t written = Update.writeStream(*stream);
      esp_task_wdt_reset();

      if (written == (size_t)contentLength) {
        Serial.println("✅ Firmware written successfully.");
        if (Update.end()) {
          Serial.println("🔁 Restarting...");
          ESP.restart();
        }
      } else {
        Serial.printf("❌ Write failed: %u/%u bytes\n", (unsigned)written, (unsigned)contentLength);
      }
    } else {
      Serial.println("❌ Not enough space for OTA.");
    }
  } else {
    Serial.printf("❌ Failed to fetch firmware.bin. HTTP %d\n", httpCode);
  }

  http.end();
  esp_task_wdt_reset();
}

void checkForOTAUpdate() {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.setTimeout(7000); // กันค้าง
  Serial.println("🔍 Checking OTA...");

  esp_task_wdt_reset();

  http.begin(versionInfoUrl);
  int httpCode = http.GET();

  esp_task_wdt_reset();

  if (httpCode == 200) {
    String payload = http.getString();
    Serial.println("📄 Version JSON: " + payload);

    esp_task_wdt_reset();

    DynamicJsonDocument doc(512);
    if (deserializeJson(doc, payload)) {
      Serial.println("❌ JSON Parse failed");
      http.end();
      return;
    }

    if (doc.containsKey("version") && doc.containsKey("url")) {
      String latest = doc["version"].as<String>();
      String binUrl = doc["url"].as<String>();

      Serial.println("Latest Version: " + latest);
      Serial.println("Current Version: " + String(CURRENT_VERSION));

      if (latest != CURRENT_VERSION) {
        Serial.println("⬆️ New version found! Start OTA...");
        http.end();
        esp_task_wdt_reset();
        doOTA(binUrl);      // OTA สำเร็จจะรีสตาร์ต
        return;
      } else {
        Serial.println("✅ Already up-to-date.");
      }
    } else {
      Serial.println("⚠ JSON missing 'version' or 'url' keys.");
    }
  } else {
    Serial.printf("❌ Version check failed. HTTP %d\n", httpCode);
  }
  http.end();
}

// ---------------- WiFi + NodeID (Input + Dropdown) ----------------
void connectWiFiWithNodeID() {
  loadNodeID();  // โหลดค่า NodeID จาก EEPROM

  // ช่องพิมพ์ NodeID (id = "node_id")
  char idBuffer[4];
  snprintf(idBuffer, sizeof(idBuffer), "%s", NodeID.c_str());
  WiFiManagerParameter param_node("node_id", "Node ID (G1..G3 / D1..D4)", idBuffer, 4);

  // ดรอปดาวน์ Quick pick (เลือกแล้วเติมลงช่องพิมพ์ผ่าน JS)
  const char *selectHTML =
    "<div><label>Quick pick</label>"
    "<select onchange=\"document.getElementById('node_id').value=this.value;\">"
      "<option value=''>--เลือก--</option>"
      "<option value='G1'>G1</option>"
      "<option value='G2'>G2</option>"
      "<option value='G3'>G3</option>"
      "<option value='D1'>D1</option>"
      "<option value='D2'>D2</option>"
      "<option value='D3'>D3</option>"
      "<option value='D4'>D4</option>"
    "</select></div><br/>";

  WiFiManagerParameter dd_label(selectHTML);

  WiFiManager wm;
  wm.addParameter(&param_node);
  wm.addParameter(&dd_label);

  String apName = "ESP_SmartFarm";  // ชื่อ AP ตายตัว
  bool ok = wm.autoConnect(apName.c_str());



  if (!ok) {
    Serial.println("❌ WiFi connect failed, reboot...");
    delay(2000);
    ESP.restart();
  }

  // อ่านค่าหลัง Save
  String newId = param_node.getValue();
  newId.trim(); newId.toUpperCase();

  if (isValidNodeId(newId) && newId != NodeID) {
    saveNodeID(newId);
    NodeID = newId;
    Serial.println("💾 Saved NodeID to EEPROM: " + NodeID);
  } else {
    Serial.println("ℹ️ NodeID unchanged or invalid, keep: " + NodeID);
  }

  Serial.print("✅ WiFi connected: ");
  Serial.println(WiFi.localIP());
  Serial.println("⭐ NodeID = " + NodeID);
}

// ---------------- TaskLoRa: อ่าน Modbus + ส่ง LoRa(Text) ----------------
void TaskLoRa(void *pv) {
  // ผูก TaskLoRa กับ WDT (ต้องเรียกภายในคอนเท็กซ์ของ task นี้)
  esp_task_wdt_add(NULL);

  const uint32_t interval_ms = 10000; // ส่งทุก 10s
  uint32_t next_ms = 0;

  const int start_addr = 0;
  const int count      = 7;

  for (;;) {
    if (millis() >= next_ms) {
      next_ms = millis() + interval_ms;

      // ========== อ่าน Modbus ==========
      uint8_t err = mb.readInputRegisters(start_addr, count);

      // เริ่มจาก NaN ทั้งหมด หากอ่านพลาด -> จะคง NaN เฉพาะตัว
      float v_moi = NAN, v_temp = NAN, v_ec = NAN, v_pH = NAN, v_N = NAN, v_P = NAN, v_K = NAN;

      if (err == mb.ku8MBSuccess) {
        v_moi  = mb.getResponseBuffer(0) / 10.0f;  // %
        v_temp = mb.getResponseBuffer(1) / 10.0f;  // °C
        v_ec   = mb.getResponseBuffer(2);          // raw/µS ตามเซ็นเซอร์
        v_pH   = mb.getResponseBuffer(3) / 10.0f;  // pH
        v_N    = mb.getResponseBuffer(4);
        v_P    = mb.getResponseBuffer(5);
        v_K    = mb.getResponseBuffer(6);

        // กรองนอกช่วง -> NaN เฉพาะตัว
        if (!inRange(v_moi , MOI_MIN , MOI_MAX )) v_moi  = NAN;
        if (!inRange(v_temp, TEMP_MIN, TEMP_MAX)) v_temp = NAN;
        if (!inRange(v_ec  , EC_MIN  , EC_MAX  )) v_ec   = NAN;
        if (!inRange(v_pH  , PH_MIN  , PH_MAX  )) v_pH   = NAN;
        if (!inRange(v_N   , N_MIN   , N_MAX   )) v_N    = NAN;
        if (!inRange(v_P   , P_MIN   , P_MAX   )) v_P    = NAN;
        if (!inRange(v_K   , K_MIN   , K_MAX   )) v_K    = NAN;
      } else {
        Serial.printf("❌ Modbus read error: %u\n", err);
      }

      // ========== ฟอร์แมตเป็น Text ตามที่ Pi จะ parse ==========
      // ตัวอย่าง: sensor_id:G2, N:30, P:15, K:120, PH:6.2, Temperature:28, Moisture:80, EC:2.1
      String payload =
        "sensor_id:"    + NodeID +
        ", N:"          + fmtIntOrNaN(v_N) +
        ", P:"          + fmtIntOrNaN(v_P) +
        ", K:"          + fmtIntOrNaN(v_K) +
        ", PH:"         + fmtFloatOrNaN(v_pH, 1) +
        ", Temperature:" + fmtIntOrNaN(v_temp) +
        ", Moisture:"   + fmtIntOrNaN(v_moi) +
        ", EC:"         + fmtFloatOrNaN(v_ec, 1);

      // ส่ง LoRa
      LoRa.beginPacket();
      LoRa.print(payload);
      LoRa.endPacket();

      Serial.println("📤 LoRa(Text): " + payload);

      // ป้อน WDT สำหรับ TaskLoRa
      esp_task_wdt_reset();
    }

    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// ---------------- setup / loop ----------------
void setup() {
  Serial.begin(115200);
  delay(200);
  Serial.println("\n=== ESP32 Sender + OTA + NodeID ===");

  // RS485 direction
  pinMode(RS485_DIRECTION_PIN, OUTPUT);
  digitalWrite(RS485_DIRECTION_PIN, RS485_RXD_SELECT);

  // UART2 for RS485
  Serial2.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);

  // Modbus master
  mb.begin(1, Serial2);  // slave id = 1 (ปรับตามอุปกรณ์จริง)
  mb.preTransmission(preTransmission);
  mb.postTransmission(postTransmission);

  // LoRa
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
  if (!LoRa.begin(LORA_FREQ_HZ)) {
    Serial.println("❌ LoRa init failed!");
  } else {
    Serial.println("✅ LoRa init OK");
  }

  // WiFi + หน้า Config NodeID
  connectWiFiWithNodeID();

  // เช็ค OTA ครั้งแรกหลังเชื่อม WiFi
  checkForOTAUpdate();

  // ----- WDT: เคลียร์ของเดิม + ตั้งค่าใหม่ (กัน TWDT already initialised) -----
  esp_task_wdt_deinit();
  esp_task_wdt_config_t cfg = {
    .timeout_ms = WDT_TIMEOUT_S * 1000,  // ms
    .idle_core_mask = 0,                 // ไม่เฝ้า idle task
    .trigger_panic = true
  };
  esp_task_wdt_init(&cfg);
  esp_task_wdt_add(NULL);                // เฝ้า loopTask
  Serial.println("🐶 WDT enabled (loopTask + TaskLoRa).");

  // สร้าง TaskLoRa (core 1)
  xTaskCreatePinnedToCore(
    TaskLoRa,
    "TaskLoRa",
    4096,
    NULL,
    1,
    NULL,
    1
  );
}

void loop() {
  // เช็ค OTA เป็นระยะ (ทุก 10 นาที)
  static uint32_t nextCheck = 0;
  if (millis() > nextCheck) {
    nextCheck = millis() + 10UL * 60UL * 1000UL;
    checkForOTAUpdate();
  }

  // ป้อน WDT สำหรับ loopTask
  esp_task_wdt_reset();
  delay(100);
}
