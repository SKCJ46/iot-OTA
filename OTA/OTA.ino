#include <WiFi.h>
#include <WiFiManager.h>   // ไลบรารีสำหรับจัดการการเชื่อมต่อ WiFi พร้อมหน้า Config
#include <EEPROM.h>        // ใช้บันทึก/โหลดข้อมูล Sensor ID ลงในหน่วยความจำถาวร
#include <HTTPClient.h>    // ใช้ดึงข้อมูลจากเว็บเซิร์ฟเวอร์ผ่าน HTTP
#include <Update.h>        // ใช้สำหรับอัปเดตเฟิร์มแวร์ OTA
#include <ArduinoJson.h>   // ใช้แปลงและอ่านข้อมูล JSON

// ---------------- CONFIG ----------------
#define EEPROM_SIZE 4                   // ขนาดพื้นที่ EEPROM (4 bytes)
const char* versionInfoUrl = "http://172.20.10.7/version.json"; // URL JSON ที่เก็บข้อมูลเวอร์ชันและลิงก์ OTA
#define CURRENT_VERSION "1.0.0"         // เวอร์ชันปัจจุบันของเฟิร์มแวร์
// -----------------------------------------

int NumberID = 1; // เก็บหมายเลข Sensor ID (1–7)

// ---------- ฟังก์ชันโหลด ID จาก EEPROM ----------
void loadID() {
  EEPROM.begin(EEPROM_SIZE);          // เริ่มการใช้งาน EEPROM
  NumberID = EEPROM.read(0);          // อ่านค่าจากตำแหน่ง Byte 0
  if (NumberID < 1 || NumberID > 7) { // ถ้าค่าไม่ถูกต้อง (น้อยกว่า 1 หรือมากกว่า 7)
    NumberID = 1;                     // ให้ตั้งค่าเป็น 1
  }
  EEPROM.end();                       // ปิดการใช้งาน EEPROM
}

// ---------- ฟังก์ชันบันทึก ID ไป EEPROM ----------
void saveID(int id) {
  EEPROM.begin(EEPROM_SIZE); // เริ่มการใช้งาน EEPROM
  EEPROM.write(0, id);       // บันทึกค่าที่ตำแหน่ง Byte 0
  EEPROM.commit();           // ยืนยันการเขียนลงหน่วยความจำ
  EEPROM.end();              // ปิดการใช้งาน EEPROM
}

// ---------- ฟังก์ชันเชื่อมต่อ WiFi + Config ID ----------
void connectWiFi() {
  WiFiManager wm; // สร้างออบเจ็กต์ WiFiManager

  // โหลดค่า ID ล่าสุดจาก EEPROM
  loadID();

  // เตรียมค่าปัจจุบันของ ID เพื่อใส่ในช่องกรอกหน้า Config
  char idBuffer[3];
  snprintf(idBuffer, sizeof(idBuffer), "%d", NumberID);

  // สร้างช่องกรอก ID ในหน้า Config ของ WiFiManager
  WiFiManagerParameter custom_id("sensor_id", "Sensor ID (1-7)", idBuffer, 3);
  wm.addParameter(&custom_id);

  // ตั้งชื่อ Access Point (AP) ชั่วคราวเป็น ESP_IDx (x = Sensor ID)
  String apName = "ESP_ID" + String(NumberID);

  // เริ่มเชื่อมต่อ WiFi (ถ้าล้มเหลว จะเข้าสู่หน้า Config ให้ผู้ใช้กรอกข้อมูล)
  bool res = wm.autoConnect(apName.c_str());

  if (!res) {
    Serial.println("❌ Failed to connect. Rebooting...");
    delay(3000);
    ESP.restart(); // รีสตาร์ทบอร์ด
  }

  // ตรวจว่าผู้ใช้กรอก Sensor ID ใหม่หรือไม่
  int newID = atoi(custom_id.getValue());
  if (newID >= 1 && newID <= 7 && newID != NumberID) {
    saveID(newID);        // บันทึก ID ใหม่ลง EEPROM
    NumberID = newID;     // อัปเดตค่าปัจจุบัน
    Serial.println("💾 Sensor ID saved to EEPROM.");
  }

  // แสดงสถานะการเชื่อมต่อ
  Serial.println("✅ WiFi connected: " + WiFi.localIP().toString());
  Serial.println("⭐ Sensor ID = " + String(NumberID));
}

// ---------- ฟังก์ชันตรวจสอบและอัปเดต OTA ----------
void checkForOTAUpdate() {
  if (WiFi.status() != WL_CONNECTED) return; // ถ้า WiFi ยังไม่เชื่อมต่อให้หยุด

  HTTPClient http;
  Serial.println("🔍 Checking OTA...");

  // เปิดการเชื่อมต่อไปยัง URL ที่เก็บข้อมูลเวอร์ชัน
  http.begin(versionInfoUrl);
  int httpCode = http.GET(); // ขอข้อมูลจากเซิร์ฟเวอร์

  if (httpCode == 200) { // ถ้าดึงข้อมูลสำเร็จ
    String payload = http.getString();
    Serial.println("📄 Version JSON: " + payload);

    DynamicJsonDocument doc(512);
    DeserializationError err = deserializeJson(doc, payload);

    if (err) {
      Serial.println("❌ JSON Parse failed");
      return;
    }

    // ตรวจว่ามี key "version" และ "url" หรือไม่
    if (doc.containsKey("version") && doc.containsKey("url")) {
      String latestVersion = doc["version"];
      String binUrl        = doc["url"];

      Serial.println("Latest Version: " + latestVersion);
      Serial.println("Current Version: " + String(CURRENT_VERSION));

      // ถ้าเวอร์ชันใน JSON ไม่ตรงกับ CURRENT_VERSION ให้ทำ OTA
      if (latestVersion != CURRENT_VERSION) {
        Serial.println("⬆️ New version found! Starting OTA...");
        doOTA(binUrl);
      } else {
        Serial.println("✅ Already up-to-date.");
      }
    } else {
      Serial.println("⚠ JSON doesn't contain version/url keys.");
    }
  } else {
    Serial.println("❌ Version check failed. HTTP " + String(httpCode));
  }
  http.end();
}

// ---------- ฟังก์ชันทำ OTA Update ----------
void doOTA(const String& firmwareURL) {
  HTTPClient http;
  http.begin(firmwareURL);
  int httpCode = http.GET();

  if (httpCode == 200) { // ถ้าดึงไฟล์ bin สำเร็จ
    int contentLength = http.getSize();
    WiFiClient* stream = http.getStreamPtr();

    if (Update.begin(contentLength)) { // เริ่มการอัปเดต OTA
      size_t written = Update.writeStream(*stream);
      if (written == contentLength) { // ถ้าเขียนข้อมูลครบ
        Serial.println("✅ Firmware written successfully.");
        if (Update.end()) {           // จบการอัปเดต
          Serial.println("🔁 Restarting...");
          ESP.restart();              // รีสตาร์ทบอร์ด
        }
      } else {
        Serial.println("❌ Write failed: size mismatch.");
      }
    } else {
      Serial.println("❌ Not enough space for OTA.");
    }
  } else {
    Serial.println("❌ Failed to fetch firmware.bin. HTTP " + String(httpCode));
  }

  http.end();
}

void setup() {
  Serial.begin(115200);
  connectWiFi();        // เชื่อมต่อ WiFi และตั้งค่า Sensor ID
  delay(1000);
  checkForOTAUpdate();  // ตรวจสอบเวอร์ชันและทำ OTA ถ้าจำเป็น
}

void loop() {
  // วนลูปหลักของโปรแกรม
}
