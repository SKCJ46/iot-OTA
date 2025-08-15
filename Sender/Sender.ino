#include <ModbusMaster.h>     // ไลบรารีสื่อสาร Modbus RTU
#include <SPI.h>              // ไลบรารี SPI (ใช้กับ LoRa)
#include <LoRa.h>             // ไลบรารี LoRa
#include <esp_task_wdt.h>     // ไลบรารี Watchdog Timer

// ====== Pin กำหนดสำหรับ RS485 ======
#define SerialRS485_RX_PIN    26
#define SerialRS485_TX_PIN    27
#define RS485_DIRECTION_PIN   25
#define RS485_RXD_SELECT      LOW
#define RS485_TXD_SELECT      HIGH 

// ====== Modbus และ LoRa ======
#define SLAVE_ADDRESS 1
#define LORA_SS 5
#define LORA_RST 14
#define LORA_DIO0 33

// ====== Watchdog Timeout (วินาที) ======
#define WDT_TIMEOUT 5

// ====== ช่วงค่าของแต่ละเซนเซอร์ ======
#define MOI_MIN 0.0f
#define MOI_MAX 100.0f
#define TEMP_MIN -20.0f
#define TEMP_MAX 80.0f
#define EC_MIN 0.0f
#define EC_MAX 20000.0f
#define PH_MIN 0.0f
#define PH_MAX 14.0f
#define N_MIN 0.0f
#define N_MAX 2000.0f
#define P_MIN 0.0f
#define P_MAX 2000.0f
#define K_MIN 0.0f
#define K_MAX 2000.0f

// ====== ตัวแปรเก็บค่าจากเซนเซอร์ ======
ModbusMaster mb;
float s_moi, s_temp, s_ec, s_pH, s_N, s_P, s_K; // ค่าที่อ่านมาใหม่
float s_moi_filtered, s_temp_filtered, s_ec_filtered, s_pH_filtered, s_N_filtered, s_P_filtered, s_K_filtered; // ค่าที่กรองแล้ว

// ====== การตั้งค่า Modbus ======
int start_addr = 0;  // เริ่มอ่านจาก Register 0
int count = 7;       // จำนวน Register ที่อ่าน
uint8_t error = 0;   // เก็บรหัส Error ของ Modbus

// ====== ฟังก์ชันควบคุมทิศทาง RS485 ======
void preTransmission() { digitalWrite(RS485_DIRECTION_PIN, RS485_TXD_SELECT); }
void postTransmission() { digitalWrite(RS485_DIRECTION_PIN, RS485_RXD_SELECT); }

// ====== ตรวจสอบว่าข้อมูลอยู่ในช่วงหรือไม่ ======
bool isValidData(float data, float minVal, float maxVal) {
    return (data >= minVal && data <= maxVal);
}

// ====== ฟังก์ชัน Low-pass filter ======
float lowPassFilter(float newData, float oldData, float alpha) {
    return oldData + alpha * (newData - oldData);
}

// ====== TaskLoRa (ทำงานบน Core 1) ======
void TaskLoRa(void *pvParameters) {
    unsigned long sensor_interval = 0;

    while (true) {
        // ทุกๆ 10 วินาที
        if (millis() >= sensor_interval) {
            sensor_interval = millis() + 10000;

            // อ่านค่าจาก Modbus
            error = mb.readInputRegisters(start_addr, count);

            if (error == mb.ku8MBSuccess) {
                // ✅ อ่านได้ → ดึงค่ามาเก็บ
                s_moi = mb.getResponseBuffer(0) / 10.0;
                s_temp = mb.getResponseBuffer(1) / 10.0;
                s_ec = mb.getResponseBuffer(2);
                s_pH = mb.getResponseBuffer(3) / 10.0;
                s_N = mb.getResponseBuffer(4);
                s_P = mb.getResponseBuffer(5);
                s_K = mb.getResponseBuffer(6);

                // ====== ตรวจสอบและกรองทีละตัว ======
                if (!isValidData(s_moi, MOI_MIN, MOI_MAX)) s_moi_filtered = NAN;
                else s_moi_filtered = lowPassFilter(s_moi, s_moi_filtered, 0.1);

                if (!isValidData(s_temp, TEMP_MIN, TEMP_MAX)) s_temp_filtered = NAN;
                else s_temp_filtered = lowPassFilter(s_temp, s_temp_filtered, 0.1);

                if (!isValidData(s_ec, EC_MIN, EC_MAX)) s_ec_filtered = NAN;
                else s_ec_filtered = lowPassFilter(s_ec, s_ec_filtered, 0.1);

                if (!isValidData(s_pH, PH_MIN, PH_MAX)) s_pH_filtered = NAN;
                else s_pH_filtered = lowPassFilter(s_pH, s_pH_filtered, 0.1);

                if (!isValidData(s_N, N_MIN, N_MAX)) s_N_filtered = NAN;
                else s_N_filtered = lowPassFilter(s_N, s_N_filtered, 0.1);

                if (!isValidData(s_P, P_MIN, P_MAX)) s_P_filtered = NAN;
                else s_P_filtered = lowPassFilter(s_P, s_P_filtered, 0.1);

                if (!isValidData(s_K, K_MIN, K_MAX)) s_K_filtered = NAN;
                else s_K_filtered = lowPassFilter(s_K, s_K_filtered, 0.1);

            } else {
                // ❌ อ่านไม่ได้ → ทุกตัวเป็น NaN
                s_moi_filtered = NAN;
                s_temp_filtered = NAN;
                s_ec_filtered = NAN;
                s_pH_filtered = NAN;
                s_N_filtered = NAN;
                s_P_filtered = NAN;
                s_K_filtered = NAN;
                Serial.println("Error reading Modbus data! Sending NaN values.");
            }

            // ====== สร้างข้อความส่ง LoRa ======
            String output = "Moisture: " + String(s_moi_filtered) + ", "
                            + "Temperature: " + String(s_temp_filtered) + ", "
                            + "EC: " + String(s_ec_filtered) + ", "
                            + "PH: " + String(s_pH_filtered) + ", "
                            + "N: " + String(s_N_filtered) + ", "
                            + "P: " + String(s_P_filtered) + ", "
                            + "K: " + String(s_K_filtered);

            // ส่ง LoRa
            LoRa.beginPacket();
            LoRa.print(output);
            LoRa.endPacket();

            // Debug Serial
            Serial.println("Data sent via LoRa: " + output);

            // Reset WDT
            esp_task_wdt_reset();  
        }

        // หน่วง 1 วินาที
        vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
}

void setup() {
    Serial.begin(115200);
    Serial.println("Initializing...");

    // ตั้งค่าขา RS485
    pinMode(RS485_DIRECTION_PIN, OUTPUT);
    digitalWrite(RS485_DIRECTION_PIN, RS485_RXD_SELECT);

    // เริ่ม Serial2 สำหรับ Modbus
    Serial2.begin(9600, SERIAL_8N1, SerialRS485_RX_PIN, SerialRS485_TX_PIN);

    // ตั้งค่า Modbus master
    mb.begin(SLAVE_ADDRESS, Serial2);
    mb.preTransmission(preTransmission);
    mb.postTransmission(postTransmission);

    // ค่าเริ่มต้น filter = 0
    s_moi_filtered = s_temp_filtered = s_ec_filtered = s_pH_filtered =
    s_N_filtered = s_P_filtered = s_K_filtered = 0.0;

    // เริ่ม LoRa
    LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);
    if (!LoRa.begin(433E6)) {
        Serial.println("LoRa init failed!");
        while (1);
    }
    Serial.println("LoRa init successful.");

    // ตั้งค่า Watchdog Timer ให้ดูทั้ง 2 core
    esp_task_wdt_config_t wdt_config = {
        .timeout_ms = WDT_TIMEOUT * 1000,
        .idle_core_mask = (1 << 0) | (1 << 1), // core 0 และ core 1
        .trigger_panic = true,
    };
    esp_task_wdt_init(&wdt_config);
    esp_task_wdt_add(NULL);
    Serial.println("Watchdog Timer (WDT) Enabled.");

    // สร้าง Task LoRa บน core 1
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
    // ว่างเปล่า เพราะใช้ TaskLoRa ทำงานทั้งหมด
}
