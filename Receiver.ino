#include <SPI.h>
#include <LoRa.h>

#define LORA_SS    5
#define LORA_RST   14
#define LORA_DIO0  33
#define LORA_FREQ  433E6

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("=== LoRa Receiver Initializing ===");

  // กำหนด SPI พินชัดเจน (SCK=18, MISO=19, MOSI=23, CS=5)
  SPI.begin(18, 19, 23, LORA_SS);
  LoRa.setSPI(SPI);
  LoRa.setPins(LORA_SS, LORA_RST, LORA_DIO0);

  if (!LoRa.begin(LORA_FREQ)) {
    Serial.println("❌ LoRa init failed! Check wiring/frequency/power.");
    while (1);
  }

  Serial.println("✅ LoRa Receiver Ready!");
}

void loop() {
  int packetSize = LoRa.parsePacket();  // ตรวจสอบว่ามีแพ็กเก็ตเข้ามาหรือไม่
  if (packetSize) {

    String receivedData = "";
    while (LoRa.available()) {
      receivedData += (char)LoRa.read();
    }

    // แสดงผลข้อมูลที่รับมา
    Serial.println("Received Data: " +receivedData);
  }
}