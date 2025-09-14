#include <WiFi.h>
#include <esp_now.h>

// === Pin plan ===
// Steering joystick X-axis -> GPIO34 (ADC1_CH6)
// Throttle joystick Y-axis -> GPIO32 (ADC1_CH4)
const int steerXPin    = 34;  // X for servo (steering)
const int throttleYPin = 32;  // Y for motor throttle/reverse

typedef struct {
  int joyX;  // steering (0..4095)
  int joyY;  // throttle (0..4095)
} struct_message;

// ← put YOUR receiver MAC address here (6 bytes)
uint8_t receiverAddress[] = { 0x6C, 0xC8, 0x40, 0x32, 0xC0, 0x6C };

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA); // required for ESP-NOW (keeps ADC1 usable)

  // Optional but recommended for consistent ADC span
  analogReadResolution(12);       // 0..4095
  analogSetAttenuation(ADC_11db); // ~0..3.3V

  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    while (1) delay(1000);
  }

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    while (1) delay(1000);
  }
}

void loop() {
  struct_message out;
  out.joyX = analogRead(steerXPin);     // Steering joystick X only
  out.joyY = analogRead(throttleYPin);  // Throttle joystick Y only

  esp_err_t ok = esp_now_send(receiverAddress, (uint8_t*)&out, sizeof(out));
  if (ok == ESP_OK) {
    Serial.printf("Sent X=%d Y=%d\n", out.joyX, out.joyY);
  } else {
    Serial.println("Send failed");
  }
  delay(20); // ~50 Hz
}
