#include <esp_now.h>
#include <WiFi.h>
#include <ESP32Servo.h>

Servo servoX;
Servo servoY;

// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  int joyX;
  int joyY;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t *mac_addr, esp_now_send_status_t *status, int len) {
if (len == sizeof(myData)){
  memcpy(&myData, status, sizeof(myData));
  Serial.print("Received X:");
  Serial.print("incomingData.joyX");
  Serial.print(" | Y:");
  Serial.println(myData.joyY);

  int servoAngleX = map(myData.joyX, 0, 4095, 0 , 180);
  int servoAngleY = map(myData.joyY, 0, 4095, 0 , 180);

  servoAngleX = constrain(servoAngleX, 0, 180);
 // servoAngleY = constrain(servoAngleY, 0, 180);

  Serial.print("Servo X Angle:");
  Serial.println(servoAngleX);
  //Serial.print("Servo Y Angle:");
  //Serial.println(servoAngleY);

  servoX.write(servoAngleX);
  servoY.write(servoAngleY);
}
else{
  Serial.println("Received data length mismatch.");
}
}

void setup() {
  // Initialize Serial Monitor
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  
  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
  servoX.attach(18);
  servoY.attach(25);

  Serial.println("Receiver is ready to receive data.");
}
 
void loop() {
delay(100);
}
